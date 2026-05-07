#include "imu.h"
#include <cmath>

#define SDA_PIN 8
#define SCL_PIN 9

ICM_20948_I2C myICM;

// Filter constants
const float gyro_trust = 0.96; // Trust gyro 96%, trust accel/mag 4%
const float mag_trust = 0.005;  // Trust magnetometer corrections at 10% to prevent drift over time
unsigned long lastTime = 0;
EulerAngles currentAngles = {0, 0, 0};


SemaphoreHandle_t i2cMutex;

void initIMU() {
    i2cMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    myICM.begin(Wire, 1); // Assuming AD0 is GND
    while (myICM.status != ICM_20948_Stat_Ok) {
        delay(500);
    }
    lastTime = micros();
}

float magX_min = 1000, magX_max = -1000;
float magY_min = 1000, magY_max = -1000;

void calibrateMag() {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
      myICM.getAGMT(); // Talk to sensor
      xSemaphoreGive(i2cMutex); // Give the "key" back
    }
    float x = myICM.magX();
    float y = myICM.magY();

    if (x < magX_min) magX_min = x;
    if (x > magX_max) magX_max = x;
    if (y < magY_min) magY_min = y;
    if (y > magY_max) magY_max = y;

    Serial.print("X_Min:"); Serial.print(magX_min);
    Serial.print(" X_Max:"); Serial.print(magX_max);
    Serial.print(" | Y_Min:"); Serial.print(magY_min);
    Serial.print(" Y_Max:"); Serial.println(magY_max);
}


EulerAngles getOrientation() {
    if (myICM.dataReady()) {
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
        myICM.getAGMT(); // Talk to sensor
        xSemaphoreGive(i2cMutex); // Give the "key" back
      }

        // 1. Calculate Delta Time
        unsigned long currentTime = micros();
        float dt = FastLoopTime / 1000.0; // Convert ms to seconds
        lastTime = currentTime;

        calcRollPitch(dt);
        calcYaw(dt);

        
      }
    return currentAngles;
}

void calcRollPitch(float dt){
        // 2. Pitch and Roll from Accelerometer (Trigonometry)
        // atan2 returns radians, we convert to degrees
        float accRoll = atan2(myICM.accY(), myICM.accZ()) * 180.0 / M_PI;
        float accPitch = atan2(-myICM.accX(), sqrt((myICM.accY() * myICM.accY()) + (myICM.accZ() * myICM.accZ()))) * 180.0 / M_PI;
        // 3. Integrate Gyroscope (Angle = Velocity * Time)
        // The ICM-20948 provides dps (degrees per second)
        currentAngles.roll  = gyro_trust * (currentAngles.roll + myICM.gyrX() * dt) + (1.0 - gyro_trust) * accRoll;
        currentAngles.pitch = gyro_trust * (currentAngles.pitch + myICM.gyrY() * dt) + (1.0 - gyro_trust) * accPitch;
}

void calcYaw(float dt){
  float magOffsetX = 22.65;
  float magOffsetY = 14.17; 

  float magX = myICM.magX() - magOffsetX;
  float magY = myICM.magY() - magOffsetY;

  float heading = atan2(magY, magX) * 180.0 / M_PI;
        
  // Normalize heading to 0-360
  if (heading < 0) heading += 360;

  float gyroRateZ = myICM.gyrZ(); // Degrees per second

  // 1. Calculate the raw difference
  float deltaYaw = heading - currentAngles.yaw;

  // 2. "Wrap" the difference so it takes the shortest path
  // This prevents the "slow ramp" across the 360/0 boundary
  if (deltaYaw > 180)  deltaYaw -= 360;
  if (deltaYaw < -180) deltaYaw += 360;

  // 3. Apply the filter using the shortest distance
  // Trust the Gyro for movement, use the wrapped Mag delta for correction
  currentAngles.yaw = (currentAngles.yaw + gyroRateZ * dt) + (mag_trust * deltaYaw);

  // 4. Keep the final result between 0 and 360
  if (currentAngles.yaw >= 360) currentAngles.yaw -= 360;
  if (currentAngles.yaw < 0)    currentAngles.yaw += 360;
}

float get_gyrY(){
    return myICM.gyrY();
}

float get_gyrZ(){
    return myICM.gyrZ();
}

float get_gyrX(){
    return myICM.gyrX();
}