#include <FSM.h>

uint8_t rawData[BUFFER_SIZE];

IMU* imu = nullptr;
MotorThrottle* motorThrottle = nullptr;
unsigned long lastDebugTime = 0;

RCInputData* inputData = new RCInputData();

PID pidRoll(kp, ki, kd, 400);  
PID pidPitch(kp, ki, kd, 400);
PID pidYaw(kp, ki, kd, 400);

unsigned long last_pid_time = 0;


void FSM(uint8_t *stage){
    switch (*stage)
    {
    case  stage1:
        Stage1();
        *stage = stage2;
        break;
    case stage2:
        Stage2();
        break;
    default:
        break;
    }
}

/*
    Initiallizes communication with the IMU sensor
*/
void Stage1() {
    motorThrottle = new MotorThrottle();
    if (motorThrottle == nullptr) {
      Serial.println("Memory Allocation Failed for MotorThrottle!");
      while(1);
  }

    resetThrottle();

    imu = new IMU(0x00);
    if (imu == nullptr) {
      Serial.println("Memory Allocation Failed for IMU!");
      while(1);
    }

    init_imu(imu);

    initData();

}

/*
    Flying routine:
    Reads data, processes it, and updates motor throttle accordingly
*/
void Stage2() {
    imu->readData(rawData);

    unsigned long now = micros();
    last_pid_time = now;

    set_current_angle();

    angle_setpoints setpoints = calc_setpoints();

    inputData->roll_stick  = pidRoll.update(setpoints.setpoint_rate_roll, imu->getXgyro(), dt);
    inputData->pitch_stick = pidPitch.update(setpoints.setpoint_rate_pitch, imu->getYgyro(), dt);
    inputData->yaw_stick   = pidYaw.update(inputData->yaw_stick, imu->getZgyro(), dt);

    calcThrottle();

    #ifdef DEBUG
    // Print logic (unchanged)
    if (millis() - lastDebugTime > 50) {
        lastDebugTime = millis();
        printThrottle();
        printAngle();
    }
    #endif
}

void set_current_angle(){
    // --- 1. CALCULATE ACCELEROMETER ANGLES ---
    // (Requires math.h)
    float accRoll  = atan2(imu->getYaccel(), imu->getZaccel()) * 57.296; 
    float accPitch = atan2(-imu->getXaccel(), sqrt(imu->getYaccel()*imu->getYaccel() + imu->getZaccel()*imu->getZaccel())) * 57.296;

    // Combine Gyro Integration (fast) with Accel (stable)
    imu->set_current_angle_roll(accRoll, dt);
    imu->set_current_angle_pitch(accPitch, dt);
}

angle_setpoints calc_setpoints() {
    // HTML sends -500 to 500. We want -45 to 45 degrees.
    // Multiplier: 45 / 500 = 0.09
    float target_angle_roll  = inputData->roll_stick * 0.09; 
    float target_angle_pitch = inputData->pitch_stick * 0.09;

    // --- 2. CALCULATE ANGLE ERROR ---
    float error_roll  = target_angle_roll - imu->getCurrentAngleRoll();
    float error_pitch = target_angle_pitch - imu->getCurrentAnglePitch();

    float setpoint_rate_roll  = error_roll * level_strength;
    float setpoint_rate_pitch = error_pitch * level_strength;

    angle_setpoints setpoints;
    setpoints.setpoint_rate_roll = setpoint_rate_roll;
    setpoints.setpoint_rate_pitch = setpoint_rate_pitch;
    return setpoints;
}

void init_imu(IMU* imu)
{
    imu->begin();
    Serial.println("IMU Initialized Successfully");
}

void resetThrottle(){
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
        
    motorThrottle->M_FR = ThrottleLowerBound;
    motorThrottle->M_RR = ThrottleLowerBound;
    motorThrottle->M_RL = ThrottleLowerBound;
    motorThrottle->M_FL = ThrottleLowerBound;
}

void calcThrottle(){
    /*
        Makes the code more readable by creating references to the motorThrottle and inputData objects
        (also saves nanoseconds)
    */
    MotorThrottle &mt = *motorThrottle;
    RCInputData &input = *inputData;

        mt.M_FR = input.throttle_raw - input.pitch_stick - input.roll_stick + input.yaw_stick;
        mt.M_FL = input.throttle_raw - input.pitch_stick + input.roll_stick - input.yaw_stick;
        mt.M_RR = input.throttle_raw + input.pitch_stick - input.roll_stick - input.yaw_stick;
        mt.M_RL = input.throttle_raw + input.pitch_stick + input.roll_stick + input.yaw_stick;

    /*
        Constrains the motor throttle values to be within the defined bounds
    */
    mt.M_FR = constrain(mt.M_FR, ThrottleLowerBound, ThrottleUpperBound);
    mt.M_RR = constrain(mt.M_RR, ThrottleLowerBound, ThrottleUpperBound);
    mt.M_RL = constrain(mt.M_RL, ThrottleLowerBound, ThrottleUpperBound);
    mt.M_FL = constrain(mt.M_FL, ThrottleLowerBound, ThrottleUpperBound); 
}

void initData() {
    // Initialize input data here if needed
    inputData->throttle_raw = ThrottleLowerBound;
    inputData->roll_stick = 0;
    inputData->pitch_stick = 0;
    inputData->yaw_stick = 0;
    inputData->arm_switch_state = false;
}

void printThrottle() { 
    MotorThrottle &mt = *motorThrottle;
    Serial.printf("M: %d %d %d %d\n", mt.M_FR, mt.M_RR, mt.M_RL, mt.M_FL);
}

void printAngle() {
    IMU &imuRef = *imu;
    Serial.printf("Angle Roll: %.2f | Angle Pitch: %.2f\n", imuRef.getCurrentAngleRoll(), imuRef.getCurrentAnglePitch());
}