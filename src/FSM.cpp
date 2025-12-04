#include <FSM.h>

uint8_t rawData[BUFFER_SIZE];
int16_t T = 800; // Sample rate
float delayTime = 1e6 / T;
float delta_t = 1.0 / T;

IMU* imu = nullptr;
MotorThrottle* motorThrottle = nullptr;
unsigned long lastDebugTime = 0;

RCInputData* inputData = new RCInputData();

PID pidRoll(1.5, 0.0, 0.5, 400);  
PID pidPitch(1.5, 0.0, 0.5, 400);
PID pidYaw(3.0, 0.0, 0.0, 400);

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
    calcThrottle();

    unsigned long now = micros();
    float dt = (now - last_pid_time) / 1000000.0;
    last_pid_time = now;

    float setpoint_roll  = inputData->roll_stick  * rollStickScaling; // Scaling factor
    float setpoint_pitch = inputData->pitch_stick * pitchStickScaling;
    float setpoint_yaw   = inputData->yaw_stick   * yawStickScaling;

    float pid_roll_out  = pidRoll.update(setpoint_roll, imu->getXgyro(), dt);
    float pid_pitch_out = pidPitch.update(setpoint_pitch, imu->getYgyro(), dt);
    float pid_yaw_out   = pidYaw.update(setpoint_yaw, imu->getZgyro(), dt);


    #ifdef DEBUG
    // Print logic (unchanged)
    if (millis() - lastDebugTime > 200) {
        lastDebugTime = millis();
        // Serial.printf("G_X: %.2f | SP: %.2f | PID: %.2f\n", gyro_x_dps, setpoint_roll, pid_roll_out);
        printThrottle();
    }
    #endif
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

    mt.M_FR = input.throttle_raw + input.roll_stick - input.pitch_stick + input.yaw_stick;
    mt.M_RR = input.throttle_raw + input.roll_stick + input.pitch_stick - input.yaw_stick;
    mt.M_RL = input.throttle_raw - input.roll_stick + input.pitch_stick + input.yaw_stick;
    mt.M_FL = input.throttle_raw - input.roll_stick - input.pitch_stick - input.yaw_stick;

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