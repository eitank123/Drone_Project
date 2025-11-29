#include <FSM.h>

uint8_t rawData[BUFFER_SIZE];
int16_t T = 800; // Sample rate
float delayTime = 1e6 / T;
float delta_t = 1.0 / T;

IMU* imu = nullptr;
MotorThrottle* motorThrottle = nullptr;
unsigned long lastDebugTime = 0;


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

    initMotorThrottle();

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
    readInput();
    calcThrottle();

    #ifdef DEBUG
    if (millis() - lastDebugTime > 200) {
        lastDebugTime = millis();
        
        // Use the Reference to make it readable
        RCInputData &in = *inputData; 
        
        Serial.printf("THROT: %d | ROLL: %d | PITCH: %d | YAW: %d | ARM: %d\n", 
                      in.throttle_raw, 
                      in.roll_stick, 
                      in.pitch_stick, 
                      in.yaw_stick, 
                      in.arm_switch_state);
                      
        // Optional: Print calculated motor outputs to see if mixing works
        MotorThrottle &out = *motorThrottle;
        Serial.printf("M_FR: %d | M_RR: %d | M_RL: %d | M_FL: %d\n", 
                      out.M_FR, out.M_RR, out.M_RL, out.M_FL);
        Serial.println("--------------------------------");
    }
    #endif
}

void init_imu(IMU* imu)
{
    imu->begin();
    Serial.println("IMU Initialized Successfully");
}

void initMotorThrottle(){
    motorThrottle->M_RR = 0;
    motorThrottle->M_RL = 0;
    motorThrottle->M_FR = 0;
    motorThrottle->M_FL = 0;
}

void setThrottle(int16_t T1, int16_t T2, int16_t T3, int16_t T4){
    MotorThrottle &mt = *motorThrottle;
    mt.M_RR = T1;
    mt.M_RL = T2;
    mt.M_FR = T3;
    mt.M_FL = T4;
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
    
    #ifdef DEBUG
        //printThrottle();
    #endif
}

void printThrottle(){
    MotorThrottle &mt = *motorThrottle;

    Serial.print("Motor Throttle FR: ");
    Serial.println(mt.M_FR);
    Serial.print("Motor Throttle RR: ");
    Serial.println(mt.M_RR);
    Serial.print("Motor Throttle RL: ");
    Serial.println(mt.M_RL);
    Serial.print("Motor Throttle FL: ");
    Serial.println(mt.M_FL);
}