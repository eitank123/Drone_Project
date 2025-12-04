#ifndef FSM_H
    #define FSM_H

    #include <Arduino.h>
    #include <imu.h>
    #include "PID.h"
    #include "Noise.h"


    #define BUFFER_SIZE 12
    #define stabilizeTime 100 //in microseconds
    #define stage1 1
    #define stage2 2

    #define ThrottleLowerBound 1000
    #define ThrottleUpperBound 2000

    #define rollStickScaling 0.4
    #define pitchStickScaling 0.4
    #define yawStickScaling 0.4

    #define gyroNoiseAmplitude 1e-4
    
    #define kpRoll 0.1
    #define kiRoll 0.1
    #define kdRoll 0.1

    #define kpPitch 0.1
    #define kiPitch 0.1
    #define kdPitch 0.1

    #define kpYaw 0.1
    #define kiYaw 0.1
    #define kdYaw 0.1
    void FSM(uint8_t *stage);
    void Stage1();
    void Stage2();
    void calcThrottle();
    void printThrottle();
    void init_imu(IMU* imu);
    void resetThrottle();
    void initData();
    void setNoise();

    struct RCInputData {
    // The raw throttle value (e.g., 1000 to 2000 microseconds)
    int16_t throttle_raw;
    
    // The control stick values (e.g., centered at 0, range -500 to 500)
    int16_t roll_stick;
    int16_t pitch_stick;
    int16_t yaw_stick;
    
    // Add other switches/modes here
    bool arm_switch_state;
    };

struct MotorThrottle {
    int16_t M_RR;
    int16_t M_RL;
    int16_t M_FR;
    int16_t M_FL;
    };
    
    extern RCInputData* inputData;
    extern MotorThrottle* motorThrottle;
#endif