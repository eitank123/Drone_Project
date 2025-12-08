#ifndef FSM_H
    #define FSM_H

    #include <Arduino.h>
    #include <imu.h>
    #include "PID.h"


    #define BUFFER_SIZE 12
    #define stabilizeTime 100 //in microseconds
    #define stage1 1
    #define stage2 2

    #define ThrottleLowerBound 1000
    #define ThrottleUpperBound 2000

    #define rollStickScaling 0.2
    #define pitchStickScaling 0.2
    #define yawStickScaling 0.2

    #define kp 0.2
    #define ki 0.005
    #define kd 0.02

    #define level_strength 10.0

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

struct angle_setpoints {
    float setpoint_rate_roll;
    float setpoint_rate_pitch;
};
    void FSM(uint8_t *stage);
    void Stage1();
    void Stage2();
    void calcThrottle();
    void printThrottle();
    void printAngle();
    void init_imu(IMU* imu);
    void resetThrottle();
    void initData();
    void set_current_angle();
    angle_setpoints calc_setpoints();
    extern RCInputData* inputData;
    extern MotorThrottle* motorThrottle;

    const long SAMPLE_RATE_HZ = 20; 
    const unsigned long LOOP_INTERVAL_US = 1e6 / SAMPLE_RATE_HZ;
    const float dt = 1.0 / SAMPLE_RATE_HZ;                
#endif