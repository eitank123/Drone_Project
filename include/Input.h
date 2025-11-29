#ifndef INPUT_H
#define INPUT_H

#include <Arduino.h>

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

extern RCInputData *inputData;
void initData();
void readInput();
#endif