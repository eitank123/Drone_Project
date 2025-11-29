#include <Input.h>

RCInputData* inputData = new RCInputData();

void initData() {
    // Initialize input data here if needed
    inputData->throttle_raw = 0;
    inputData->roll_stick = 0;
    inputData->pitch_stick = 0;
    inputData->yaw_stick = 0;
    inputData->arm_switch_state = false;
}

void readInput() {
    inputData->throttle_raw = 1001;
    inputData->roll_stick = 0;
    inputData->pitch_stick = 0;
    inputData->yaw_stick = 0;
}

