
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <FSM.h>
#include "BLEInput.h"

const long SAMPLE_RATE_HZ = 20; 
const unsigned long LOOP_INTERVAL_US = 1e6 / SAMPLE_RATE_HZ;
void FlightControlTask(void *parameter);
extern TaskHandle_t FlightTaskHandle;

#endif