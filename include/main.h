
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <FSM.h>
#include "BLEInput.h"


void FlightControlTask(void *parameter);
extern TaskHandle_t FlightTaskHandle;

#endif