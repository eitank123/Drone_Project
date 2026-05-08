#ifndef ESC_COMMUNICATION_H
#define ESC_COMMUNICATION_H

#include <Arduino.h>
#include "motorcontrol.h" // Assuming this defines MotorSignals

// ESP32-S3 Pins (ensure these match your PCB/Wiring)
#define MOTOR_1_PIN 4
#define MOTOR_2_PIN 5
#define MOTOR_3_PIN 6
#define MOTOR_4_PIN 7

class DShotESC {
public:
    void begin();
    void writeMotors(MotorSignals signals);
    void disarm();
};

// This 'extern' makes the 'escs' object visible to main.cpp
extern DShotESC escs; 

#endif