#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

struct MotorSignals {
    float m1; // Front-Right
    float m2; // Back-Left
    float m3; // Front-Left
    float m4; // Back-Right
};

// Function to calculate individual motor power
void mixMotors(float throttle, float outP, float outR, float outY);

// Function to actually write to the ESCs (PWM or DShot)
void writeToESCs(MotorSignals signals);

#define maxOut 2000.0f // Max output from PID (tune based on ESC range)
#define minOut 200.0f // Min output from PID (tune based on ESC range)


#define Test

#endif