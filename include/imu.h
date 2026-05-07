#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "ICM_20948.h"


struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};

void initIMU();
EulerAngles getOrientation();
void calibrateMag();
void calcRollPitch(float dt);
void calcYaw(float dt);
float get_gyrY();
float get_gyrZ();
float get_gyrX();

# define FastLoopTime 10 // ms (100Hz)

#endif