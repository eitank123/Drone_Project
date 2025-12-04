#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
    PID(float kp, float ki, float kd, float limit);
    
    // The main compute function
    // setpoint: Desired rate (from stick)
    // measured: Actual rate (from gyro)
    // dt: Time since last loop in seconds
    float update(float setpoint, float measured, float dt);
    
    void reset();

private:
    float _kp, _ki, _kd;
    float _limit;           // Max output limit (to prevent saturation)
    float _integral;        // Accumulated Error
    float _prev_error;      // Previous Error (for D term)
};

#endif