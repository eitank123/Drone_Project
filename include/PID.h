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
    float _prev_measured;   // Previous measured value (for D term)
    float _d_filtered = 0;
};

// derivative filter coefficient: $$ \alpha = \frac{2\pi f_c dt}{1 + 2\pi f_c dt} $$
#define derivative_filter 0.5f // Smoothing factor for D-term low pass filter (lower = smoother but more delay)

// TODO: Tune these limits based on your ESC's expected input range and desired responsiveness.
#define _i_limit 100.0f // Integral windup limit (tune as needed)
#define _out_limit 100.0f // Max output limit (tune based on ESC range)

#endif