#include "PID.h"

PID::PID(float kp, float ki, float kd, float limit) 
    : _kp(kp), _ki(ki), _kd(kd), _limit(limit), _integral(0), _prev_measured(0) {}

float PID::update(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // 1. Proportional Term
    float P = _kp * error;

    // 2. Integral Term
    _integral += error * dt;
    _integral = constrain(_integral, -_i_limit, _i_limit); // Use a dedicated I-limit
    float I = _ki * _integral;

    // 3. Derivative Term (On Measurement, not Error)
    float derivative = - (measured - _prev_measured) / dt;
    
    // Simple D-term Low Pass Filter
    _d_filtered = _d_filtered + derivative_filter * (derivative - _d_filtered);
    float D = _kd * _d_filtered;

    _prev_measured = measured;

    // 4. Sum and Limit
    float output = P + I + D;
    return constrain(output, -_out_limit, _out_limit);
}

void PID::reset() {
    _integral = 0;
    _prev_measured = 0;
    _d_filtered = 0;
}