#include "PID.h"

PID::PID(float kp, float ki, float kd, float limit) 
    : _kp(kp), _ki(ki), _kd(kd), _limit(limit), _integral(0), _prev_error(0) {}

float PID::update(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // 1. Proportional Term
    float P = _kp * error;

    // 2. Integral Term
    _integral += error * dt;
    // Anti-windup clamping
    _integral = constrain(_integral, -_limit, _limit);
    float I = _ki * _integral;

    // 3. Derivative Term
    // Standard: (error - prev_error) / dt
    float derivative = (error - _prev_error) / dt;
    float D = _kd * derivative;

    _prev_error = error;

    // 4. Sum and Limit
    float output = P + I + D;
    return constrain(output, -_limit, _limit);
}

void PID::reset() {
    _integral = 0;
    _prev_error = 0;
}