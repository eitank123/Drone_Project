#include "ModeOfOperation.h"

ModeOfOperation::ModeOfOperation() {}

PIDTarget ModeOfOperation::getLogic(FlightMode mode, float stickValue, float currentAngle, float currentRate) {
    PIDTarget target;

    if (mode == ANGLE_MODE) {
        // Mode 1: Self-Leveling
        // Stick controls the Angle, Sensor gives us Angle
        target.setpoint = stickValue; 
        target.measured = currentAngle;
    } 
    else {
        // Mode 0: Acro/Rate
        // Stick controls the Rate, Sensor gives us Gyro Rate
        target.setpoint = stickValue;
        target.measured = currentRate;
    }

    return target;
}