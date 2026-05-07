#ifndef MODE_OF_OPERATION_H
#define MODE_OF_OPERATION_H

#include <Arduino.h>

/**
 * @brief Enum to define the pilot's control style.
 */
enum FlightMode {
    ACRO_MODE = 0,   // Sticks control rotation rate (Deg/s)
    ANGLE_MODE = 1   // Sticks control absolute angle (Degrees)
};

/**
 * @brief Simple struct to bundle the PID inputs.
 */
struct PIDTarget {
    float setpoint;
    float measured;
};

class ModeOfOperation {
public:
    ModeOfOperation();

    /**
     * @brief "The Multiplexer" - Returns correct data based on the mode.
     * * @param mode Selected mode (0 or 1)
     * @param stickValue The scaled input from the RC controller
     * @param currentAngle The filtered angle from the IMU (Degrees)
     * @param currentRate The raw gyro rate from the IMU (Deg/s)
     * @return PIDTarget Combined setpoint and measurement
     */
    PIDTarget getLogic(FlightMode mode, float stickValue, float currentAngle, float currentRate);

private:
    // You can add scaling constants here later
    float _maxAngle = 35.0f;     // Max tilt in Angle Mode
    float _maxRate = 200.0f;     // Max rotation in Acro Mode
};

#endif