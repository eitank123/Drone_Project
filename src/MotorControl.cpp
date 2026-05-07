#include "motorcontrol.h"

void mixMotors(float throttle, float outP, float outR, float outY) {
    MotorSignals signals;

    /* The "X" Configuration Logic:
       Motor 1 (FR): Pulls Pitch Down (-), Pulls Roll Left (-), Yaw CCW (+)
       Motor 2 (BL): Pulls Pitch Up (+),   Pulls Roll Right (+), Yaw CCW (+)
       Motor 3 (FL): Pulls Pitch Down (-), Pulls Roll Right (+), Yaw CW (-)
       Motor 4 (BR): Pulls Pitch Up (+),   Pulls Roll Left (-), Yaw CW (-)
    */

    signals.m1 = throttle - outP - outR + outY; // Front-Right
    signals.m2 = throttle + outP + outR + outY; // Back-Left
    signals.m3 = throttle - outP + outR - outY; // Front-Left
    signals.m4 = throttle + outP - outR - outY; // Back-Right

    // --- Safety: Constraints ---
    // We must ensure we don't send values below 'Min Throttle' (where motors stop)
    // or above 'Max Throttle' (where ESCs saturate).
    signals.m1 = constrain(signals.m1, minOut, maxOut);
    signals.m2 = constrain(signals.m2, minOut, maxOut);
    signals.m3 = constrain(signals.m3, minOut, maxOut);
    signals.m4 = constrain(signals.m4, minOut, maxOut);

    writeToESCs(signals);
}

void writeToESCs(MotorSignals signals) {
    // This function would convert the MotorSignals to the appropriate PWM or DShot signals
    // and write them to the ESCs. For example, if using PWM:
    // pwmWrite(MOTOR1_PIN, signals.m1);
    // pwmWrite(MOTOR2_PIN, signals.m2);
    // pwmWrite(MOTOR3_PIN, signals.m3);
    // pwmWrite(MOTOR4_PIN, signals.m4);

    #ifdef Test
    // For this example, we'll just print the values to Serial for demonstration.
    Serial.print("M1: "); Serial.print(signals.m1);
    Serial.print(" | M2: "); Serial.print(signals.m2);
    Serial.print(" | M3: "); Serial.print(signals.m3);
    Serial.print(" | M4: "); Serial.println(signals.m4);
    #endif
}