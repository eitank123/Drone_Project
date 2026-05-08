#include <main.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


// Handle for the task
TaskHandle_t FastLoopTask;
EulerAngles orientation;
ModeOfOperation modeHandler;
FlightMode currentMode = ANGLE_MODE;

PID PID_pitch(1.2, 0.01, 0.5, 100);
PID PID_roll(1.2, 0.01, 0.5, 100);
PID PID_yaw(2.0, 0.02, 0.1, 100);

#ifdef Test
float stickPitch = 0.0f;
float stickRoll = 0.0f;
float stickYaw = 0.0f;
float throttle = 1500.0f; // Mid-throttle for testing
#endif

void FastLoop(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(FastLoopTime); // 10ms = 100Hz

    for(;;) {
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // --- FAST LOOP CODE HERE ---
        orientation = getOrientation();

        // 1. Get logic targets for all three (Simplified for example)
        PIDTarget pLog = modeHandler.getLogic(currentMode, stickPitch, orientation.pitch, get_gyrY());
        PIDTarget rLog = modeHandler.getLogic(currentMode, stickRoll,  orientation.roll,  get_gyrX());
        PIDTarget yLog = modeHandler.getLogic(currentMode, stickYaw,   orientation.yaw,   get_gyrZ());

        // 2. Compute PID outputs
        float outP = PID_pitch.update(pLog.setpoint, pLog.measured, 0.01);
        float outR = PID_roll.update(rLog.setpoint, rLog.measured, 0.01);
        float outY = PID_yaw.update(yLog.setpoint, yLog.measured, 0.01);

        // 3. COMBINE (The Mixer)
        // This is where the magic happens!
        mixMotors(throttle, outP, outR, outY);
    }
}


void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Initializing IMU...");
    initIMU();

    // Create the task on Core 1 (leave Core 0 for Wi-Fi/System)
    xTaskCreatePinnedToCore(
        FastLoop,        /* Function to implement the task */
        "FastLoop",      /* Name of the task */
        10000,           /* Stack size in words */
        NULL,            /* Task input parameter */
        3,               /* Priority (Higher is better) */
        &FastLoopTask,   /* Task handle */
        1                /* Core ID */
    );

    escs.begin();
}


void loop() {
    #ifdef CALIBRATE_MAG
        calibrateMag();
    #endif
    Serial.printf("R:%.2f P:%.2f Y:%.2f\n", orientation.roll, orientation.pitch, orientation.yaw);
    delay(50); // Run at ~20Hz for smooth tracking
}


