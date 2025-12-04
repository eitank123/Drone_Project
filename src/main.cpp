#include <main.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

TaskHandle_t FlightTaskHandle;

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000));
    Serial.begin(115200);
    //while(!Serial);
    Serial.println("Booting...");

    inputData = new RCInputData();
    if (inputData == nullptr) {
        Serial.println("FATAL: Input Memory Failed");
        while(1);
    }
    // Set Safe Defaults
    inputData->throttle_raw = 1000;
    inputData->roll_stick = 0;
    inputData->pitch_stick = 0;
    inputData->yaw_stick = 0;
    inputData->arm_switch_state = false;

    // 3. START BLUETOOTH (Core 0)
    // This starts the radio listeners on the background core.
    initBLE();

    // Create the Flight Task pinned to Core 1
    xTaskCreatePinnedToCore(
        FlightControlTask,   // Function to run
        "FlightLoop",        // Name of task
        10000,               // Stack size (bytes) - made generous to prevent crashes
        NULL,                // Parameters
        1,                   // Priority (1 = high)
        &FlightTaskHandle,     // Task handle
        1                    // PIN TO CORE 1 (User Code Core)
    );
    
    Serial.println("System Started: Flight Loop on Core 1, Bluetooth on Core 0");
}

void FlightControlTask(void *parameter) {
    uint8_t current_stage = stage1;
    unsigned long last_time = 0;

    // Infinite loop
    for(;;) { 
        unsigned long now = micros();
        
        // Non-blocking timer
        if (now - last_time >= LOOP_INTERVAL_US) {
            last_time = now;
            FSM(&current_stage);
        }
        
        delayMicroseconds(10); 
    }
}


void loop() {
  delay(1000);
}


