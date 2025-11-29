#include "BLEInput.h"

extern RCInputData* inputData; 

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println(">>> CONNECTED - Signal is Good (-60dBm) <<<");
    };

    void onDisconnect(BLEServer* pServer) {
        Serial.println("!!! DISCONNECTED !!!");
        // Restart advertising to allow reconnect
        delay(500); 
        BLEDevice::startAdvertising();
        Serial.println("... Advertising Restarted");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        // 1. DEBUG: Did we get here?
        // Serial.println("Packet Received..."); 

        std::string rxValue = pCharacteristic->getValue();
        
        // 2. DEBUG: Check length
        if (rxValue.length() != 9) {
            Serial.printf("Error: Wrong Length (%d bytes)\n", rxValue.length());
            return;
        }

        // 3. DEBUG: Check Pointer Safety
        if (inputData == nullptr) {
            Serial.println("CRASH AVOIDED: inputData is NULL!");
            return;
        }

        // 4. Safe Write
        inputData->throttle_raw = (uint8_t)rxValue[0] | ((uint8_t)rxValue[1] << 8);
        inputData->roll_stick   = (int16_t)((uint8_t)rxValue[2] | ((uint8_t)rxValue[3] << 8));
        inputData->pitch_stick  = (int16_t)((uint8_t)rxValue[4] | ((uint8_t)rxValue[5] << 8));
        inputData->yaw_stick    = (int16_t)((uint8_t)rxValue[6] | ((uint8_t)rxValue[7] << 8));
        inputData->arm_switch_state = (bool)rxValue[8];

        // 5. Success Print (Limit speed to avoid serial lag)
        // static unsigned long lastPrint = 0;
        // if (millis() - lastPrint > 500) {
        //    Serial.printf("Data OK! Throttle: %d\n", inputData->throttle_raw);
        //    lastPrint = millis();
        // }
    }
};

void initBLE() {
    BLEDevice::init("ESP32_DRONE");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_WRITE_NR
                                       );
    pCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    
    // Slow down advertising slightly to ensure stability
    pAdvertising->setMinPreferred(0x10);  
    pAdvertising->setMinPreferred(0x20);
    
    BLEDevice::startAdvertising();
    Serial.println("BLE Ready.");
}