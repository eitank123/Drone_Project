#include "BLEInput.h"


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

        std::string rxValue = pCharacteristic->getValue();
        
        if (rxValue.length() != 9) {
            Serial.printf("Error: Wrong Length (%d bytes)\n", rxValue.length());
            return;
        }

        if (inputData == nullptr) {
            Serial.println("CRASH AVOIDED: inputData is NULL!");
            return;
        }
        updateInputData(rxValue);
    }

    void updateInputData(std::string rxValue) {
        inputData->throttle_raw = (uint8_t)rxValue[0] | ((uint8_t)rxValue[1] << 8);
        inputData->roll_stick   = (int16_t)((uint8_t)rxValue[2] | ((uint8_t)rxValue[3] << 8));
        inputData->pitch_stick  = (int16_t)((uint8_t)rxValue[4] | ((uint8_t)rxValue[5] << 8));
        inputData->yaw_stick    = (int16_t)((uint8_t)rxValue[6] | ((uint8_t)rxValue[7] << 8));
        inputData->arm_switch_state = (bool)rxValue[8];
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