#ifndef DSHOT_S3_H
#define DSHOT_S3_H

#include <Arduino.h>
#include <driver/rmt.h> // Arduino pulls this from the underlying IDF

class DShotS3 {
public:
    DShotS3(int pin, int channel);
    void begin();
    void sendThrottle(uint16_t throttle);

private:
    int _pin;
    rmt_channel_t _channel; 
    uint16_t prepareFrame(uint16_t throttle, bool telemetry = false);
};

#endif