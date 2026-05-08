#include "DShotS3.h"

DShotS3::DShotS3(int pin, int channel) : _pin(pin) {
    _channel = (rmt_channel_t)channel;
}

void DShotS3::begin() {
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = _channel;
    config.gpio_num = (gpio_num_t)_pin;
    config.mem_block_num = 1;
    config.clk_div = 8; // 80MHz / 8 = 10MHz (100ns ticks)
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    rmt_config(&config);
    rmt_driver_install(_channel, 0, 0);
}

uint16_t DShotS3::prepareFrame(uint16_t throttle, bool telemetry) {
    // 11-bit throttle, 1-bit telemetry (0), 4-bit CRC
    uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);
    uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    return (packet << 4) | crc;
}

void DShotS3::sendThrottle(uint16_t throttle) {
    uint16_t frame = prepareFrame(throttle, false);
    rmt_item32_t items[16];

    for (int i = 0; i < 16; i++) {
        bool bit = (frame >> (15 - i)) & 0x01;
        // DShot600 Timings (1 tick = 100ns)
        if (bit) {
            items[i] = {{{ 12, 1, 5, 0 }}}; // High 1.2us, Low 0.5us
        } else {
            items[i] = {{{ 6, 1, 11, 0 }}}; // High 0.6us, Low 1.1us
        }
    }
    rmt_write_items(_channel, items, 16, false);
}