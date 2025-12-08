// imu.h
// Header file for IMU communication
#define IMU_H

#ifdef IMU_H

#include <Arduino.h>
#include <spi_comm.h>

#define DEBUG

// --- BMI323 Registers ---
#define BMI323_REG_ACC_DATA_X   0x03
#define BMI323_REG_ACC_CONF     0x20
#define BMI323_REG_GYR_CONF     0x21
#define BMI323_REG_CMD          0x7E

// --- Configuration Values ---
// 0x4027 Breakdown:
// [15:12] Mode = 4 (High Performance)
// [10:8]  Range = 0 (2g / 125dps)
// [3:0]   ODR = 7 (100 Hz)
#define CONF_ACC_ENABLE  0x4028 // range of 8G and 100 HZ
#define CONF_GYR_ENABLE  0x4048 // range of 2000 DPS and 100 HZ
#define GYRO_SENSITIVITY 16.384
#define ACCEL_SENSITIVITY 4096.0
#define CS_PIN   17

#define BMI323_ID 0x43
#define ACCEL_CUTOFF 0.05
#define GYRO_CUTOFF 0.3

#define alpha 0.98 // Gyro integrity

//#define DEBUG


struct IMUData {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
};

class IMU {
public:
    IMU(uint8_t address);
    void begin();
    void readData(uint8_t rawData[12]);
    void printData();
    float getXaccel();
    float getYaccel();
    float getZaccel();
    float getXgyro();
    float getYgyro();
    float getZgyro();
    float getCurrentAngleRoll();
    float getCurrentAnglePitch();
    void set_current_angle_roll(float accRoll, float dt);
    void set_current_angle_pitch(float accPitch, float dt);
private:
    IMUData _data;
    uint8_t _address;
    SPIComm _spiComm;
    float _current_angle_roll;
    float _current_angle_pitch;
    void dataCutoff();
};

#endif // IMU_H
