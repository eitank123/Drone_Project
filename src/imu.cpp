// imu.cpp
// Implementation file for IMU communication

#include <imu.h>


IMU::IMU(uint8_t address) : _address(address), _spiComm(CS_PIN) {}

void IMU::begin() {
    // Add IMU initialization code here
    _spiComm.begin();
    Serial.println("Initializing BMI323...");
  
     // 1. Soft Reset
    _spiComm.spiWrite16(BMI323_REG_CMD, 0xDEAF);
    delay(50); // Wait for reset

    // 2. Dummy Read (To clear SPI garbage)
    _spiComm.spiRead8(0x00);
    // 3. Verify Chip ID
    uint8_t chipID = _spiComm.spiRead8(_address);
    Serial.print("Chip ID: 0x");
    Serial.println(chipID, HEX);
    
    if(chipID != BMI323_ID) {
        Serial.println("Error: Chip ID mismatch or communication fail.");
        while(1);
    }

    // 4. ENABLE SENSORS (The Fix)
    // We must write 16 bits to set Mode to "High Performance"
    Serial.println("Enabling Accelerometer & Gyro...");
    _spiComm.spiWrite16(BMI323_REG_ACC_CONF, CONF_ACC_ENABLE);
    delay(10);
    _spiComm.spiWrite16(BMI323_REG_GYR_CONF, CONF_GYR_ENABLE);
    delay(50); // Wait for filter settling
}

void IMU::readData(uint8_t rawData[12]) {
    // Read 12 bytes: Acc X, Y, Z + Gyro X, Y, Z
  _spiComm.spiReadBurst(BMI323_REG_ACC_DATA_X, rawData, 12);

  // Reassemble 16-bit signed integers
  try
  {
    /* code */
    _data.accelX = (int16_t)((rawData[1] << 8) | rawData[0]) / ACCEL_SENSITIVITY;
    _data.accelY = (int16_t)((rawData[3] << 8) | rawData[2]) / ACCEL_SENSITIVITY;
    _data.accelZ = (int16_t)((rawData[5] << 8) | rawData[4]) / ACCEL_SENSITIVITY;

    _data.gyroX = (int16_t)((rawData[7] << 8) | rawData[6]) / GYRO_SENSITIVITY;
    _data.gyroY = (int16_t)((rawData[9] << 8) | rawData[8]) / GYRO_SENSITIVITY;
    _data.gyroZ = (int16_t)((rawData[11] << 8) | rawData[10]) / GYRO_SENSITIVITY;

    dataCutoff();
  }
  catch(const std::exception& e)
  {
    Serial.println(e.what());
    #ifdef DEBUG
      Serial.println("Read Failed");
    #endif
  }  
}

void IMU::dataCutoff()
{
    _data.accelX = abs(_data.accelX) > ACCEL_CUTOFF ? _data.accelX : 0;
    _data.accelY = abs(_data.accelY) > ACCEL_CUTOFF ? _data.accelY : 0;
    _data.accelZ = abs(_data.accelZ) > ACCEL_CUTOFF ? _data.accelZ : 0;

    _data.gyroX = abs(_data.gyroX) > GYRO_CUTOFF ? _data.gyroX : 0;
    _data.gyroY = abs(_data.gyroY) > GYRO_CUTOFF ? _data.gyroY : 0;
    _data.gyroZ = abs(_data.gyroZ) > GYRO_CUTOFF ? _data.gyroZ : 0;
}


float IMU::getXaccel()
{
    return _data.accelX;
}


float IMU::getYaccel()
{
    return _data.accelY;
}

float IMU::getZaccel()
{
    return _data.accelZ;
}

float IMU::getXgyro()
{
    return _data.gyroX;
}

float IMU::getYgyro()
{
    return _data.gyroY;
}

float IMU::getZgyro()
{
    return _data.gyroZ;
}

void IMU::printData()
{
    Serial.print("AX: "); Serial.print(_data.accelX);
    Serial.print(" g | AY: "); Serial.print(_data.accelY);
    Serial.print(" g | AZ: "); Serial.print(_data.accelZ);
    Serial.print(" g | GX: "); Serial.print(_data.gyroX);
    Serial.print(" dps | GY: "); Serial.print(_data.gyroY);
    Serial.print(" dps | GZ: "); Serial.print(_data.gyroZ);
    Serial.println(" dps");
}
