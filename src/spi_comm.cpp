// spi_comm.cpp
// Implementation file for SPI communication

#include <SPI.h>
#include "spi_comm.h"


SPIComm::SPIComm(uint8_t csPin) : _csPin(csPin) {}

void SPIComm::begin() {
    // Prevent I2C Mode Latch-up
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
}

uint8_t SPIComm::transfer(uint8_t data) {
    digitalWrite(_csPin, LOW);
    uint8_t result = SPI.transfer(data);
    digitalWrite(_csPin, HIGH);
    return result;
}

void SPIComm::end() {
    SPI.end();
}

// --- 16-bit Write Function (CRITICAL FOR BMI323) ---
void SPIComm::spiWrite16(uint8_t reg, uint16_t val) {
  SPISettings settings(4000000, MSBFIRST, SPI_MODE0);

  SPI.beginTransaction(settings);
  digitalWrite(_csPin, LOW);
  
  // 1. Send Register Address (Write mode = MSB 0)
  SPI.transfer(reg & 0x7F); 
  
  // 2. Send LSB (Least Significant Byte) FIRST for Bosch 16-bit regs
  SPI.transfer(val & 0xFF);
  
  // 3. Send MSB (Most Significant Byte) SECOND
  SPI.transfer(val >> 8);
  
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

// --- Burst Read Function ---
void SPIComm::spiReadBurst(uint8_t startReg, uint8_t *buffer, uint8_t len) {
  SPISettings settings(4000000, MSBFIRST, SPI_MODE0);

  SPI.beginTransaction(settings);
  digitalWrite(_csPin, LOW);
  
  SPI.transfer(startReg | 0x80); // Read mode
  SPI.transfer(0x00);            // Dummy Byte (Required)
  
  for(int i=0; i<len; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

uint8_t SPIComm::spiRead8(uint8_t reg) {
  SPISettings settings(4000000, MSBFIRST, SPI_MODE0);

  SPI.beginTransaction(settings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00); // Dummy
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  return val;
}
