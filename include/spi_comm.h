// spi_comm.h
// Header file for SPI communication
#define SPI_COMM_H

#ifdef SPI_COMM_H
// --- Pin Definitions ---
#define SCK_PIN  14 // Confirmed SCK pin for standard SPI
#define MOSI_PIN 11
#define MISO_PIN 12


#include <Arduino.h>

class SPIComm {
public:
    SPIComm(uint8_t csPin);
    uint8_t getCSPin() const {return _csPin; }
    void begin();
    uint8_t transfer(uint8_t data);
    void spiWrite16(uint8_t reg, uint16_t val);
    void spiReadBurst(uint8_t startReg, uint8_t *buffer, uint8_t len);
    uint8_t spiRead8(uint8_t reg);
    void end();
private:
    uint8_t _csPin;
};

#endif // SPI_COMM_H
