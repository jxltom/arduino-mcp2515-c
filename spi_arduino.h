#ifndef _SPI_ARDUINO_H_
#define _SPI_ARDUINO_H_

#include <Arduino.h>

static const uint32_t SPI_CLOCK = 10000000; // 10MHz
const uint8_t SPICS = 10;

#if defined(__cplusplus)
extern "C"
{
#endif

    void spi_begin();
    void spi_start();
    void spi_end();
    uint8_t spi_transfer(uint8_t data);

#if defined(__cplusplus)
}
#endif

#endif
