#ifndef _SPI_H_
#define _SPI_H_

#include "config.h"

#if defined(PLATFORM_ARDUINO)
#include "arduino/spi_arduino.h"
#elif defined(PLATFORM_CC2640)
#include "cc2640/spi_cc2640.h"
#endif

const unsigned char SPI_DUMMY_INT = 0x00;
const unsigned char SPI_TRANSFER_LEN = 1;

#if defined(__cplusplus)
extern "C"
{
#endif

    void spi_init();
    void spi_start();
    void spi_end();
    unsigned char spi_transfer(const unsigned char data);

#if defined(__cplusplus)
}
#endif

#endif
