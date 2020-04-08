#ifndef _SPI_H_
#define _SPI_H_

#include "config.h"

#if defined(SPI_ARDUINO)
#include "spi_arduino.h"
#elif defined(SPI_CC2640)
#include "spi_cc2640.h"
#endif

const unsigned char SPI_DUMMY_INT = 0x00;
const unsigned char SPI_TRANSFER_LEN = 1;

#endif
