#ifndef _SPI_H_
#define _SPI_H_

#define SPI_ARDUINO

#if defined(SPI_ARDUINO)
#include "spi_arduino.h"
#elif defined(SPI_CC2640)
#include "spi_cc2640.h"
#endif

#endif
