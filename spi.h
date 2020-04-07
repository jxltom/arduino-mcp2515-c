#ifndef _SPI_H_
#define _SPI_H_

#define SPI_MODULE 1

#if SPI_MODULE == 1
#include "spi_arduino.h"
#elif SPI_MODULE == 2
#include "spi_cc2640.h"
#endif

#endif
