#ifndef _SPI_CC2640_H_
#define _SPI_CC2640_H_

#include <ti/drivers/PIN.h>

const uint32_t SPI_CLOCK = 5000000; // 5MHz
const uint8_t SPICS = IOID_20;

void spi_begin();
void spi_start();
void spi_end();
uint8_t spi_transfer(uint8_t data);

#endif
