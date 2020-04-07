#if SPI_MODULE == 1

#include <Arduino.h>
#include <SPI.h>
#include "spi_arduino.h"

void spi_begin()
{
    SPI.begin();
}

void spi_start()
{
    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(SPICS, LOW);
}

void spi_end()
{
    digitalWrite(SPICS, HIGH);
    SPI.endTransaction();
}

uint8_t spi_transfer(uint8_t data)
{
    return SPI.transfer(data);
}

#endif
