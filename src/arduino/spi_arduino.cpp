#include "../config.h"
#if defined(PLATFORM_ARDUINO)

#include <Arduino.h>
#include <SPI.h>
#include "../spi.h"

void spi_init()
{
    SPI.begin();

    pinMode(SPICS, OUTPUT);
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

uint8_t spi_transfer(const uint8_t data)
{
    return SPI.transfer(data);
}

#endif
