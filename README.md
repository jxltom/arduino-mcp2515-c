# mcp2515-c

## Supported Platform

- Arduino
- TI CC2640
- Linux (work in process)
- Supporting on other platforms by implementing appropriate SPI and Util interface are welcomed!

## Getting Started

Add ```wjcan_hal.h``` in your application and define platform such as by ```#define PLATFORM_ARDUINO``` or ```#define PLATFORM_CC2640``` or ```#define PLATFORM_CC2642```.

## Development

- Install PlatformIO

- Build by ```pipenv run build```

### SPI and Util Interface

This library supports any MCU only if SPI and Util is implemented according to following interface.

```c
void spi_init();
void spi_start();
void spi_end();
unsigned char spi_transfer(unsigned char data);
```

```c
unsigned long util_msecs();
void util_delay(unsigned int ms);
```

### Issues

- The project's parent directory should be emtpy for using PlatformIO, otherwise PlatformIO will build all other directories as libraries which makes compilation process very slow.
