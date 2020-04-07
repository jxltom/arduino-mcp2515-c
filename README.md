# mcp2515-c

## Supported Platform

- Arduino
- TI CC2640
- Linux (still in implementation)
- Other MCU by providing appropriate SPI interface

## Getting Started

- Adjust ```spi.h``` to your platform by ```#define SPI_ARDUINO``` or ```#define SPI_CC2640```.

## Development

- Install PlatformIO

- Build by ```pipenv run build```.

### Issues

- The project's parent directory should be emtpy for using PlatformIO, otherwise PlatformIO will build all other directories as libraries which makes compilation process very slow.
