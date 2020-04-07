# mcp2515-c

## Supported Platform

- Arduino

- TI CC2640

- Linux (still in implementation)

- Other MCU only by providing appropriate SPI interface

## Getting Started

- Adjust ```spi.h``` to your platform such as ```#include "spi_arduino.h"``` or ```#include "spi_cc2640.h"```

## Development

- For using PlatformIO, you have to put this project inside an empty root directory, otherwise PlatformIO will build all other directories as libraries.

- Build by ```pipenv run build```.
