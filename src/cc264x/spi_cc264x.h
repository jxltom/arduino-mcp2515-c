#ifndef _SPI_CC264x_H_
#define _SPI_CC264x_H_

#if defined(PLATFORM_CC2640)
#include <ti/devices/cc26x0r2/driverlib/ioc.h>
#elif defined(PLATFORM_CC2642)
#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#endif

#define SPI_CLOCK 5000000  // 5MHz
#define SPICS IOID_20

#endif
