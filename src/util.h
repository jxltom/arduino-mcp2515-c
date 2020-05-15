#ifndef _UTIL_H_
#define _UTIL_H_

#include "config.h"

#if defined(PLATFORM_ARDUINO)
#include "arduino/util_arduino.h"
#elif defined(PLATFORM_CC2640)
#include "cc2640/util_cc2640.h"
#elif defined(PLATFORM_CC2642)
#include "cc2642/util_cc2642.h"
#endif

#if defined(__cplusplus)
extern "C"
{
#endif

    uint64_t util_msecs();
    void util_delay(uint32_t ms);

#if defined(__cplusplus)
}
#endif

#endif
