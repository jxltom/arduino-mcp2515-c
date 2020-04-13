#ifndef _UTIL_H_
#define _UTIL_H_

#include "config.h"

#if defined(PLATFORM_ARDUINO)
#include "util_arduino.h"
#elif defined(PLATFORM_CC2640)
#include "util_cc2640.h"
#endif

#endif
