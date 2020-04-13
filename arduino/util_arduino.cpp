#include "config.h"
#if defined(PLATFORM_ARDUINO)

#include <Arduino.h>
#include "../util.h"

unsigned long current_millis() {
    return millis();
}

#endif
