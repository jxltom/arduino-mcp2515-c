#include "../config.h"
#if defined(PLATFORM_ARDUINO)

#include <Arduino.h>
#include "../util.h"

unsigned long util_millis() {
    return millis();
}

void util_delay(unsigned int ms) {
    return delay(ms);
}

#endif
