#include "../config.h"
#if defined(PLATFORM_ARDUINO)

#include <Arduino.h>
#include "../util.h"

uint64_t util_msecs() {
    return millis();
}

void util_delay(uint32_t ms) {
    return delay(ms);
}

#endif
