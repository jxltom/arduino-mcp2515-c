#include "../config.h"
#if defined(PLATFORM_CC2640)

#include "../util.h"

unsigned long util_millis() {
    return 0;
}

void util_delay(unsigned int ms) {
    return;
}

#endif
