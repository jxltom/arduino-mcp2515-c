#include "../config.h"
#if defined(PLATFORM_CC2640)

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include "../util.h"

unsigned long util_millis() {
    return 0;
}

void util_delay(unsigned int ms) {
    return Task_sleep(ms * 1000 / Clock_tickPeriod);
}

#endif
