#include "../config.h"
#if defined(PLATFORM_CC2640)

#include <ti/sysbios/hal/Seconds.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include "../util.h"

unsigned long util_millis() {
    Seconds_Time ts;
    Seconds_getTime(&ts);
    uint64_t milliseconds = (ts.secs * 1000) + (ts.nsecs / 1000000);
    return (unsigned long) milliseconds;
}

void util_delay(unsigned int ms) {
    return Task_sleep(ms * 1000 / Clock_tickPeriod);
}

#endif
