#include "../config.h"
#if defined(PLATFORM_CC2642)

#include <stdint.h>
#include <ti/sysbios/hal/Seconds.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include "../util.h"

uint64_t util_msecs() {
    Seconds_Time ts;
    Seconds_getTime(&ts);
    uint64_t milliseconds = (ts.secs * 1000) + (ts.nsecs / 1000000);
    return milliseconds;
}

void util_delay(uint32_t ms) {
    return Task_sleep(ms * 1000 / Clock_tickPeriod);
}

#endif
