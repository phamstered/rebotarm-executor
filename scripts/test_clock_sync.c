#include "microros_clock_sync.h"
#include <assert.h>
int main(void)
{
    MicrorosClockSync s = {0};
    microros_clock_reset(&s);
    assert(microros_clock_stamp(&s, 100) == 0);
    microros_clock_sample(&s, 100, 1000000100, false, 10);
    assert(!s.valid && !microros_clock_due(&s, 1099));
    assert(microros_clock_due(&s, 1100));
    microros_clock_sample(&s, 1100, 1000001100, true, 3);
    assert(microros_clock_stamp(&s, 1104) == 1000001104);
    assert(!microros_clock_due(&s, 6099) && microros_clock_due(&s, 6100));
    uint32_t generation = s.generation;
    microros_clock_sample(&s, 6100, 1000006070, true, 5);
    int64_t previous = microros_clock_stamp(&s, 6100);
    for (uint64_t t = 6104; t < 6500; t += 4) {
        int64_t stamp = microros_clock_stamp(&s, t);
        assert(stamp > previous); previous = stamp;
    }
    assert(s.generation == generation);
    /* A 15-second backward correction establishes a new epoch immediately. */
    microros_clock_sample(&s, 7000, 999992000, true, 4);
    assert(s.generation == generation + 1);
    assert(microros_clock_stamp(&s, 7000) == 999992000);
    microros_clock_sample(&s, 12000, 0, false, 10);
    assert(microros_clock_state(&s, 21999) == 1);
    assert(microros_clock_state(&s, 22000) == 2 && microros_clock_stamp(&s, 22000) == 0);
    microros_clock_sample(&s, 22000, 1000022000, true, 21);
    assert(microros_clock_state(&s, 22000) == 2);
    microros_clock_sample(&s, 24000, 1000024000, true, 5);
    assert(microros_clock_state(&s, 24000) == 1);
    microros_clock_reset(&s);
    assert(microros_clock_age(&s, 25000) == UINT32_MAX);
    /* The monotonic input is 64-bit, including a 32-bit RTOS tick wrap. */
    microros_clock_sample(&s, UINT32_MAX - 2ULL, 1700000000000LL, true, 2);
    assert(microros_clock_stamp(&s, UINT32_MAX + 6ULL) == 1700000000008LL);
    generation = s.generation;
    microros_clock_sample(&s, UINT32_MAX + 10ULL, 1700000000300LL, true, 2);
    assert(s.generation == generation + 1); /* Large forward correction. */
    microros_clock_reset(&s);
    uint64_t attempt = 100;
    for (unsigned i = 0; i < 5; ++i) {
        microros_clock_sample(&s, attempt, 0, false, 10);
        uint32_t delay = i == 0 ? 1000 : i == 1 ? 2000 : 5000;
        assert(!microros_clock_due(&s, attempt + delay - 1));
        assert(microros_clock_due(&s, attempt + delay));
        attempt += delay;
    }
    return 0;
}
