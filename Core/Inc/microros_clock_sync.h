#ifndef MICROROS_CLOCK_SYNC_H
#define MICROROS_CLOCK_SYNC_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ROS epoch mapping only. Motor watchdogs continue to use HAL_GetTick(). */
typedef struct {
    bool valid;
    uint64_t next_attempt_ms, last_sync_ms, slew_at_ms;
    int64_t offset_ms, target_offset_ms;
    uint32_t success_count, failure_count, duration_ms, max_duration_ms;
    uint32_t generation, failures;
    int32_t correction_ms;
} MicrorosClockSync;

static inline void microros_clock_reset(MicrorosClockSync *s)
{
    s->valid = false;
    s->next_attempt_ms = 0;
    s->failures = 0;
    s->generation++;
}

static inline uint32_t microros_clock_state(const MicrorosClockSync *s, uint64_t now)
{
    return !s->valid ? 0U : now - s->last_sync_ms >= 15000U ? 2U : 1U;
}

static inline uint32_t microros_clock_age(const MicrorosClockSync *s, uint64_t now)
{
    if (!s->valid) return UINT32_MAX;
    uint64_t age = now - s->last_sync_ms;
    return age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
}

static inline bool microros_clock_due(const MicrorosClockSync *s, uint64_t now)
{
    return now >= s->next_attempt_ms;
}

static inline int64_t microros_clock_stamp(MicrorosClockSync *s, uint64_t now)
{
    if (microros_clock_state(s, now) != 1U) return 0;
    /* Slew at <=100 ms/s, preserving increasing timestamps within an epoch. */
    uint64_t budget = (now - s->slew_at_ms) / 10U;
    if (budget) {
        int64_t delta = s->target_offset_ms - s->offset_ms;
        if (delta > (int64_t)budget) delta = (int64_t)budget;
        if (delta < -(int64_t)budget) delta = -(int64_t)budget;
        s->offset_ms += delta;
        s->slew_at_ms += budget * 10U;
    }
    return (int64_t)now + s->offset_ms;
}

static inline void microros_clock_sample(MicrorosClockSync *s, uint64_t now,
                                         int64_t epoch_ms, bool success,
                                         uint32_t duration_ms)
{
    s->duration_ms = duration_ms;
    if (duration_ms > s->max_duration_ms) s->max_duration_ms = duration_ms;
    /* Reject an over-budget call even if the library accepted its reply. */
    if (!success || epoch_ms <= 0 || epoch_ms / 1000 > INT32_MAX || duration_ms > 20U) {
        s->failure_count++;
        uint32_t delay = s->failures == 0 ? 1000U : s->failures == 1 ? 2000U : 5000U;
        if (s->failures < 2U) s->failures++;
        s->next_attempt_ms = now + delay;
        return;
    }
    int64_t current = microros_clock_stamp(s, now);
    int64_t offset = epoch_ms - (int64_t)now;
    int64_t correction = current ? epoch_ms - current : 0;
    s->correction_ms = correction > INT32_MAX ? INT32_MAX : correction < INT32_MIN ? INT32_MIN : (int32_t)correction;
    if (!current || correction > 100 || correction < -100) {
        s->offset_ms = offset;
        s->slew_at_ms = now;
        s->generation++;
    }
    s->target_offset_ms = offset;
    s->last_sync_ms = now;
    s->next_attempt_ms = now + 5000U;
    s->valid = true;
    s->failures = 0;
    s->success_count++;
}
#endif
