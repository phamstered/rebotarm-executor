#ifndef MOTOR_BENCH_STARTUP_TRACE_H
#define MOTOR_BENCH_STARTUP_TRACE_H

#include "motor_bench_trace.h"

#define MOTOR_BENCH_STARTUP_CAPACITY 256U
#define MOTOR_BENCH_STARTUP_ENABLE_WINDOW_MS 100U
#define MOTOR_BENCH_STARTUP_NO_ENABLE_MS 500U
#define MOTOR_BENCH_STARTUP_START_KIND 7U

typedef enum {
    MOTOR_BENCH_STARTUP_NONE = 0,
    MOTOR_BENCH_STARTUP_WINDOW = 1,
    MOTOR_BENCH_STARTUP_CAPACITY_FULL = 2,
    MOTOR_BENCH_STARTUP_TIMEOUT = 3,
    MOTOR_BENCH_STARTUP_STOP = 4,
    MOTOR_BENCH_STARTUP_TRIP = 5
} MotorBenchStartupFreezeReason;

/* Independent first-attempt prefix, never a ring. Once armed it cannot be
 * reset until boot. A frozen prefix does not imply the enable sequence exists
 * or succeeded; the exported records must establish those facts separately. */
typedef struct {
    uint32_t version, armed, frozen, reason, count, start_ms;
    uint32_t enable_enqueued, enable_ms, command_sequence;
} MotorBenchStartupMeta;

typedef struct {
    MotorBenchStartupMeta meta;
    uint32_t first_sequence, count, gap;
} MotorBenchStartupPage;

_Static_assert(sizeof(MotorBenchStartupMeta) == 36U, "startup metadata ABI");

static inline void motor_bench_startup_init(MotorBenchStartupMeta *meta,
    MotorBenchTraceRecord *records)
{
    memset(meta, 0, sizeof(*meta));
    memset(records, 0, MOTOR_BENCH_STARTUP_CAPACITY * sizeof(*records));
    meta->version = 1U;
}

/* The adapter calls this immediately before the first pre-enable precise read,
 * while outputs are disabled. Configuration/idle feedback cannot consume it. */
static inline void motor_bench_startup_begin(MotorBenchStartupMeta *meta,
    MotorBenchTraceRecord *records, uint32_t command_sequence, uint32_t now)
{
    if (meta->armed) return;
    meta->armed = 1U;
    meta->start_ms = now;
    meta->command_sequence = command_sequence;
    records[0] = (MotorBenchTraceRecord){now,
        motor_bench_trace_tag(MOTOR_BENCH_STARTUP_START_KIND, 7U, 0U, 0U),
        command_sequence, 0U};
    meta->count = 1U;
}

static inline void motor_bench_startup_tick(MotorBenchStartupMeta *meta, uint32_t now)
{
    if (!meta->armed || meta->frozen) return;
    /* Signed elapsed comparisons also tolerate a tick sampled just before a
     * concurrent begin/FC hook, and uint32 tick wrap within this short window. */
    if (meta->enable_enqueued) {
        if ((int32_t)(now - meta->enable_ms) <
            (int32_t)MOTOR_BENCH_STARTUP_ENABLE_WINDOW_MS) return;
        meta->reason = MOTOR_BENCH_STARTUP_WINDOW;
    } else {
        if ((int32_t)(now - meta->start_ms) <
            (int32_t)MOTOR_BENCH_STARTUP_NO_ENABLE_MS) return;
        meta->reason = MOTOR_BENCH_STARTUP_TIMEOUT;
    }
    meta->frozen = 1U;
}

static inline void motor_bench_startup_append(MotorBenchStartupMeta *meta,
    MotorBenchTraceRecord *records, const MotorBenchTraceRecord *record, uint32_t now)
{
    motor_bench_startup_tick(meta, now);
    if (!meta->armed || meta->frozen) return;
    records[meta->count++] = *record;
    if (!meta->enable_enqueued && record->tag ==
        motor_bench_trace_tag(MOTOR_BENCH_TRACE_TX_OK, 7U, 0x107U, 8U) &&
        record->data0 == 0xffffffffU && record->data1 == 0xfcffffffU) {
        meta->enable_enqueued = 1U;
        meta->enable_ms = record->ms;
    }
    if (meta->count == MOTOR_BENCH_STARTUP_CAPACITY) {
        meta->reason = MOTOR_BENCH_STARTUP_CAPACITY_FULL;
        meta->frozen = 1U;
    }
}

static inline void motor_bench_startup_stop(MotorBenchStartupMeta *meta,
    MotorBenchTraceRecord *records, uint32_t kind, uint32_t sequence,
    uint32_t reason, uint32_t now)
{
    if (!meta->armed || meta->frozen ||
        (kind != MOTOR_BENCH_TRACE_STOP && kind != MOTOR_BENCH_TRACE_TRIP)) return;
    MotorBenchTraceRecord marker = {now,
        motor_bench_trace_tag(kind, 7U, 0U, 0U), sequence, reason};
    motor_bench_startup_append(meta, records, &marker, now);
    /* Preserve a timeout/capacity/window freeze that won the race. */
    if (meta->frozen) return;
    meta->reason = kind == MOTOR_BENCH_TRACE_STOP ?
        MOTOR_BENCH_STARTUP_STOP : MOTOR_BENCH_STARTUP_TRIP;
    meta->frozen = 1U;
}

static inline uint32_t motor_bench_startup_copy(const MotorBenchStartupMeta *meta,
    const MotorBenchTraceRecord *records, uint32_t start_sequence,
    MotorBenchTraceRecord *output, uint32_t capacity, MotorBenchStartupPage *page)
{
    page->meta = *meta;
    page->gap = start_sequence > meta->count;
    if (page->gap) start_sequence = 0U;
    uint32_t count = meta->count - start_sequence;
    if (capacity > MOTOR_BENCH_TRACE_PAGE_MAX) capacity = MOTOR_BENCH_TRACE_PAGE_MAX;
    if (count > capacity) count = capacity;
    if (!output) count = 0U;
    for (uint32_t i = 0U; i < count; ++i) output[i] = records[start_sequence + i];
    page->first_sequence = start_sequence;
    page->count = count;
    return count;
}

#endif
