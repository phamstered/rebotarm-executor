#ifndef MOTOR_BENCH_TRACE_H
#define MOTOR_BENCH_TRACE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MOTOR_BENCH_TRACE_CAPACITY 512U
#define MOTOR_BENCH_TRACE_PAGE_MAX 8U
#define MOTOR_BENCH_TRACE_TAIL_MS 100U
#define MOTOR_BENCH_TRACE_TAIL_RECORDS 32U

/* TX kinds report HAL enqueue results only, never physical transmission. */
typedef enum {
    MOTOR_BENCH_TRACE_TX_OK = 1,
    MOTOR_BENCH_TRACE_TX_ERROR = 2,
    MOTOR_BENCH_TRACE_RX = 3,
    MOTOR_BENCH_TRACE_ACCEPTED_V2 = 4,
    MOTOR_BENCH_TRACE_STOP = 5,
    MOTOR_BENCH_TRACE_TRIP = 6
} MotorBenchTraceKind;

/* tag: CAN ID [0:10], DLC [11:14], axis 1..7 [15:17], kind [18:21].
 * CAN records contain exact bytes in data0/data1 (little endian). Markers use
 * data0=accepted/stop sequence; data1=mode|control_mode<<8|mask<<16, fault,
 * or bench trip reason, respectively. Record sequence is the page offset. */
typedef struct {
    uint32_t ms, tag, data0, data1;
} MotorBenchTraceRecord;

typedef struct {
    uint32_t version;
    uint32_t first_sequence, next_sequence, count, overwritten;
    uint32_t armed, triggered, frozen;
    uint32_t trigger_kind, trigger_ms, tail_count;
} MotorBenchTraceMeta;

typedef struct {
    MotorBenchTraceMeta meta;
    uint32_t first_sequence, count, gap;
} MotorBenchTracePage;

_Static_assert(sizeof(MotorBenchTraceRecord) == 16U, "trace record ABI");

/* Pure helpers; the adapter serializes all calls with its short RTOS critical
 * sections. The NOLOAD records must be explicitly initialized after boot. */
static inline void motor_bench_trace_init(MotorBenchTraceMeta *meta,
                                          MotorBenchTraceRecord *records)
{
    memset(meta, 0, sizeof(*meta));
    memset(records, 0, MOTOR_BENCH_TRACE_CAPACITY * sizeof(*records));
    meta->version = 1U;
}

static inline uint32_t motor_bench_trace_tag(uint32_t kind, uint32_t axis,
                                            uint32_t can_id, uint32_t dlc)
{
    return (can_id & 0x7ffU) | ((dlc & 15U) << 11) |
        ((axis & 7U) << 15) | ((kind & 15U) << 18);
}

static inline void motor_bench_trace_tick(MotorBenchTraceMeta *meta, uint32_t now)
{
    /* A caller can have sampled its tick just before the safety task triggered
     * capture. That older tick must not look like a uint32-sized elapsed time. */
    if (meta->triggered && !meta->frozen &&
        (int32_t)(now - meta->trigger_ms) >= (int32_t)MOTOR_BENCH_TRACE_TAIL_MS)
        meta->frozen = 1U;
}

static inline void motor_bench_trace_append(MotorBenchTraceMeta *meta,
    MotorBenchTraceRecord *records, const MotorBenchTraceRecord *record,
    uint32_t now)
{
    motor_bench_trace_tick(meta, now);
    if (meta->frozen) return;
    records[meta->next_sequence % MOTOR_BENCH_TRACE_CAPACITY] = *record;
    ++meta->next_sequence;
    if (meta->count < MOTOR_BENCH_TRACE_CAPACITY) ++meta->count;
    else ++meta->overwritten;
    meta->first_sequence = meta->next_sequence - meta->count;
    if (meta->triggered && ++meta->tail_count >= MOTOR_BENCH_TRACE_TAIL_RECORDS)
        meta->frozen = 1U;
}

static inline void motor_bench_trace_trigger(MotorBenchTraceMeta *meta,
    MotorBenchTraceRecord *records, uint32_t kind, uint32_t sequence,
    uint32_t reason, uint32_t now)
{
    if (!meta->armed || meta->triggered || meta->frozen) return;
    MotorBenchTraceRecord marker = {
        now, motor_bench_trace_tag(kind, 7U, 0U, 0U), sequence, reason
    };
    motor_bench_trace_append(meta, records, &marker, now);
    meta->triggered = 1U;
    meta->trigger_kind = kind;
    meta->trigger_ms = now;
}

/* Non-consuming pagination. A stale/future cursor returns the current earliest
 * page and gap=1; sequence comparisons remain valid across uint32 wrap. */
static inline uint32_t motor_bench_trace_copy(const MotorBenchTraceMeta *meta,
    const MotorBenchTraceRecord *records, uint32_t start_sequence,
    MotorBenchTraceRecord *output, uint32_t capacity, MotorBenchTracePage *page)
{
    page->meta = *meta;
    page->gap = 0U;
    uint32_t offset = start_sequence - meta->first_sequence;
    if (offset > meta->count) {
        start_sequence = meta->first_sequence;
        offset = 0U;
        page->gap = 1U;
    }
    uint32_t count = meta->count - offset;
    if (capacity > MOTOR_BENCH_TRACE_PAGE_MAX) capacity = MOTOR_BENCH_TRACE_PAGE_MAX;
    if (count > capacity) count = capacity;
    if (!output) count = 0U;
    for (uint32_t i = 0U; i < count; ++i)
        output[i] = records[(start_sequence + i) % MOTOR_BENCH_TRACE_CAPACITY];
    page->first_sequence = start_sequence;
    page->count = count;
    return count;
}

#endif
