#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include "motor_bench_trace.h"

static MotorBenchTraceRecord records[MOTOR_BENCH_TRACE_CAPACITY];
static MotorBenchTraceMeta meta;

static void append(uint32_t now, uint32_t value)
{
    MotorBenchTraceRecord record = {
        now, motor_bench_trace_tag(MOTOR_BENCH_TRACE_RX, 7, 0x17, 8), value, ~value
    };
    motor_bench_trace_append(&meta, records, &record, now);
}

int main(void)
{
    memset(records, 0xa5, sizeof(records));
    memset(&meta, 0xa5, sizeof(meta));
    motor_bench_trace_init(&meta, records);
    assert(sizeof(records) == 8192 && meta.version == 1 && !meta.count);
    for (unsigned i = 0; i < MOTOR_BENCH_TRACE_CAPACITY; ++i)
        assert(!records[i].ms && !records[i].tag && !records[i].data0 && !records[i].data1);
    /* Startup/preflight stops do not consume the one capture. */
    motor_bench_trace_trigger(&meta, records, MOTOR_BENCH_TRACE_STOP, 1, 0, 0);
    assert(!meta.triggered && !meta.frozen && !meta.count);

    for (unsigned i = 0; i < 520; ++i) append(i, i);
    assert(meta.count == 512 && meta.overwritten == 8);
    assert(meta.first_sequence == 8 && meta.next_sequence == 520);
    MotorBenchTraceRecord page_records[9];
    memset(page_records, 0x5a, sizeof(page_records));
    MotorBenchTracePage page;
    assert(motor_bench_trace_copy(&meta, records, 0, page_records, 9, &page) == 8);
    assert(page.gap && page.first_sequence == 8 && page.meta.overwritten == 8);
    assert(page_records[0].data0 == 8 && page_records[7].data0 == 15);
    assert(page_records[8].ms == 0x5a5a5a5a); /* Hard page-size bound. */
    assert(motor_bench_trace_copy(&meta, records, 518, page_records, 8, &page) == 2);
    assert(!page.gap && page_records[1].data0 == 519);
    assert(motor_bench_trace_copy(&meta, records, 520, page_records, 8, &page) == 0);
    assert(!page.gap);
    assert(motor_bench_trace_copy(&meta, records, 521, page_records, 8, &page) == 8 && page.gap);
    assert(meta.next_sequence == 520); /* Copy is not consuming. */

    meta.armed = 1;
    motor_bench_trace_trigger(&meta, records, MOTOR_BENCH_TRACE_TRIP, 55, 4, 600);
    assert(meta.triggered && !meta.frozen && meta.trigger_kind == MOTOR_BENCH_TRACE_TRIP);
    assert(meta.trigger_ms == 600 && !meta.tail_count);
    motor_bench_trace_tick(&meta, 599);
    assert(!meta.frozen); /* A preempted task may carry an older sampled tick. */
    const uint32_t triggered_sequence = meta.next_sequence - 1;
    assert(motor_bench_trace_copy(&meta, records, triggered_sequence, page_records, 8, &page) == 1);
    assert(page_records[0].data0 == 55 && page_records[0].data1 == 4);
    motor_bench_trace_trigger(&meta, records, MOTOR_BENCH_TRACE_STOP, 56, 0, 610);
    assert(meta.trigger_kind == MOTOR_BENCH_TRACE_TRIP && meta.trigger_ms == 600);
    for (unsigned i = 0; i < 31; ++i) append(620, 900 + i);
    assert(!meta.frozen && meta.tail_count == 31);
    append(621, 931);
    assert(meta.frozen && meta.tail_count == 32);
    uint32_t stopped_sequence = meta.next_sequence;
    append(622, 999);
    motor_bench_trace_trigger(&meta, records, MOTOR_BENCH_TRACE_TRIP, 99, 5, 623);
    assert(meta.next_sequence == stopped_sequence && meta.trigger_ms == 600);

    motor_bench_trace_init(&meta, records);
    meta.armed = 1;
    motor_bench_trace_trigger(&meta, records, MOTOR_BENCH_TRACE_STOP, 57, 0, UINT32_MAX - 49);
    append(49, 1);
    assert(!meta.frozen && meta.tail_count == 1);
    motor_bench_trace_tick(&meta, 50);
    assert(meta.frozen); /* Deadline works across HAL tick wrap, without traffic. */

    motor_bench_trace_init(&meta, records);
    meta.first_sequence = meta.next_sequence = UINT32_MAX - 1;
    for (unsigned i = 0; i < 4; ++i) append(i, 100 + i);
    assert(meta.next_sequence == 2 && meta.first_sequence == UINT32_MAX - 1);
    assert(motor_bench_trace_copy(&meta, records, UINT32_MAX, page_records, 8, &page) == 3);
    assert(!page.gap && page_records[0].data0 == 101 && page_records[2].data0 == 103);
    assert(motor_bench_trace_tag(MOTOR_BENCH_TRACE_TX_ERROR, 7, 0x107, 8) ==
        (0x107U | (8U << 11) | (7U << 15) | (2U << 18)));
    puts("joint7 bounded trace tests passed");
}
