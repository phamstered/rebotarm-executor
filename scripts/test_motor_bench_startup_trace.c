#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bench_ros.h"

static void append(MotorBenchStartupMeta *meta, MotorBenchTraceRecord *records,
    uint32_t now, uint32_t kind, uint32_t can_id, uint32_t data0, uint32_t data1)
{
    MotorBenchTraceRecord record = {now, motor_bench_trace_tag(kind, 7U, can_id,
        kind <= MOTOR_BENCH_TRACE_RX ? 8U : 0U), data0, data1};
    motor_bench_startup_append(meta, records, &record, now);
}

static void emit(const uint32_t *words, size_t count)
{
    printf("{\"data\":[");
    for (size_t i = 0U; i < count; ++i) printf("%s%u", i ? "," : "", words[i]);
    puts("]}");
}

int main(int argc, char **argv)
{
    (void)argv;
    MotorBenchStartupMeta meta;
    MotorBenchTraceRecord records[MOTOR_BENCH_STARTUP_CAPACITY], page_records[8];
    MotorBenchStartupPage page;
    motor_bench_startup_init(&meta, records);
    for (uint32_t now = 0U; now < 100000U; ++now)
        append(&meta, records, now, MOTOR_BENCH_TRACE_RX, 0x17U, 7U, 0U);
    assert(meta.count == 0U && !meta.armed); /* arbitrarily long idle is free */
    motor_bench_startup_begin(&meta, records, 123U, 100U);
    motor_bench_startup_tick(&meta, 99U); /* stale concurrent tick */
    assert(!meta.frozen);
    append(&meta, records, 100U, MOTOR_BENCH_TRACE_ACCEPTED_V2, 0U, 123U,
           0U | (2U << 8) | (0x40U << 16)); /* first command is HOLD */
    append(&meta, records, 101U, MOTOR_BENCH_TRACE_TX_OK, 0x7ffU, 0x50330007U, 0U);
    append(&meta, records, 102U, MOTOR_BENCH_TRACE_RX, 0x17U, 0x50330007U, 0x3f000000U);
    append(&meta, records, 103U, MOTOR_BENCH_TRACE_TX_OK, 0x107U, 0x3f000000U, 0x3d4ccccdU);
    append(&meta, records, 104U, MOTOR_BENCH_TRACE_TX_OK, 0x107U, 0xffffffffU, 0xfcffffffU);
    for (uint32_t now = 105U; now <= 107U; ++now)
        append(&meta, records, now, MOTOR_BENCH_TRACE_RX, 0x17U, 0x17U, 0U);
    assert(meta.count == 9U && meta.enable_enqueued && meta.enable_ms == 104U);
    motor_bench_startup_tick(&meta, 203U);
    assert(!meta.frozen);
    motor_bench_startup_tick(&meta, 204U);
    assert(meta.frozen && meta.reason == MOTOR_BENCH_STARTUP_WINDOW);
    MotorBenchStartupMeta frozen = meta;
    MotorBenchTraceRecord saved[MOTOR_BENCH_STARTUP_CAPACITY];
    memcpy(saved, records, sizeof(saved));

    /* Independent old ring keeps rolling for thousands of frames, then stops;
     * neither it nor new attempts can erase the original prefix. */
    MotorBenchTraceMeta tail_meta;
    MotorBenchTraceRecord tail[MOTOR_BENCH_TRACE_CAPACITY];
    motor_bench_trace_init(&tail_meta, tail);
    tail_meta.armed = 1U;
    for (uint32_t now = 205U; now < 5000U; ++now) {
        MotorBenchTraceRecord record = {now,
            motor_bench_trace_tag(MOTOR_BENCH_TRACE_RX, 7U, 0x17U, 8U), 0x17U, 0U};
        motor_bench_trace_append(&tail_meta, tail, &record, now);
        motor_bench_startup_append(&meta, records, &record, now);
    }
    motor_bench_startup_begin(&meta, records, 999U, 5000U);
    motor_bench_startup_stop(&meta, records, MOTOR_BENCH_TRACE_STOP, 999U, 0U, 5000U);
    motor_bench_trace_trigger(&tail_meta, tail, MOTOR_BENCH_TRACE_STOP, 999U, 0U, 5000U);
    motor_bench_trace_tick(&tail_meta, 5100U);
    assert(tail_meta.frozen);
    assert(tail_meta.overwritten > 4000U);
    assert(memcmp(&meta, &frozen, sizeof(meta)) == 0);
    assert(memcmp(saved, records, sizeof(saved)) == 0);

    uint32_t words[MOTOR_BENCH_ROS_STARTUP_MAX_WORDS + 1U];
    for (uint32_t offset = 0U; offset < meta.count; offset += page.count) {
        assert(motor_bench_startup_copy(&meta, records, offset, page_records, 8U, &page) > 0U);
        memset(words, 0xa5, sizeof(words));
        size_t length = 22U + page.count * 4U;
        assert(motor_bench_ros_encode_startup(words, length - 1U, 1U, 2U,
            &page, page_records) == 0U);
        assert(words[0] == 0xa5a5a5a5U);
        assert(motor_bench_ros_encode_startup(words, length, 0x1234567887654321ULL,
            5101U, &page, page_records) == length);
        assert(words[length] == 0xa5a5a5a5U);
        assert(words[0] == 0x4a374531U && words[1] == 1U && words[2] == length);
        assert(words[3] == 0x87654321U && words[4] == 0x12345678U && words[6] == 0U);
        assert(words[16] == offset && words[18] == 0U && words[19] == 256U &&
               words[20] == 4U && words[21] == 0U);
        assert(memcmp(words + 22U, records + offset, page.count * 16U) == 0);
        if (argc > 1) emit(words, length);
    }
    assert(motor_bench_startup_copy(&meta, records, 999U, page_records, 9U, &page) == 8U);
    assert(page.gap && page.first_sequence == 0U);
    assert(motor_bench_startup_copy(&meta, records, meta.count, page_records, 8U, &page) == 0U);
    assert(!page.gap);
    assert(motor_bench_startup_copy(&meta, records, 0U, NULL, 8U, &page) == 0U);
    page.count = 9U;
    assert(!motor_bench_ros_encode_startup(words, sizeof(words)/4U, 1U, 2U, &page, page_records));
    page.count = 1U;
    assert(!motor_bench_ros_encode_startup(NULL, 54U, 1U, 2U, &page, page_records));
    assert(!motor_bench_ros_encode_startup(words, 54U, 1U, 2U, &page, NULL));

    motor_bench_startup_init(&meta, records);
    motor_bench_startup_begin(&meta, records, 1U, 0xffffff00U);
    /* Failed FC enqueue does not start the enabled observation window. */
    append(&meta, records, 0xffffff01U, MOTOR_BENCH_TRACE_TX_ERROR, 0x107U,
           0xffffffffU, 0xfcffffffU);
    motor_bench_startup_tick(&meta, 243U);
    assert(!meta.frozen && !meta.enable_enqueued);
    motor_bench_startup_tick(&meta, 244U);
    assert(meta.frozen && meta.reason == MOTOR_BENCH_STARTUP_TIMEOUT);

    motor_bench_startup_init(&meta, records);
    motor_bench_startup_begin(&meta, records, 1U, 0xfffffff0U);
    append(&meta, records, 0xfffffff1U, MOTOR_BENCH_TRACE_TX_OK, 0x107U,
           0xffffffffU, 0xfcffffffU);
    motor_bench_startup_tick(&meta, 84U);
    assert(!meta.frozen);
    motor_bench_startup_tick(&meta, 85U);
    assert(meta.frozen && meta.reason == MOTOR_BENCH_STARTUP_WINDOW);

    motor_bench_startup_init(&meta, records);
    motor_bench_startup_begin(&meta, records, 1U, 1U);
    for (uint32_t i = 1U; i < 256U; ++i)
        append(&meta, records, 2U, MOTOR_BENCH_TRACE_RX, 0x17U, 7U, i);
    assert(meta.frozen && meta.reason == MOTOR_BENCH_STARTUP_CAPACITY_FULL && meta.count == 256U);
    uint32_t first = records[0].tag, last = records[255].data1;
    append(&meta, records, 3U, MOTOR_BENCH_TRACE_RX, 0x17U, 7U, 999U);
    assert(records[0].tag == first && records[255].data1 == last);
    for (uint32_t kind = MOTOR_BENCH_TRACE_STOP; kind <= MOTOR_BENCH_TRACE_TRIP; ++kind) {
        motor_bench_startup_init(&meta, records);
        motor_bench_startup_begin(&meta, records, 1U, 1U);
        motor_bench_startup_stop(&meta, records, kind, 2U, 3U, 2U);
        assert(meta.frozen && meta.count == 2U && meta.reason == kind - 1U);
        assert(records[1].data0 == 2U && records[1].data1 == 3U);
    }
    if (argc == 1) puts("startup prefix preservation, bounds, and wire tests passed");
    return 0;
}
