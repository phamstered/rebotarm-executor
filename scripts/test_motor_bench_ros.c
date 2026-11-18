#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bench_ros.h"

#if MOTOR_BENCH_JOINT7_POSVEL
static void emit(const uint32_t *words, size_t count)
{
    printf("{\"data\":[");
    for (size_t i = 0U; i < count; ++i) printf("%s%u", i ? "," : "", words[i]);
    puts("]}");
}

int main(int argc, char **argv)
{
    (void)argv;
    MotorBenchDiagnostics admission = {0};
    MotorBenchLoopDiagnostics loop = {0};
    MotorBenchTraceMeta meta = {0};
    MotorBenchRuntimeDiagnostics runtime = {10000U, 9000U, 777U, 5U};
    admission.magic = 0x4a374233U;
    admission.complete = 1U;
    admission.read_mask = 0xfffU;
    admission.admitted_mask = 0x40U;
    admission.position_min = 0.48f;
    admission.position_max = 0.51f;
    admission.velocity_max = 0.05f;
    admission.startup_position = 0.5f;
    for (uint32_t i = 0U; i < 12U; ++i) admission.raw[i] = 0x3f800000U + i;
    admission.trip_velocity = -0.1f;
    loop.version = loop.complete = 1U;
    loop.read_mask = 0x1ffU;
    loop.started_ms = 10U;
    loop.completed_ms = 20U;
    for (uint32_t i = 0U; i < 9U; ++i) loop.raw[i] = 0x40000000U + i;
    meta.version = 1U;
    meta.first_sequence = 0xfffffffcU;
    meta.next_sequence = 4U;
    meta.count = 8U;
    meta.armed = meta.triggered = meta.frozen = 1U;
    meta.trigger_kind = MOTOR_BENCH_TRACE_STOP;
    meta.trigger_ms = 50U;
    meta.tail_count = 1U;
    uint32_t words[MOTOR_BENCH_ROS_SNAPSHOT_WORDS + 1U];
    memset(words, 0xa5, sizeof(words));
    assert(motor_bench_ros_encode_snapshot(words, 86U, 1U, 2U,
        &admission, &loop, &meta, &runtime) == 0U);
    assert(words[0] == 0xa5a5a5a5U);
    assert(motor_bench_ros_encode_snapshot(NULL, 87U, 1U, 2U,
        &admission, &loop, &meta, &runtime) == 0U);
    size_t count = motor_bench_ros_encode_snapshot(words, 87U,
        0x1234567887654321ULL, 123U, &admission, &loop, &meta, &runtime);
    assert(count == 87U && words[87] == 0xa5a5a5a5U);
    assert(words[3] == 0x87654321U && words[4] == 0x12345678U && words[5] == 123U);
    assert(words[6] == 0U && words[13] == 5U && words[14] == 0U && words[15] == 0U);
    assert(words[7] == 10000U && words[8] == 9000U && words[9] == 777U);
    assert(words[10] == 37U && words[11] == 23U && words[12] == 11U);
    assert(memcmp(words + 16U, &admission, 148U) == 0);
    assert(memcmp(words + 53U, &loop, 92U) == 0);
    assert(words[76] == 1U && words[77] == 0xfffffffcU && words[78] == 4U);
    if (argc > 1) emit(words, count);

    MotorBenchTracePage page = {meta, meta.first_sequence, 8U, 0U};
    MotorBenchTraceRecord records[8];
    for (uint32_t i = 0U; i < 8U; ++i) {
        records[i] = (MotorBenchTraceRecord){30U + i,
            motor_bench_trace_tag(MOTOR_BENCH_TRACE_TX_OK, 7U, 0x107U, 8U),
            0x3f000000U, 0x3d4ccccdU};
    }
    records[6] = (MotorBenchTraceRecord){50U,
        motor_bench_trace_tag(MOTOR_BENCH_TRACE_STOP, 7U, 0U, 0U), 123U, 0U};
    records[7].ms = 51U;
    memset(words, 0xa5, sizeof(words));
    assert(motor_bench_ros_encode_trace(words, 55U, 1U, 2U, &page, records) == 0U);
    assert(words[0] == 0xa5a5a5a5U);
    assert(motor_bench_ros_encode_trace(words, 56U, 1U, 2U, &page, NULL) == 0U);
    count = motor_bench_ros_encode_trace(words, 56U, 0x1234567887654321ULL,
                                        124U, &page, records);
    assert(count == 56U && words[56] == 0xa5a5a5a5U);
    assert(words[18] == 0xfffffffcU && words[19] == 8U && words[20] == 0U);
    assert(words[21] == 512U && words[22] == 4U && words[23] == 0U);
    assert(memcmp(words + 24U, records, 128U) == 0);
    if (argc > 1) emit(words, count);
    page.count = 9U;
    assert(motor_bench_ros_encode_trace(words, 87U, 1U, 2U, &page, records) == 0U);
    page.count = 0U;
    assert(motor_bench_ros_encode_trace(words, 24U, 1U, 2U, &page, NULL) == 24U);

    MotorBenchCalibrationDiagnostics calibration = {0};
    calibration.version = calibration.complete = 1U;
    calibration.read_mask = 0xffU;
    calibration.started_ms = 21U;
    calibration.completed_ms = 29U;
    for (uint32_t i = 0U; i < 8U; ++i) calibration.raw[i] = 0x3f800000U + i;
    calibration.raw[0] = 10000U; /* TIMEOUT is a u32 count of 50 us units. */
    calibration.raw[2] = 21U; /* NPP is u32, not float32. */
    memset(words, 0xa5, sizeof(words));
    assert(motor_bench_ros_encode_calibration(words, 28U, 1U, 2U, &calibration) == 0U);
    assert(words[0] == 0xa5a5a5a5U);
    assert(motor_bench_ros_encode_calibration(NULL, 29U, 1U, 2U, &calibration) == 0U);
    assert(motor_bench_ros_encode_calibration(words, 29U, 1U, 2U, NULL) == 0U);
    assert(words[0] == 0xa5a5a5a5U);
    count = motor_bench_ros_encode_calibration(words, 29U, 0x1234567887654321ULL,
                                              125U, &calibration);
    assert(count == 29U && words[29] == 0xa5a5a5a5U);
    assert(words[0] == 0x4a374331U && words[1] == 1U && words[2] == 29U);
    assert(words[3] == 0x87654321U && words[4] == 0x12345678U && words[5] == 125U);
    assert(words[6] == 0U && words[7] == 0U);
    assert(memcmp(words + 8U, &calibration, 84U) == 0);
    if (argc > 1) emit(words, count);
    /* Raw nonfinite values and signed failures are observations, preserved verbatim. */
    calibration.complete = 0U;
    calibration.read_mask = 0U;
    calibration.raw[1] = 0x7fc00000U;
    calibration.errors[1] = -6;
    assert(motor_bench_ros_encode_calibration(words, 29U, 1U, 2U, &calibration) == 29U);
    assert(words[9] == 0U && words[10] == 0U && words[14] == 0x7fc00000U);
    assert(words[22] == 0xfffffffaU && words[29] == 0xa5a5a5a5U);
    assert(memcmp(words + 8U, &calibration, 84U) == 0);
    if (argc == 1) puts("bench ROS wire encoding tests passed");
    return 0;
}
#else
#ifdef MOTOR_BENCH_ROS_SNAPSHOT_WORDS
#error "Production build must not expose bench ROS wire constants"
#endif
#ifdef MOTOR_BENCH_TRACE_CAPACITY
#error "Production wire header must not include trace storage/helpers"
#endif
#ifdef MOTOR_BENCH_ROS_CALIBRATION_WORDS
#error "Production build must not expose bench calibration encoding"
#endif
int main(void)
{
    puts("production build excludes bench ROS encoding");
    return 0;
}
#endif
