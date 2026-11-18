#include <assert.h>
#include <stdio.h>
#include "motor_bench.h"

static unsigned calls;
static int bad_rid = -1;
/* Recorded joint7 values; Deta must be read separately from physical Damp. */
static float damp = 0.000212438f, acc = 2.0f, dec = -2.0f;
static float max_spd = 600.0f, vmax = 30.0f, deta = 4.0f;
static int read_register(uint8_t rid, uint32_t *value, void *context)
{
    (void)context;
    ++calls;
    if (rid == bad_rid) return -3;
    float f = rid == 11 ? damp : rid == 4 ? acc : rid == 5 ? dec :
        rid == 6 ? max_spd : rid == 31 ? deta :
        rid == 21 ? 12.5f : rid == 22 ? vmax : rid == 23 ? 10.0f : 1.0f;
    memcpy(value, &f, sizeof(f));
    if (rid == 13 || rid == 15) *value = 0; /* Reserved fields may be zero. */
    if (rid == 14) *value = 0x38313037;
    if (rid == 36) *value = 54;
    return 0;
}
int main(void)
{
    MotorBenchDiagnostics d;
    assert(motor_bench_collect(&d, true, 0.50f, read_register, NULL));
    assert(sizeof(d) == 148 && d.magic == 0x4a374233);
    assert(d.complete && d.read_mask == 0xfff && d.admitted_mask == 0x40 && calls == 12);
    assert(d.raw[0] == 0 && d.raw[1] == 0x38313037 && d.raw[2] == 0);
    assert(motor_bench_float(d.raw[4]) == damp && motor_bench_float(d.raw[6]) == -2.0f);
    assert(motor_bench_float(d.raw[11]) == 4.0f);
    calls = 0;
    assert(!motor_bench_collect(&d, false, 0.50f, read_register, NULL));
    assert(calls == 0 && d.admitted_mask == 0);
    assert(!motor_bench_collect(&d, true, 0.52f, read_register, NULL));
    assert(!motor_bench_collect(&d, true, NAN, read_register, NULL));
    for (unsigned i = 0; i < MOTOR_BENCH_REGISTER_COUNT; ++i) {
        bad_rid = motor_bench_registers[i];
        assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
        assert(!(d.read_mask & (1U << i)) && d.admitted_mask == 0);
    }
    bad_rid = -1;
    const float invalid_dec[] = {0.0f, -0.0f, 2.0f, NAN, INFINITY, -INFINITY};
    for (unsigned i = 0; i < sizeof(invalid_dec) / sizeof(*invalid_dec); ++i) {
        dec = invalid_dec[i];
        assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
        assert(d.admitted_mask == 0);
    }
    dec = -2.0f;
    const float invalid_deta[] = {0.0f, -4.0f, 0.999f, 30.001f, NAN, INFINITY, -INFINITY};
    for (unsigned i = 0; i < sizeof(invalid_deta) / sizeof(*invalid_deta); ++i) {
        deta = invalid_deta[i];
        assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
        assert(d.admitted_mask == 0); /* Positive Damp never substitutes for Deta. */
    }
    deta = 1.0f; assert(motor_bench_collect(&d, true, 0.50f, read_register, NULL));
    deta = 30.0f; assert(motor_bench_collect(&d, true, 0.50f, read_register, NULL));
    deta = 4.0f;
    float *positive_parameters[] = {&damp, &acc, &max_spd};
    const float invalid_positive[] = {0.0f, -2.0f, NAN, INFINITY, -INFINITY};
    for (unsigned i = 0; i < sizeof(positive_parameters) / sizeof(*positive_parameters); ++i) {
        float original = *positive_parameters[i];
        for (unsigned j = 0; j < sizeof(invalid_positive) / sizeof(*invalid_positive); ++j) {
            *positive_parameters[i] = invalid_positive[j];
            assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
        }
        *positive_parameters[i] = original;
    }
    max_spd = 0.001f; /* Different shaft reference: never compare against command vmax. */
    assert(motor_bench_collect(&d, true, 0.50f, read_register, NULL));
    max_spd = 600.0f;
    vmax = 50; assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
    assert(!motor_bench_feedback_reason(true, 100, 0.5f, 0.10f, 0.30f));
    assert(!motor_bench_feedback_reason(true, 100, 0.5f, -0.10f, -0.30f));
    assert(motor_bench_feedback_reason(true, 101, 0.5f, 0, 0) == 1);
    assert(motor_bench_feedback_reason(false, 0, 0.5f, 0, 0) == 1);
    assert(motor_bench_feedback_reason(true, 0, NAN, 0, 0) == 2);
    assert(!motor_bench_feedback_reason(true, 0, MOTOR_BENCH_POSITION_MIN - 0.003f, 0, 0));
    assert(!motor_bench_feedback_reason(true, 0, MOTOR_BENCH_POSITION_MAX + 0.003f, 0, 0));
    assert(motor_bench_feedback_reason(true, 0, MOTOR_BENCH_POSITION_MIN - 0.0031f, 0, 0) == 3);
    assert(motor_bench_feedback_reason(true, 0, MOTOR_BENCH_POSITION_MAX + 0.0031f, 0, 0) == 3);
    assert(motor_bench_feedback_reason(true, 0, 0.5f, -0.101f, 0) == 4);
    assert(motor_bench_feedback_reason(true, 0, 0.5f, 0, -0.301f) == 5);
    MotorBenchLoopDiagnostics loop;
    calls = 0;
    motor_bench_collect_loop(&loop, false, read_register, NULL);
    assert(loop.version == 1 && loop.complete && !loop.read_mask && !calls);
    for (unsigned i = 0; i < MOTOR_BENCH_LOOP_REGISTER_COUNT; ++i) assert(loop.errors[i] == -6);
    motor_bench_collect_loop(&loop, true, read_register, NULL);
    assert(loop.complete && loop.read_mask == 0x1ff && calls == 9);
    const uint8_t expected_rids[] = {20,24,25,26,27,28,32,33,34};
    assert(!memcmp(motor_bench_loop_registers, expected_rids, sizeof(expected_rids)));
    for (unsigned i = 0; i < MOTOR_BENCH_LOOP_REGISTER_COUNT; ++i) {
        bad_rid = expected_rids[i];
        motor_bench_collect_loop(&loop, true, read_register, NULL);
        assert(loop.read_mask == (0x1ffU & ~(1U << i)) && loop.errors[i] == -3);
    }
    puts("joint7 bench diagnostic admission tests passed");
}
