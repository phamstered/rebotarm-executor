#include <assert.h>
#include <stdio.h>
#include "motor_bench.h"

static unsigned calls;
static int bad_rid = -1;
static float damp = 4.0f, vmax = 30.0f;
static int read_register(uint8_t rid, uint32_t *value, void *context)
{
    (void)context;
    ++calls;
    if (rid == bad_rid) return -3;
    float f = rid == 11 ? damp : rid == 21 ? 12.5f : rid == 22 ? vmax : rid == 23 ? 10.0f : 1.0f;
    memcpy(value, &f, sizeof(f));
    if (rid == 13 || rid == 14 || rid == 15 || rid == 36) *value = 123;
    return 0;
}
int main(void)
{
    MotorBenchDiagnostics d;
    assert(motor_bench_collect(&d, true, 0.50f, read_register, NULL));
    assert(d.complete && d.read_mask == 0x7ff && d.admitted_mask == 0x40 && calls == 11);
    assert(d.raw[0] == 123 && motor_bench_float(d.raw[4]) == 4.0f);
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
    damp = 0; assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
    damp = NAN; assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
    damp = 4; vmax = 50; assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
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
    puts("joint7 bench diagnostic admission tests passed");
}
