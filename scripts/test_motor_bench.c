#include <assert.h>
#include <stdio.h>
#include "motor_bench.h"

static unsigned calls;
static int bad_rid = -1;
/* Recorded joint7 values; Deta must be read separately from physical Damp. */
static float damp = 0.000212438f, acc = 2.0f, dec = -2.0f;
static float max_spd = 600.0f, vmax = 30.0f, deta = 4.0f;
static uint32_t calibration_values[] = {
    UINT32_MAX, 0x37e501f9U, 14U, 0x3f19897aU, 0x39551d0eU, 0x3b900943U, 0U, 0x3f800000U
};
static int read_calibration(uint8_t rid, uint32_t *value, void *context)
{
    (void)context;
    assert(calls < MOTOR_BENCH_CALIBRATION_REGISTER_COUNT);
    assert(rid == motor_bench_calibration_registers[calls]);
    unsigned index = calls++;
    if (rid == bad_rid) return -3;
    *value = calibration_values[index];
    return 0;
}
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
    assert(d.raw[5] == 0x40000000U && d.raw[6] == 0xc0000000U);
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
    /* An MCU reset does not reset the drive's volatile ACC. Recollecting the
     * temporary ACC-only profile must preserve the evidence and deny admission. */
    acc = 0.0002f;
    assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
    assert(d.complete && d.read_mask == 0xfffU && d.admitted_mask == 0U);
    assert(motor_bench_float(d.raw[5]) == acc && d.raw[6] == 0xc0000000U);
    acc = 2.0f;
    assert(motor_bench_collect(&d, true, 0.50f, read_register, NULL));
    /* Positive ACC and negative DEC alone are insufficient: even one ULP from
     * either baseline value must fail, not pass an approximate comparison. */
    const uint32_t nonbaseline_acc[] = {0x3fffffffU, 0x40000001U};
    const uint32_t nonbaseline_dec[] = {0xbfffffffU, 0xc0000001U};
    for (unsigned i = 0U; i < 2U; ++i) {
        acc = motor_bench_float(nonbaseline_acc[i]);
        assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
        assert(d.raw[5] == nonbaseline_acc[i] && d.admitted_mask == 0U);
        acc = 2.0f;
        dec = motor_bench_float(nonbaseline_dec[i]);
        assert(!motor_bench_collect(&d, true, 0.50f, read_register, NULL));
        assert(d.raw[6] == nonbaseline_dec[i] && d.admitted_mask == 0U);
        dec = -2.0f;
    }
    assert(motor_bench_collect(&d, true, MOTOR_BENCH_POSITION_MIN, read_register, NULL));
    assert(motor_bench_collect(&d, true, MOTOR_BENCH_POSITION_MAX, read_register, NULL));
    assert(!motor_bench_collect(&d, true, MOTOR_BENCH_POSITION_MIN - 0.000001f,
                               read_register, NULL));
    assert(!motor_bench_collect(&d, true, MOTOR_BENCH_POSITION_MAX + 0.000001f,
                               read_register, NULL));
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
    assert(!motor_bench_feedback_reason(true, 100, 0.5f, 0.1f, 0.30f));
    assert(!motor_bench_feedback_reason(true, 100, 0.5f, -0.1f, -0.30f));
    assert(motor_bench_feedback_reason(true, 101, 0.5f, 0, 0) == 1);
    assert(motor_bench_feedback_reason(false, 0, 0.5f, 0, 0) == 1);
    assert(motor_bench_feedback_reason(true, 0, NAN, 0, 0) == 2);
    assert(motor_bench_feedback_reason(true, 0, 0.5f, INFINITY, 0) == 2);
    assert(motor_bench_feedback_reason(true, 0, 0.5f, -INFINITY, 0) == 2);
    assert(motor_bench_feedback_reason(true, 0, 0.5f, 0, NAN) == 2);
    assert(!motor_bench_feedback_reason(true, 0, MOTOR_BENCH_POSITION_MIN, 0, 0));
    assert(!motor_bench_feedback_reason(true, 0, MOTOR_BENCH_POSITION_MAX, 0, 0));
    assert(motor_bench_feedback_reason(true, 0, MOTOR_BENCH_POSITION_MIN - 0.000001f, 0, 0) == 3);
    assert(motor_bench_feedback_reason(true, 0, MOTOR_BENCH_POSITION_MAX + 0.000001f, 0, 0) == 3);
#if MOTOR_BENCH_OVERSPEED_STOP
    const float above_velocity_limit = motor_bench_float(0x3dccccceU); /* One ULP above 0.1f. */
    assert(above_velocity_limit > 0.1f);
    assert(motor_bench_feedback_reason(true, 0, 0.5f, above_velocity_limit, 0) == 4);
    assert(motor_bench_feedback_reason(true, 0, 0.5f, -above_velocity_limit, 0) == 4);
    assert(!motor_bench_feedback_reason(true, 0, 0.5f, 0.075f, 0));
    assert(!motor_bench_feedback_reason(true, 0, 0.5f, -0.075f, 0));
#else
    assert(!motor_bench_feedback_reason(true, 0, 0.5f, 0.15f, 0));
    assert(!motor_bench_feedback_reason(true, 0, 0.5f, -0.15f, 0));
#endif
#if MOTOR_BENCH_EFFORT_STOP
    assert(motor_bench_feedback_reason(true, 0, 0.5f, 0, -0.301f) == 5);
    assert(motor_bench_feedback_reason(true, 0, 0.5f, 0, 0.301f) == 5);
#else
    assert(!motor_bench_feedback_reason(true, 0, 0.5f, 0, -0.5f));
    assert(!motor_bench_feedback_reason(true, 0, 0.5f, 0, 0.5f));
#endif
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
    /* The independent calibration snapshot preserves raw values (including
     * legitimate u32 max/float zero and nonfinite floats) without qualifying. */
    MotorBenchCalibrationDiagnostics calibration;
    assert(sizeof(calibration) == 84U);
    const uint8_t expected_calibration_rids[] = {9,12,16,17,18,19,1,30};
    assert(!memcmp(motor_bench_calibration_registers, expected_calibration_rids,
                   sizeof(expected_calibration_rids)));
    memset(&calibration, 0xff, sizeof(calibration));
    motor_bench_init_calibration(&calibration);
    assert(calibration.version == 1U && !calibration.complete && !calibration.read_mask);
    for (unsigned i = 0; i < MOTOR_BENCH_CALIBRATION_REGISTER_COUNT; ++i)
        assert(calibration.errors[i] == -6 && calibration.raw[i] == 0U);
    calls = 0;
    motor_bench_collect_calibration(&calibration, false, NULL, NULL);
    assert(calibration.complete && !calibration.read_mask && !calls);
    for (unsigned i = 0; i < MOTOR_BENCH_CALIBRATION_REGISTER_COUNT; ++i)
        assert(calibration.errors[i] == -6);
    bad_rid = -1;
    motor_bench_collect_calibration(&calibration, true, read_calibration, NULL);
    assert(calibration.complete && calibration.read_mask == 0xffU && calls == 8U);
    assert(!memcmp(calibration.raw, calibration_values, sizeof(calibration_values)));
    assert(calibration.raw[0] == UINT32_MAX && calibration.raw[2] == 14U &&
           calibration.raw[6] == 0U);
    for (unsigned i = 0; i < MOTOR_BENCH_CALIBRATION_REGISTER_COUNT; ++i) {
        bad_rid = expected_calibration_rids[i]; calls = 0;
        motor_bench_collect_calibration(&calibration, true, read_calibration, NULL);
        assert(calls == 8U && calibration.read_mask == (0xffU & ~(1U << i)));
        assert(calibration.errors[i] == -3 && calibration.raw[i] == 0U);
    }
    bad_rid = -1;
    const uint32_t nonfinite[] = {0x7fc00001U, 0x7f800000U, 0xff800000U};
    for (unsigned i = 0; i < sizeof(nonfinite) / sizeof(*nonfinite); ++i) {
        calls = 0; calibration_values[1] = nonfinite[i];
        motor_bench_collect_calibration(&calibration, true, read_calibration, NULL);
        assert(calibration.read_mask == 0xffU && calibration.errors[1] == 0);
        assert(calibration.raw[1] == nonfinite[i]);
    }
    puts("joint7 bench diagnostic admission tests passed");
}
