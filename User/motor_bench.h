#ifndef MOTOR_BENCH_H
#define MOTOR_BENCH_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#ifndef MOTOR_BENCH_JOINT7_POSVEL
#define MOTOR_BENCH_JOINT7_POSVEL 0
#endif

#ifndef MOTOR_BENCH_OVERSPEED_STOP
#define MOTOR_BENCH_OVERSPEED_STOP 1
#endif

#ifndef MOTOR_BENCH_EFFORT_STOP
#define MOTOR_BENCH_EFFORT_STOP 1
#endif

#if MOTOR_BENCH_JOINT7_POSVEL
#if !defined(MOTOR_BENCH_POSITION_MIN) || !defined(MOTOR_BENCH_POSITION_MAX) || !defined(MOTOR_BENCH_VELOCITY_MAX)
#error "Bench image requires explicit joint7 position and speed bounds"
#endif

/* SWD-readable, startup-only diagnostics. This is not a qualification record.
 * raw[] order: hw_ver, sw_ver, SN, sub_ver, Damp, ACC, DEC, MAX_SPD,
 * PMAX, VMAX, TMAX, Deta. Versions/serial are u32; remaining values are float bits.
 * Damp is physical viscosity; Deta is the speed-loop damping factor. */
#define MOTOR_BENCH_REGISTER_COUNT 12U
static const uint8_t motor_bench_registers[MOTOR_BENCH_REGISTER_COUNT] =
    {13, 14, 15, 36, 11, 4, 5, 6, 21, 22, 23, 31};
typedef struct {
    uint32_t magic, complete, read_mask, admitted_mask;
    float position_min, position_max, velocity_max, startup_position;
    uint32_t raw[MOTOR_BENCH_REGISTER_COUNT];
    int32_t errors[MOTOR_BENCH_REGISTER_COUNT];
    uint32_t trip_reason, trip_ms;
    float trip_position, trip_velocity, trip_effort;
} MotorBenchDiagnostics;
typedef int (*MotorBenchRead)(uint8_t rid, uint32_t *value, void *context);

/* Separate from J7B3: these are observations, not added admission criteria. */
#define MOTOR_BENCH_LOOP_REGISTER_COUNT 9U
static const uint8_t motor_bench_loop_registers[MOTOR_BENCH_LOOP_REGISTER_COUNT] =
    {20, 24, 25, 26, 27, 28, 32, 33, 34};
typedef struct {
    uint32_t version, complete, read_mask, started_ms, completed_ms;
    uint32_t raw[MOTOR_BENCH_LOOP_REGISTER_COUNT];
    int32_t errors[MOTOR_BENCH_LOOP_REGISTER_COUNT];
} MotorBenchLoopDiagnostics;

/* Separate startup-only observations. TIMEOUT/NPP are u32; the other raw
 * values are float bits. No value or read error changes bench admission. */
#define MOTOR_BENCH_CALIBRATION_REGISTER_COUNT 8U
static const uint8_t motor_bench_calibration_registers[MOTOR_BENCH_CALIBRATION_REGISTER_COUNT] =
    {9, 12, 16, 17, 18, 19, 1, 30};
typedef struct {
    uint32_t version, complete, read_mask, started_ms, completed_ms;
    uint32_t raw[MOTOR_BENCH_CALIBRATION_REGISTER_COUNT];
    int32_t errors[MOTOR_BENCH_CALIBRATION_REGISTER_COUNT];
} MotorBenchCalibrationDiagnostics;

static inline void motor_bench_init_calibration(MotorBenchCalibrationDiagnostics *d)
{
    memset(d, 0, sizeof(*d));
    d->version = 1U;
    for (uint32_t i = 0U; i < MOTOR_BENCH_CALIBRATION_REGISTER_COUNT; ++i)
        d->errors[i] = -6; /* DAMIAO_ERR_NOT_DISABLED; no request was sent. */
}

static inline void motor_bench_collect_calibration(MotorBenchCalibrationDiagnostics *d,
    bool disabled, MotorBenchRead read_register, void *context)
{
    motor_bench_init_calibration(d);
    for (uint32_t i = 0U; i < MOTOR_BENCH_CALIBRATION_REGISTER_COUNT; ++i) {
        if (!disabled) continue;
        d->errors[i] = read_register(motor_bench_calibration_registers[i], &d->raw[i], context);
        if (d->errors[i] == 0) d->read_mask |= 1U << i;
    }
    d->complete = 1U;
}

static inline void motor_bench_collect_loop(MotorBenchLoopDiagnostics *d,
    bool disabled, MotorBenchRead read_register, void *context)
{
    memset(d, 0, sizeof(*d));
    d->version = 1U;
    for (uint32_t i = 0U; i < MOTOR_BENCH_LOOP_REGISTER_COUNT; ++i) {
        /* Not-read is distinct from a successful zero-valued register. */
        d->errors[i] = -6; /* DAMIAO_ERR_NOT_DISABLED; no request was sent. */
        if (!disabled) continue;
        d->errors[i] = read_register(motor_bench_loop_registers[i], &d->raw[i], context);
        if (d->errors[i] == 0) d->read_mask |= 1U << i;
    }
    d->complete = 1U;
}

/* Feedback stop thresholds, not a drive-internal current/torque limiter.
 * The configured position interval has no extra overshoot allowance. */
static inline uint32_t motor_bench_feedback_reason(bool valid, uint32_t age_ms,
    float position, float velocity, float effort)
{
    if (!valid || age_ms > 100U) return 1U;
    if (!isfinite(position) || !isfinite(velocity) || !isfinite(effort)) return 2U;
    if (position < MOTOR_BENCH_POSITION_MIN ||
        position > MOTOR_BENCH_POSITION_MAX) return 3U;
    if (MOTOR_BENCH_OVERSPEED_STOP && fabsf(velocity) > 0.1f) return 4U;
    if (MOTOR_BENCH_EFFORT_STOP && fabsf(effort) > 0.30f) return 5U;
    return 0U;
}

static inline float motor_bench_float(uint32_t raw)
{
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static inline bool motor_bench_collect(MotorBenchDiagnostics *d, bool disabled,
    float position, MotorBenchRead read_register, void *context)
{
    memset(d, 0, sizeof(*d));
    d->magic = 0x4a374233U; /* J7B3: appended Deta; dump layout changed. */
    d->position_min = MOTOR_BENCH_POSITION_MIN;
    d->position_max = MOTOR_BENCH_POSITION_MAX;
    d->velocity_max = MOTOR_BENCH_VELOCITY_MAX;
    d->startup_position = position;
    bool valid = disabled && isfinite(position) &&
        isfinite(d->position_min) && isfinite(d->position_max) &&
        isfinite(d->velocity_max) && d->position_min < d->position_max &&
        position >= d->position_min && position <= d->position_max &&
        d->velocity_max > 0.0f && d->velocity_max <= 0.05f;
    /* Never issue diagnostic requests if startup feedback reported an enabled drive. */
    if (!disabled) { d->complete = 1; return false; }
    for (uint32_t i = 0; i < MOTOR_BENCH_REGISTER_COUNT; ++i) {
        d->errors[i] = read_register(motor_bench_registers[i], &d->raw[i], context);
        if (d->errors[i] == 0) d->read_mask |= 1U << i;
        else valid = false;
    }
    for (uint32_t i = 4; i <= 7; ++i) {
        float value = motor_bench_float(d->raw[i]);
        /* DEC is signed deceleration, unlike positive ACC/Damp/MAX_SPD. */
        valid = valid && isfinite(value) && (i == 6 ? value < 0.0f : value > 0.0f);
    }
    /* Startup must observe the exact baseline ACC=2 / DEC=-2 float bits.
     * A drive's temporary RAM profile can survive an MCU-only reset. Do not
     * silently admit it as a fresh baseline or overwrite it during collection. */
    valid = valid && d->raw[5] == 0x40000000U && d->raw[6] == 0xc0000000U;
    /* MAX_SPD is a pre-gearbox motor parameter, not the output-shaft command cap. */
    float damping_factor = motor_bench_float(d->raw[11]);
    valid = valid && isfinite(damping_factor) &&
        damping_factor >= 1.0f && damping_factor <= 30.0f &&
        motor_bench_float(d->raw[8]) == 12.5f &&
        motor_bench_float(d->raw[9]) == 30.0f &&
        motor_bench_float(d->raw[10]) == 10.0f;
    d->admitted_mask = valid ? 0x40U : 0U;
    d->complete = 1;
    return valid;
}
#endif
#endif
