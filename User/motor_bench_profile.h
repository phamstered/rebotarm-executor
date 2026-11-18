#ifndef MOTOR_BENCH_PROFILE_H
#define MOTOR_BENCH_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "motor_bench.h"

#if MOTOR_BENCH_JOINT7_POSVEL
#define MOTOR_BENCH_PROFILE_REQUEST_MAGIC 0x4a375131U /* J7Q1 */
#define MOTOR_BENCH_PROFILE_STATUS_MAGIC 0x4a375031U /* J7P1 */
#define MOTOR_BENCH_PROFILE_VERSION 1U
#define MOTOR_BENCH_PROFILE_REQUEST_WORDS 14U
#define MOTOR_BENCH_PROFILE_STATUS_WORDS 32U
#define MOTOR_BENCH_PROFILE_ACC_ORIGINAL 0x40000000U
#define MOTOR_BENCH_PROFILE_DEC_ORIGINAL 0xc0000000U
#define MOTOR_BENCH_PROFILE_ACC_TARGET 0x3951b717U
#define MOTOR_BENCH_PROFILE_PHASE_MS 100U
#define MOTOR_BENCH_PROFILE_OPERATION_MS 1200U
#define MOTOR_BENCH_PROFILE_RECOVERY_MS 600U

enum { MBP_APPLY = 1, MBP_RESTORE = 2 };
enum { MBP_IDLE, MBP_PENDING, MBP_APPLIED, MBP_RESTORED,
       MBP_ROLLED_BACK, MBP_FAILED, MBP_RESTORE_REQUIRED };
enum { MBP_OK, MBP_INVALID, MBP_IDENTITY, MBP_STALE, MBP_BUSY,
       MBP_UNSAFE, MBP_BASELINE, MBP_CAN, MBP_TIMEOUT, MBP_READBACK,
       MBP_NO_ORIGINAL, MBP_CANCELLED };
enum { MBP_PHASE_IDLE, MBP_READ_ACC, MBP_WAIT_ACC, MBP_READ_DEC,
       MBP_WAIT_DEC, MBP_WRITE_ACC, MBP_WAIT_WRITE, MBP_SETTLE,
       MBP_VERIFY_ACC, MBP_WAIT_VERIFY_ACC, MBP_VERIFY_DEC, MBP_WAIT_VERIFY_DEC };

typedef struct {
    uint64_t boot_id, source_id;
    uint32_t operation_id, action, profile_id, expected_control_epoch;
    uint32_t disable_sequence, expected_profile_generation, valid_until_ms;
} MotorBenchProfileRequest;

typedef struct {
    uint64_t boot_id, source_id;
    uint32_t phase, result, error, generation, operation_id, action, profile_id;
    uint32_t busy, dirty, original_valid, original_acc, original_dec;
    uint32_t current_acc, current_dec, read_mask, applied_verified, restore_verified;
    uint32_t started_ms, phase_ms, completed_ms;
    uint32_t last_reject_operation, last_reject_error, write_attempts, rollback_attempts;
} MotorBenchProfileStatus;

typedef struct {
    uint64_t boot_id;
    uint32_t control_epoch, disable_sequence;
    bool initialized, disabled_verified, owner_free, lease_free, not_switching;
    bool fault_free, zero_idle;
} MotorBenchProfileEnvironment;

typedef struct {
    MotorBenchProfileStatus status;
    MotorBenchProfileRequest last_request;
    bool have_request, recovering;
    uint32_t recovery_started_ms;
} MotorBenchProfile;

/* Return 0 for enqueue, 1 for FIFO backpressure, -1 for a genuine CAN error. */
typedef int (*MotorBenchProfileSend)(uint8_t rid, bool write, uint32_t value, void *context);

static inline bool motor_bench_profile_decode_request(const uint32_t *w,
    size_t count, MotorBenchProfileRequest *r)
{
    if (!w || !r || count != MOTOR_BENCH_PROFILE_REQUEST_WORDS ||
        w[0] != MOTOR_BENCH_PROFILE_REQUEST_MAGIC || w[1] != 1U || w[2] != count)
        return false;
    *r = (MotorBenchProfileRequest){
        .boot_id = (uint64_t)w[3] | ((uint64_t)w[4] << 32),
        .source_id = (uint64_t)w[5] | ((uint64_t)w[6] << 32),
        .operation_id = w[7], .action = w[8], .profile_id = w[9],
        .expected_control_epoch = w[10], .disable_sequence = w[11],
        .expected_profile_generation = w[12], .valid_until_ms = w[13]
    };
    return true;
}

/* Status words: header 0..5 = magic/version/length/boot low/high/MCU tick;
 * 6 phase,7 result,8 error,9 generation,10/11 source low/high,12 operation,
 * 13 action,14 profile,15 busy,16 dirty,17 original_valid,18/19 original bits,
 * 20/21 latest ACC/DEC bits,22 read mask,23 applied_verified,24 restore_verified,
 * 25 started,26 phase start,27 completed,28/29 last rejected op/error,
 * 30 write attempts,31 rollback attempts. No field implies motor qualification. */
static inline size_t motor_bench_profile_encode_status(uint32_t *w, size_t capacity,
    const MotorBenchProfileStatus *s, uint32_t now)
{
    if (!w || !s || capacity < MOTOR_BENCH_PROFILE_STATUS_WORDS) return 0U;
    const uint32_t words[MOTOR_BENCH_PROFILE_STATUS_WORDS] = {
        MOTOR_BENCH_PROFILE_STATUS_MAGIC,1U,MOTOR_BENCH_PROFILE_STATUS_WORDS,
        (uint32_t)s->boot_id,(uint32_t)(s->boot_id>>32),now,
        s->phase,s->result,s->error,s->generation,(uint32_t)s->source_id,
        (uint32_t)(s->source_id>>32),s->operation_id,s->action,s->profile_id,
        s->busy,s->dirty,s->original_valid,s->original_acc,s->original_dec,
        s->current_acc,s->current_dec,s->read_mask,s->applied_verified,s->restore_verified,
        s->started_ms,s->phase_ms,s->completed_ms,s->last_reject_operation,
        s->last_reject_error,s->write_attempts,s->rollback_attempts
    };
    memcpy(w, words, sizeof(words));
    return MOTOR_BENCH_PROFILE_STATUS_WORDS;
}

static inline void motor_bench_profile_init(MotorBenchProfile *p, uint64_t boot)
{
    memset(p, 0, sizeof(*p));
    p->status.boot_id = boot;
}

static inline bool motor_bench_profile_inhibits(const MotorBenchProfile *p)
{
    return p->status.busy || (p->status.dirty && !p->status.applied_verified);
}

static inline bool motor_bench_profile_ready(const MotorBenchProfile *p)
{
    return !p->status.busy && p->status.result == MBP_APPLIED && p->status.applied_verified;
}

static inline bool motor_bench_profile_safe(const MotorBenchProfileEnvironment *e)
{
    return e->initialized && e->disabled_verified && e->lease_free &&
        e->not_switching && e->zero_idle;
}

static inline bool motor_bench_profile_same(const MotorBenchProfileRequest *a,
                                           const MotorBenchProfileRequest *b)
{
    return a->boot_id==b->boot_id && a->source_id==b->source_id &&
        a->operation_id==b->operation_id && a->action==b->action &&
        a->profile_id==b->profile_id && a->expected_control_epoch==b->expected_control_epoch &&
        a->disable_sequence==b->disable_sequence &&
        a->expected_profile_generation==b->expected_profile_generation &&
        a->valid_until_ms==b->valid_until_ms;
}

static inline void motor_bench_profile_finish(MotorBenchProfile *p,
                                             uint32_t result, uint32_t now)
{
    p->status.busy = 0U;
    p->status.phase = MBP_PHASE_IDLE;
    p->status.result = result;
    p->status.completed_ms = now;
    if (result == MBP_RESTORED || result == MBP_ROLLED_BACK) {
        p->status.dirty = 0U;
        p->status.restore_verified = 1U;
    }
    p->recovering = false;
}

static inline void motor_bench_profile_fail(MotorBenchProfile *p,
                                           uint32_t error, uint32_t now)
{
    if (!p->status.error) p->status.error = error;
    p->status.applied_verified = 0U;
    if (p->status.dirty && p->status.original_valid && !p->recovering &&
        p->status.action == MBP_APPLY) {
        p->recovering = true;
        ++p->status.rollback_attempts;
        p->recovery_started_ms = now;
        p->status.phase = MBP_READ_ACC;
        p->status.phase_ms = now;
        p->status.read_mask = 0U;
    } else motor_bench_profile_finish(p,
        p->status.dirty ? MBP_RESTORE_REQUIRED : MBP_FAILED, now);
}

static inline bool motor_bench_profile_request(MotorBenchProfile *p,
    const MotorBenchProfileRequest *r, const MotorBenchProfileEnvironment *e, uint32_t now)
{
    uint32_t error = MBP_OK;
    if (!r->operation_id || !r->source_id || r->profile_id != 1U ||
        (r->action != MBP_APPLY && r->action != MBP_RESTORE)) error = MBP_INVALID;
    else if (!e->boot_id || r->boot_id != e->boot_id || r->boot_id != p->status.boot_id)
        error = MBP_IDENTITY;
    else if (p->have_request && r->source_id == p->last_request.source_id &&
             r->operation_id == p->last_request.operation_id) {
        if (motor_bench_profile_same(r, &p->last_request)) return true;
        error = MBP_INVALID;
    } else if (p->status.source_id && r->source_id != p->status.source_id) error = MBP_IDENTITY;
    else if (p->have_request && (int32_t)(r->operation_id-p->last_request.operation_id) <= 0)
        error = MBP_STALE;
    else if (p->status.busy) error = MBP_BUSY;
    else if (r->expected_control_epoch != e->control_epoch ||
        r->expected_profile_generation != p->status.generation ||
        r->disable_sequence != e->disable_sequence) error = MBP_IDENTITY;
    else if ((int32_t)(r->valid_until_ms-now) <= 0 ||
             (uint32_t)(r->valid_until_ms-now) > 1000U) error = MBP_STALE;
    else if (!motor_bench_profile_safe(e)) error = MBP_UNSAFE;
    else if (r->action == MBP_APPLY && (!e->owner_free || !e->fault_free)) error = MBP_UNSAFE;
    else if (r->action == MBP_APPLY && p->status.original_valid) error = MBP_BUSY;
    else if (r->action == MBP_RESTORE && !p->status.original_valid) error = MBP_NO_ORIGINAL;
    else if (p->status.generation == UINT32_MAX) error = MBP_STALE;
    if (error) {
        p->status.last_reject_operation = r->operation_id;
        p->status.last_reject_error = error;
        return false;
    }
    p->last_request = *r;
    p->have_request = true;
    p->recovering = false;
    p->status.source_id = r->source_id;
    p->status.operation_id = r->operation_id;
    p->status.action = r->action;
    p->status.profile_id = 1U;
    ++p->status.generation;
    p->status.busy = 1U;
    p->status.phase = MBP_READ_ACC;
    p->status.result = MBP_PENDING;
    p->status.error = MBP_OK;
    p->status.applied_verified = p->status.restore_verified = 0U;
    p->status.read_mask = 0U;
    p->status.started_ms = p->status.phase_ms = now;
    p->status.completed_ms = 0U;
    p->status.last_reject_operation = p->status.last_reject_error = 0U;
    return true;
}

static inline bool motor_bench_profile_restoring(const MotorBenchProfile *p)
{
    return p->recovering || p->status.action == MBP_RESTORE;
}

static inline void motor_bench_profile_cancel(MotorBenchProfile *p, uint32_t now)
{
    /* Repeated DISABLE cannot restart/starve an already-running rollback. */
    if (p->status.busy && !motor_bench_profile_restoring(p))
        motor_bench_profile_fail(p, MBP_CANCELLED, now);
}

/* Call even when the control loop must return early for STOP/CAN failures.
 * Pure bookkeeping only: it cannot enqueue or delay a CAN frame. */
static inline bool motor_bench_profile_deadline(MotorBenchProfile *p, uint32_t now)
{
    if (!p->status.busy) return false;
    uint32_t start = p->recovering ? p->recovery_started_ms : p->status.started_ms;
    uint32_t deadline = p->recovering ? MOTOR_BENCH_PROFILE_RECOVERY_MS : MOTOR_BENCH_PROFILE_OPERATION_MS;
    if ((uint32_t)(now-start) < deadline) return false;
    motor_bench_profile_fail(p, MBP_TIMEOUT, now);
    return true;
}

static inline void motor_bench_profile_step(MotorBenchProfile *p,
    const MotorBenchProfileEnvironment *e, uint32_t now, MotorBenchProfileSend send, void *ctx)
{
    if (!p->status.busy) return;
    if (motor_bench_profile_deadline(p, now)) return;
    if (!motor_bench_profile_safe(e)) {
        /* Wait only while attempting recovery of a potentially modified drive.
         * The overall deadline still ends in RESTORE_REQUIRED, never motion. */
        if (!motor_bench_profile_restoring(p)) motor_bench_profile_fail(p, MBP_UNSAFE, now);
        return;
    }
    if (e->boot_id != p->status.boot_id) {
        motor_bench_profile_finish(p, p->status.dirty ? MBP_RESTORE_REQUIRED : MBP_FAILED, now);
        p->status.error = MBP_IDENTITY;
        p->status.applied_verified = 0U;
        return;
    }
    if (!motor_bench_profile_restoring(p) && (!e->fault_free || !e->owner_free)) {
        motor_bench_profile_fail(p, MBP_UNSAFE, now); return;
    }
    uint32_t phase = p->status.phase;
    if (phase == MBP_SETTLE) {
        if ((uint32_t)(now-p->status.phase_ms) < 20U) return;
        p->status.phase = phase = MBP_VERIFY_ACC;
        p->status.phase_ms = now;
        p->status.read_mask = 0U;
    }
    if (phase == MBP_WAIT_WRITE &&
        (uint32_t)(now-p->status.phase_ms) >= MOTOR_BENCH_PROFILE_PHASE_MS) {
        /* Missing ACK is not proof of failure. Query both registers after a
         * settle interval; only independent readback can commit/restore. */
        p->status.phase = MBP_SETTLE;
        p->status.phase_ms = now;
        return;
    }
    if ((uint32_t)(now-p->status.phase_ms) >= MOTOR_BENCH_PROFILE_PHASE_MS) {
        motor_bench_profile_fail(p, MBP_TIMEOUT, now); return;
    }
    bool write = phase == MBP_WRITE_ACC;
    if (!write && phase != MBP_READ_ACC && phase != MBP_READ_DEC &&
        phase != MBP_VERIFY_ACC && phase != MBP_VERIFY_DEC) return;
    uint8_t rid = phase == MBP_READ_DEC || phase == MBP_VERIFY_DEC ? 5U : 4U;
    uint32_t value = motor_bench_profile_restoring(p) ? p->status.original_acc : MOTOR_BENCH_PROFILE_ACC_TARGET;
    int result = send(rid, write, value, ctx);
    if (result > 0) return; /* Backpressure is not a HAL write failure. */
    if (write) { p->status.dirty = 1U; ++p->status.write_attempts; }
    if (result < 0) { motor_bench_profile_fail(p, MBP_CAN, now); return; }
    p->status.phase = phase + 1U;
    p->status.phase_ms = now;
}

static inline void motor_bench_profile_reply(MotorBenchProfile *p, uint16_t can_id,
    const uint8_t data[8], uint32_t received_ms, uint32_t now)
{
    if (!p->status.busy || can_id != 0x17U || data[0] != 7U || data[1] != 0U ||
        (int32_t)(received_ms-p->status.phase_ms) < 0 ||
        (int32_t)(received_ms-now) > 0 ||
        (uint32_t)(now-p->status.phase_ms) >= MOTOR_BENCH_PROFILE_PHASE_MS) return;
    uint32_t phase = p->status.phase;
    bool write = phase == MBP_WAIT_WRITE;
    bool acc = phase == MBP_WAIT_ACC || phase == MBP_WAIT_VERIFY_ACC || write;
    if (!acc && phase != MBP_WAIT_DEC && phase != MBP_WAIT_VERIFY_DEC) return;
    if (data[2] != (write ? 0x55U : 0x33U) || data[3] != (acc ? 4U : 5U)) return;
    uint32_t value; memcpy(&value, data+4, sizeof(value));
    uint32_t target = motor_bench_profile_restoring(p) ? p->status.original_acc : MOTOR_BENCH_PROFILE_ACC_TARGET;
    if (write) {
        if (value != target) { motor_bench_profile_fail(p, MBP_READBACK, now); return; }
        p->status.phase = MBP_SETTLE;
    } else if (phase == MBP_WAIT_ACC) {
        p->status.current_acc = value; p->status.read_mask |= 1U;
        p->status.phase = MBP_READ_DEC;
    } else if (phase == MBP_WAIT_DEC) {
        p->status.current_dec = value; p->status.read_mask |= 2U;
        if (!motor_bench_profile_restoring(p) &&
            (value != MOTOR_BENCH_PROFILE_DEC_ORIGINAL ||
             p->status.current_acc != MOTOR_BENCH_PROFILE_ACC_ORIGINAL)) {
            motor_bench_profile_fail(p, MBP_BASELINE, now); return;
        }
        /* On recovery an externally changed DEC cannot be repaired here. Still
         * restore the known ACC; final independent DEC verification will fail
         * closed without ever sending a DEC write. */
        if (!motor_bench_profile_restoring(p)) {
            p->status.original_acc = p->status.current_acc;
            p->status.original_dec = value;
            p->status.original_valid = 1U;
        }
        p->status.phase = MBP_WRITE_ACC;
    } else if (phase == MBP_WAIT_VERIFY_ACC) {
        p->status.current_acc = value; p->status.read_mask |= 1U;
        if (value != target) { motor_bench_profile_fail(p, MBP_READBACK, now); return; }
        p->status.phase = MBP_VERIFY_DEC;
    } else {
        p->status.current_dec = value; p->status.read_mask |= 2U;
        if (value != p->status.original_dec) { motor_bench_profile_fail(p, MBP_READBACK, now); return; }
        if (motor_bench_profile_restoring(p)) {
            bool rollback = p->recovering;
            motor_bench_profile_finish(p, rollback ? MBP_ROLLED_BACK : MBP_RESTORED, now);
        } else {
            p->status.applied_verified = 1U;
            motor_bench_profile_finish(p, MBP_APPLIED, now);
        }
    }
    p->status.phase_ms = now;
}
#endif
#endif
