#include "motor_control_v2.h"
#include <math.h>
#include <string.h>

enum { POSITION_SEND, POSITION_WAIT, WRITE_SEND, WRITE_WAIT, SETTLE, READ_SEND, READ_WAIT };
static bool same_request(const MotorControlRequest *a, const MotorControlRequest *b)
{
    return a->action == b->action && a->operation_id == b->operation_id &&
        a->boot_id == b->boot_id && a->expected_control_epoch == b->expected_control_epoch &&
        a->source_id == b->source_id && a->active_joint_mask == b->active_joint_mask &&
        a->control_mode == b->control_mode && a->lease_id == b->lease_id &&
        a->disable_sequence == b->disable_sequence;
}
static void finish(MotorControlV2 *s, uint8_t result, uint16_t error)
{
    s->switching = false;
    s->operation_state = result;
    s->error_code = error;
    s->control_epoch++;
    /* Epoch exhaustion requires a fresh boot; never recycle an old grant. */
    if (s->control_epoch == 0U) s->boot_id = 0U;
    s->lease_id = 0U;
}
void motor_v2_init(MotorControlV2 *s, uint64_t boot_id)
{
    memset(s, 0, sizeof(*s));
    s->boot_id = boot_id;
    s->control_epoch = 1U;
}
bool motor_v2_mit_allowed(const MotorControlV2 *s)
{
    if (s->switching || s->owner_source_id || s->verified_joint_mask != MV2_ALL) return false;
    for (uint8_t i = 0; i < MV2_JOINTS; ++i) if (s->control_modes[i] != 1U) return false;
    return true;
}
MotorControlResponse motor_v2_request(MotorControlV2 *s, const MotorControlRequest *r,
    bool disabled_proven, uint16_t fault, uint32_t last_sequence, uint32_t now)
{
    MotorControlResponse response = {false, MV2_OK, s->control_epoch, s->lease_id, r->operation_id};
    if (s->have_request && r->source_id == s->last_request.source_id &&
        r->operation_id == s->last_request.operation_id) {
        if (same_request(r, &s->last_request)) {
            response = s->last_response;
            if (r->action == 2U && (!s->lease_id || response.lease_id != s->lease_id)) {
                response.accepted = false; response.error_code = MV2_LEASE;
            }
            return response;
        }
        response.error_code = MV2_INVALID;
        return response;
    }
    /* A timed-out/released lease cannot be recreated by replaying an older
     * service request after a newer operation has replaced the cache. */
    if (s->have_request && r->source_id == s->last_request.source_id &&
        (int32_t)(r->operation_id - s->last_request.operation_id) <= 0) {
        response.error_code = MV2_SEQUENCE; return response;
    }
    uint32_t mask = r->active_joint_mask;
    if (!r->operation_id || !r->source_id || r->action < 1 || r->action > 4 ||
        mask == 0 || (mask & ~MV2_ALL)) response.error_code = MV2_INVALID;
    else if (!s->boot_id || r->boot_id != s->boot_id ||
        r->expected_control_epoch != s->control_epoch) response.error_code = MV2_IDENTITY;
    else if (s->owner_source_id && s->owner_source_id != r->source_id &&
        !(r->action == 1U && disabled_proven && !s->lease_id && !s->switching &&
          last_sequence == r->disable_sequence)) response.error_code = MV2_BUSY;
    else if (r->action == 4U) {
        motor_v2_disable(s);
        response.accepted = true;
    } else if (s->switching || (r->action == 1U && s->lease_id)) response.error_code = MV2_BUSY;
    else if (!disabled_proven || last_sequence != r->disable_sequence) response.error_code = MV2_NOT_DISABLED;
    else if (fault && r->action != 3U) response.error_code = MV2_FAULT;
    else if (r->action == 1U) {
        if (r->control_mode != 1U && r->control_mode != 2U) response.error_code = MV2_MODE;
        else if (r->control_mode == 2U && (mask & s->pos_vel_supported_mask) != mask) response.error_code = MV2_UNSUPPORTED;
        else {
            s->owner_source_id = r->source_id;
            s->selected_joint_mask = mask;
            s->config_mask = mask;
            s->target_mode = r->control_mode;
            s->lease_id = 0;
            s->switching = true;
            s->operation_id = r->operation_id;
            s->operation_state = MV2_PENDING;
            s->error_code = MV2_OK;
            s->axis = 0;
            while (!(mask & (1U << s->axis))) ++s->axis;
            s->phase = s->target_mode == 2U ? POSITION_SEND : WRITE_SEND;
            s->phase_at = now;
            response.accepted = true;
        }
    } else if (r->action == 2U) {
        if (r->control_mode != 2U || (mask & s->pos_vel_supported_mask) != mask) response.error_code = MV2_UNSUPPORTED;
        else if ((mask & s->verified_joint_mask) != mask) response.error_code = MV2_MODE;
        else if (s->lease_id) response.error_code = MV2_BUSY;
        else {
            for (uint8_t i = 0; i < MV2_JOINTS; ++i)
                if ((mask & (1U << i)) && s->control_modes[i] != 2U) response.error_code = MV2_MODE;
            if (response.error_code == MV2_OK) {
                if (++s->next_lease == 0U) { s->boot_id = 0; response.error_code = MV2_IDENTITY; }
                else {
                    s->owner_source_id = r->source_id;
                    s->selected_joint_mask = mask;
                    s->lease_id = s->next_lease;
                    s->last_frame_ms = now;
                    s->lease_granted_at = now;
                    s->lease_has_frame = false;
                    response.accepted = true;
                }
            }
        }
    } else {
        bool mit = s->verified_joint_mask == MV2_ALL;
        for (uint8_t i = 0; i < MV2_JOINTS; ++i) mit = mit && s->control_modes[i] == 1U;
        if (!mit) response.error_code = MV2_MODE;
        else { s->owner_source_id = 0; s->lease_id = 0; s->selected_joint_mask = 0; response.accepted = true; }
    }
    response.control_epoch = s->control_epoch;
    response.lease_id = s->lease_id;
    if (response.accepted) {
        if (r->action != 1U) {
            s->operation_id = r->operation_id;
            s->operation_state = r->action == 4U ? MV2_CANCELLED : MV2_SUCCESS;
            s->error_code = MV2_OK;
        }
        s->last_request = *r;
        s->last_response = response;
        s->have_request = true;
    }
    return response;
}
uint16_t motor_v2_command(MotorControlV2 *s, const MotorCommandV2 *c,
    uint16_t fault, bool have_sequence, uint32_t last_sequence, uint32_t now)
{
    uint16_t error = MV2_OK;
    if (!s->boot_id || c->boot_id != s->boot_id || c->control_epoch != s->control_epoch) error = MV2_IDENTITY;
    else if (s->switching) error = MV2_BUSY;
    else if (fault) error = MV2_FAULT;
    else if (c->control_mode != 2U || c->mode > 1U) error = MV2_MODE;
    else if (!s->lease_id || c->lease_id != s->lease_id || c->source_id != s->owner_source_id ||
        (uint32_t)(now - s->last_frame_ms) > (s->lease_has_frame ? 100U : 500U)) error = MV2_LEASE;
    else if (c->active_joint_mask != s->selected_joint_mask) error = MV2_INVALID;
    else if (have_sequence && (int32_t)(c->sequence - last_sequence) <= 0) error = MV2_SEQUENCE;
    else for (uint8_t i = 0; i < MV2_JOINTS; ++i) {
        bool selected = (c->active_joint_mask & (1U << i)) != 0;
        if (!isfinite(c->position[i]) || !isfinite(c->velocity_limit[i])) { error = MV2_NONFINITE; break; }
        if ((!selected && (c->position[i] != 0 || c->velocity_limit[i] != 0)) ||
            (c->mode == 0U && c->position[i] != 0)) { error = MV2_INVALID; break; }
        if (selected && (!(s->verified_joint_mask & (1U << i)) || s->control_modes[i] != 2U)) { error = MV2_MODE; break; }
        if (selected && (!(s->pos_vel_supported_mask & (1U << i)) ||
            c->velocity_limit[i] <= 0 || c->velocity_limit[i] > s->velocity_limit_max[i] ||
            (c->mode == 1U && (c->position[i] < s->position_min[i] || c->position[i] > s->position_max[i])))) { error = MV2_LIMITS; break; }
    }
    if (error) { s->rejected_sequence = c->sequence; s->reject_code = error; }
    else { s->last_frame_ms = now; s->lease_has_frame = true; s->reject_code = MV2_OK; }
    return error;
}
void motor_v2_disable(MotorControlV2 *s)
{
    s->lease_id = 0;
    /* Preserve the last operation tombstone: replay cannot issue another grant. */
    if (s->switching) finish(s, MV2_CANCELLED, MV2_OK);
}
void motor_v2_fault(MotorControlV2 *s) { motor_v2_disable(s); }
void motor_v2_watchdog(MotorControlV2 *s, uint32_t now)
{
    if (s->lease_id && (uint32_t)(now - s->last_frame_ms) > (s->lease_has_frame ? 100U : 500U)) {
        motor_v2_disable(s);
    }
}
void motor_v2_step(MotorControlV2 *s, bool disabled_proven, uint32_t now,
    MotorRegisterSend send, void *context)
{
    if (!s->switching) return;
    if (!disabled_proven) { finish(s, MV2_FAILURE, MV2_NOT_DISABLED); return; }
    if ((s->phase == POSITION_WAIT || s->phase == WRITE_WAIT || s->phase == READ_WAIT ||
         (s->target_mode == 2U && s->phase == WRITE_SEND)) &&
        (uint32_t)(now - s->phase_at) > MV2_REGISTER_TIMEOUT_MS) {
        finish(s, MV2_FAILURE, MV2_TIMEOUT); return;
    }
    if (s->phase == SETTLE && (uint32_t)(now - s->phase_at) >= 20U) s->phase = READ_SEND;
    if (s->phase == POSITION_SEND || s->phase == WRITE_SEND || s->phase == READ_SEND) {
        bool position = s->phase == POSITION_SEND;
        bool write = s->phase == WRITE_SEND;
        /* The old value is no longer verified once a write may reach the bus. */
        if (write) { s->verified_joint_mask &= ~(1U << s->axis); s->control_modes[s->axis] = 0; }
        s->phase = position ? POSITION_WAIT : write ? WRITE_WAIT : READ_WAIT;
        s->phase_at = now;
        if (!send(s->axis, position ? MV2_REG_PRECISE_POSITION : MV2_REG_MODE,
                  write, s->target_mode, context)) finish(s, MV2_FAILURE, MV2_READBACK);
    }
}
void motor_v2_register_reply(MotorControlV2 *s, uint8_t axis, uint8_t rid, bool write,
    uint32_t value, uint32_t received_ms, uint32_t now)
{
    if (!s->switching || axis != s->axis ||
        (int32_t)(received_ms - s->phase_at) < 0 ||
        (uint32_t)(now - received_ms) > MV2_REGISTER_TIMEOUT_MS) return;
    if ((uint32_t)(now - s->phase_at) > MV2_REGISTER_TIMEOUT_MS) {
        finish(s, MV2_FAILURE, MV2_TIMEOUT); return;
    }
    if (s->phase == POSITION_WAIT) {
        if (write || rid != MV2_REG_PRECISE_POSITION) return;
        float position; memcpy(&position, &value, sizeof(position));
        if (!isfinite(position)) { finish(s, MV2_FAILURE, MV2_NONFINITE); return; }
        if (!isfinite(s->position_min[axis]) || !isfinite(s->position_max[axis]) ||
            s->position_min[axis] > s->position_max[axis] ||
            position < s->position_min[axis] || position > s->position_max[axis]) {
            finish(s, MV2_FAILURE, MV2_LIMITS); return;
        }
        s->phase = WRITE_SEND; s->phase_at = now;
        return;
    }
    if (rid != MV2_REG_MODE) return;
    if (write && s->phase == WRITE_WAIT) {
        if (value != s->target_mode) { finish(s, MV2_FAILURE, MV2_READBACK); return; }
        s->phase = SETTLE; s->phase_at = now;
    } else if (!write && s->phase == READ_WAIT) {
        if (value != s->target_mode) { finish(s, MV2_FAILURE, MV2_READBACK); return; }
        s->control_modes[axis] = (uint8_t)value;
        s->verified_joint_mask |= 1U << axis;
        do { ++s->axis; } while (s->axis < MV2_JOINTS && !(s->config_mask & (1U << s->axis)));
        if (s->axis == MV2_JOINTS) finish(s, MV2_SUCCESS, MV2_OK);
        else { s->phase = s->target_mode == 2U ? POSITION_SEND : WRITE_SEND; s->phase_at = now; }
    }
}
