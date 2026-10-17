#include "motor_bridge.h"

#include "FreeRTOS.h"
#include "bsp_fdcan.h"
#include "damiao_protocol.h"
#include "main.h"
#include "task.h"

#include <math.h>
#include <string.h>

#define COMMAND_TIMEOUT_MS 100U
#define COMMAND_LOSS_LATCH_MS 500U
#define FEEDBACK_TIMEOUT_MS 100U
#define ENABLE_RETRY_MS 20U
#define ENABLE_TIMEOUT_MS 150U
#define DISABLED_FEEDBACK_REQUEST_MS 50U
#define ZERO_REGISTER_TIMEOUT_MS 100U
#define ZERO_MAX_VELOCITY_RAD_S 0.10f
#define ZERO_MAX_EFFORT_NM 0.50f
#define ZERO_VERIFY_POSITION_RAD 0.05f

/* Feed-forward effort retains an independent application-level safety limit. */
#define COMMAND_EFFORT_LIMIT_NM 10.0f

extern FDCAN_HandleTypeDef hfdcan1;

static const uint16_t motor_ids[MOTOR_BRIDGE_JOINT_COUNT] = {
    0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U
};

static const uint16_t feedback_ids[MOTOR_BRIDGE_JOINT_COUNT] = {
    0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U
};

/* MIT fields are normalized integers on the wire.  These ranges must match
 * the model-specific ranges used by the motor firmware; they are not command
 * safety limits.  COMMAND_EFFORT_LIMIT_NM above remains the independent
 * application-level safety policy. */
static const DamiaoLimits motor_limits[MOTOR_BRIDGE_JOINT_COUNT] = {{
    /* joint1: DM4340P */
    .p_min = -12.5f, .p_max = 12.5f,
    .v_min = -10.0f, .v_max = 10.0f,
    .t_min = -28.0f, .t_max = 28.0f
}, {
    /* joint2: DM4340P */
    .p_min = -12.5f, .p_max = 12.5f, .v_min = -10.0f, .v_max = 10.0f,
    .t_min = -28.0f, .t_max = 28.0f
}, {
    /* joint3: DM4340P */
    .p_min = -12.5f, .p_max = 12.5f, .v_min = -10.0f, .v_max = 10.0f,
    .t_min = -28.0f, .t_max = 28.0f
}, {
    /* joint4: DM4310 */
    .p_min = -12.5f, .p_max = 12.5f, .v_min = -30.0f, .v_max = 30.0f,
    .t_min = -10.0f, .t_max = 10.0f
}, {
    /* joint5: DM4310 */
    .p_min = -12.5f, .p_max = 12.5f, .v_min = -30.0f, .v_max = 30.0f,
    .t_min = -10.0f, .t_max = 10.0f
}, {
    /* joint6: DM4310 */
    .p_min = -12.5f, .p_max = 12.5f, .v_min = -30.0f, .v_max = 30.0f,
    .t_min = -10.0f, .t_max = 10.0f
}, {
    /* gripper: DM4310 */
    .p_min = -12.5f, .p_max = 12.5f, .v_min = -30.0f, .v_max = 30.0f,
    .t_min = -10.0f, .t_max = 10.0f
}};

typedef struct {
    MotorBridgeCommand command;
    MotorBridgeSnapshot snapshot;
    uint32_t last_command_ms;
    bool have_command;
    bool motor_enabled[MOTOR_BRIDGE_JOINT_COUNT];
    bool enable_pending[MOTOR_BRIDGE_JOINT_COUNT];
    bool communication_loss_latched;
    uint32_t enable_started_ms[MOTOR_BRIDGE_JOINT_COUNT];
    uint32_t last_enable_ms[MOTOR_BRIDGE_JOINT_COUNT];
    uint32_t last_feedback_request_ms[MOTOR_BRIDGE_JOINT_COUNT];
    bool hold_latched;
    float hold_position[MOTOR_BRIDGE_JOINT_COUNT];
    uint8_t drive_status[MOTOR_BRIDGE_JOINT_COUNT];
    uint32_t feedback_valid_mask;
    MotorBridgeZeroSnapshot zero_snapshot;
    uint8_t zero_pending_operation;
    uint32_t zero_pending_mask;
    bool initialized;
} MotorBridgeState;

static MotorBridgeState state;
static DamiaoMotorState motor_state[MOTOR_BRIDGE_JOINT_COUNT];

static bool sequence_is_newer(uint32_t candidate, uint32_t current)
{
    return (int32_t)(candidate - current) > 0;
}

static bool any_motor_enabled(void)
{
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (state.motor_enabled[i]) return true;
    }
    return false;
}

static uint16_t validate_command(const MotorBridgeCommand *command)
{
    if (command->mode > MOTOR_MODE_DISABLE) {
        return MOTOR_FAULT_INVALID_MODE;
    }
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (!isfinite(command->position[i]) || !isfinite(command->velocity[i]) ||
            !isfinite(command->kp[i]) || !isfinite(command->kd[i]) ||
            !isfinite(command->effort[i])) {
            return MOTOR_FAULT_NONFINITE_COMMAND;
        }
    }
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (command->kp[i] < 0.0f || command->kd[i] < 0.0f ||
            fabsf(command->effort[i]) > COMMAND_EFFORT_LIMIT_NM) {
            return MOTOR_FAULT_COMMAND_LIMIT;
        }
    }
    return MOTOR_FAULT_NONE;
}

static bool send_frame(uint16_t can_id, uint8_t *data)
{
    return fdcanx_send_data(&hfdcan1, can_id, data, DAMIAO_DLC) == 0U;
}

static int protocol_send(uint16_t can_id, const uint8_t *data, void *ctx)
{
    (void)ctx;
    return send_frame(can_id, (uint8_t *)data) ? 0 : -1;
}

static int protocol_recv(uint16_t *can_id, uint8_t *data,
                         uint32_t timeout_ms, void *ctx)
{
    (void)ctx;
    uint32_t start = HAL_GetTick();
    for (;;) {
        if (fdcan1_get_rx_data(can_id, data) == DAMIAO_DLC) {
            return 0;
        }
        uint32_t elapsed = HAL_GetTick() - start;
        if (elapsed >= timeout_ms) {
            return -1;
        }
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms - elapsed)) == 0U) {
            return -1;
        }
    }
}

static void protocol_sleep(uint32_t ms, void *ctx)
{
    (void)ctx;
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void send_disable(void)
{
    uint8_t data[DAMIAO_DLC];
    damiao_encode_disable_cmd(data);
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (!send_frame(damiao_get_mit_can_id(motor_ids[i]), data)) {
            state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
        }
        state.motor_enabled[i] = false;
        state.enable_pending[i] = false;
    }
}

static void set_mode(uint8_t mode)
{
    if (state.snapshot.mode != mode) {
        state.snapshot.mode = mode;
        state.hold_latched = false;
    }
}

static DamiaoModeSwitchCtx make_mode_context(uint32_t index)
{
    DamiaoModeSwitchCtx context = {
        .motor_id = motor_ids[index],
        .send = protocol_send,
        .recv = protocol_recv,
        .sleep = protocol_sleep,
        .user_ctx = NULL,
        .limits = &motor_limits[index]
    };
    return context;
}

static bool read_output_offset(uint32_t index, float *offset)
{
    uint32_t raw = 0U;
    DamiaoModeSwitchCtx context = make_mode_context(index);
    if (damiao_read_register_u32(&context, DAMIAO_REG_OUTPUT_POS_OFFSET,
                                 &raw, ZERO_REGISTER_TIMEOUT_MS) !=
        DAMIAO_ERR_OK) {
        return false;
    }
    memcpy(offset, &raw, sizeof(*offset));
    return isfinite(*offset);
}

static void finish_zero_request(uint32_t success_mask, uint32_t failure_mask,
                                uint8_t result_code)
{
    taskENTER_CRITICAL();
    uint32_t completed_ms = HAL_GetTick();
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if ((state.feedback_valid_mask & (1UL << i)) != 0U) {
            /* Maintenance owns the RX queue while it runs. Give the normal
             * disabled-feedback poll one watchdog interval to refresh it. */
            state.snapshot.last_feedback_ms[i] = completed_ms;
        }
    }
    state.zero_snapshot.success_mask = success_mask;
    state.zero_snapshot.failure_mask = failure_mask;
    state.zero_snapshot.result_code = result_code;
    state.zero_snapshot.busy = false;
    state.zero_pending_operation = MOTOR_ZERO_OPERATION_NONE;
    state.zero_pending_mask = 0U;
    if (result_code == MOTOR_ZERO_RESULT_OK) {
        state.snapshot.fault_code = MOTOR_FAULT_NONE;
    }
    taskEXIT_CRITICAL();
}

static void process_zero_request(void)
{
    uint8_t operation;
    uint32_t requested_mask;
    taskENTER_CRITICAL();
    operation = state.zero_pending_operation;
    requested_mask = state.zero_pending_mask;
    taskEXIT_CRITICAL();
    if (operation == MOTOR_ZERO_OPERATION_NONE) return;

    if (state.snapshot.mode != MOTOR_MODE_DISABLE || any_motor_enabled()) {
        finish_zero_request(0U, requested_mask,
                            MOTOR_ZERO_RESULT_UNSAFE_MODE);
        return;
    }
    if (operation == MOTOR_ZERO_OPERATION_SET) {
        for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
            if ((requested_mask & (1UL << i)) != 0U &&
                (((state.feedback_valid_mask & (1UL << i)) == 0U) ||
                 fabsf(state.snapshot.velocity[i]) > ZERO_MAX_VELOCITY_RAD_S ||
                 fabsf(state.snapshot.effort[i]) > ZERO_MAX_EFFORT_NM)) {
                finish_zero_request(0U, requested_mask,
                                    MOTOR_ZERO_RESULT_UNSAFE_MOTION);
                return;
            }
        }
    }

    uint32_t success_mask = 0U;
    uint32_t failure_mask = 0U;
    uint8_t result_code = MOTOR_ZERO_RESULT_OK;
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        uint32_t bit = 1UL << i;
        if ((requested_mask & bit) == 0U) continue;

        bool ok = true;
        if (operation == MOTOR_ZERO_OPERATION_SET) {
            ok = damiao_motor_disable(&motor_state[i], protocol_send, NULL) ==
                     DAMIAO_ERR_OK;
            if (ok) protocol_sleep(20U, NULL);
            ok = ok &&
                 damiao_motor_set_zero_position(&motor_state[i], protocol_send,
                                                protocol_sleep, NULL) ==
                     DAMIAO_ERR_OK;
            if (ok) {
                DamiaoModeSwitchCtx context = make_mode_context(i);
                ok = damiao_motor_request_feedback_state(&motor_state[i],
                        &context, ZERO_REGISTER_TIMEOUT_MS) == DAMIAO_ERR_OK;
                if (ok) {
                    state.snapshot.position[i] =
                        motor_state[i].last_feedback.pos;
                    state.snapshot.velocity[i] =
                        motor_state[i].last_feedback.vel;
                    state.snapshot.effort[i] =
                        motor_state[i].last_feedback.torq;
                    state.snapshot.last_feedback_ms[i] = HAL_GetTick();
                    ok = fabsf(state.snapshot.position[i]) <=
                        ZERO_VERIFY_POSITION_RAD;
                    if (!ok) result_code = MOTOR_ZERO_RESULT_VERIFY;
                }
            }
        }
        if (ok) ok = read_output_offset(i,
                                        &state.zero_snapshot.output_offset[i]);
        if (ok) success_mask |= bit;
        else {
            failure_mask |= bit;
            if (result_code == MOTOR_ZERO_RESULT_OK) {
                result_code = MOTOR_ZERO_RESULT_IO;
            }
        }
    }
    finish_zero_request(success_mask, failure_mask, result_code);
}

void motor_bridge_init(void)
{
    memset(&state, 0, sizeof(state));
    state.snapshot.mode = MOTOR_MODE_DISABLE;
    state.snapshot.active_joint_mask = MOTOR_BRIDGE_ACTIVE_JOINT_MASK;
    send_disable();

    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        damiao_motor_state_init(&motor_state[i], motor_ids[i], feedback_ids[i],
                                &motor_limits[i]);
        DamiaoModeSwitchCtx mode_ctx = make_mode_context(i);
        if (damiao_motor_request_feedback_state(&motor_state[i], &mode_ctx,
                                                300U) != DAMIAO_ERR_OK ||
            damiao_ensure_control_mode(&mode_ctx, DAMIAO_MODE_MIT, 300U) !=
                DAMIAO_ERR_OK) {
            state.snapshot.fault_code = MOTOR_FAULT_INIT;
            send_disable();
            return;
        }
        state.snapshot.position[i] = motor_state[i].last_feedback.pos;
        state.snapshot.velocity[i] = motor_state[i].last_feedback.vel;
        state.snapshot.effort[i] = motor_state[i].last_feedback.torq;
        state.snapshot.last_feedback_ms[i] = HAL_GetTick();
        state.feedback_valid_mask |= (1UL << i);
        state.drive_status[i] = motor_state[i].last_feedback.status_code;
        state.motor_enabled[i] =
            motor_state[i].last_feedback.status_code == DAMIAO_STATUS_ENABLED;
        state.last_feedback_request_ms[i] = HAL_GetTick();
    }
    taskENTER_CRITICAL();
    uint32_t ready_ms = HAL_GetTick();
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        state.snapshot.last_feedback_ms[i] = ready_ms;
    }
    state.snapshot.feedback_valid = true;
    state.snapshot.fault_code = MOTOR_FAULT_NONE;
    state.initialized = true;
    taskEXIT_CRITICAL();
}

bool motor_bridge_accept_command(const MotorBridgeCommand *command, uint32_t now_ms)
{
    uint16_t fault = validate_command(command);
    taskENTER_CRITICAL();
    if (!state.initialized) {
        fault = MOTOR_FAULT_INIT;
    } else if (state.zero_snapshot.busy) {
        fault = MOTOR_FAULT_ZERO_POSITION_BUSY;
    } else if (!state.communication_loss_latched && state.have_command &&
        state.snapshot.mode != MOTOR_MODE_DISABLE &&
        (uint32_t)(now_ms - state.last_command_ms) >
            COMMAND_LOSS_LATCH_MS) {
        state.communication_loss_latched = true;
        state.snapshot.fault_code = MOTOR_FAULT_COMMAND_LOSS_LATCHED;
        set_mode(any_motor_enabled() ? MOTOR_MODE_HOLD : MOTOR_MODE_DISABLE);
    }
    if (state.communication_loss_latched) {
        fault = MOTOR_FAULT_COMMAND_LOSS_LATCHED;
    } else if (fault == MOTOR_FAULT_NONE && state.have_command &&
        !sequence_is_newer(command->sequence, state.command.sequence)) {
        fault = MOTOR_FAULT_STALE_SEQUENCE;
    }
    if (fault != MOTOR_FAULT_NONE) {
        state.snapshot.fault_code = fault;
        taskEXIT_CRITICAL();
        return false;
    }

    state.command = *command;
    state.have_command = true;
    state.last_command_ms = now_ms;
    state.snapshot.last_command_sequence = command->sequence;
    state.snapshot.fault_code = MOTOR_FAULT_NONE;
    set_mode(command->mode);
    taskEXIT_CRITICAL();
    return true;
}

static void consume_feedback(uint32_t now_ms)
{
    uint16_t can_id;
    uint8_t data[DAMIAO_DLC];
    while (fdcan1_get_rx_data(&can_id, data) == DAMIAO_DLC) {
        for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
            if (can_id == feedback_ids[i]) {
                DamiaoFeedback feedback;
                damiao_decode_sensor_feedback(data, &motor_limits[i], &feedback);
                state.snapshot.position[i] = feedback.pos;
                state.snapshot.velocity[i] = feedback.vel;
                state.snapshot.effort[i] = feedback.torq;
                state.snapshot.last_feedback_ms[i] = now_ms;
                bool finite = isfinite(feedback.pos) &&
                    isfinite(feedback.vel) && isfinite(feedback.torq);
                if (finite) state.feedback_valid_mask |= (1UL << i);
                else state.feedback_valid_mask &= ~(1UL << i);
                state.drive_status[i] = feedback.status_code;
                state.motor_enabled[i] =
                    feedback.status_code == DAMIAO_STATUS_ENABLED;
                if (!finite) {
                    state.snapshot.fault_code = MOTOR_FAULT_FEEDBACK_TIMEOUT;
                    set_mode(MOTOR_MODE_DISABLE);
                } else if (feedback.status_code >= DAMIAO_STATUS_OVER_VOLTAGE) {
                    state.snapshot.fault_code = MOTOR_FAULT_DRIVE;
                    set_mode(MOTOR_MODE_DISABLE);
                }
                break;
            }
        }
    }
    state.snapshot.feedback_valid =
        (state.feedback_valid_mask & MOTOR_BRIDGE_ACTIVE_JOINT_MASK) ==
            MOTOR_BRIDGE_ACTIVE_JOINT_MASK;
}

void motor_bridge_control_step(uint32_t now_ms)
{
    uint8_t data[DAMIAO_DLC];
    consume_feedback(now_ms);
    if (state.zero_snapshot.busy) {
        process_zero_request();
        return;
    }

    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        /* A disabled Damiao drive does not stream feedback autonomously. */
        if (!state.motor_enabled[i] &&
            (uint32_t)(now_ms - state.last_feedback_request_ms[i]) >=
                DISABLED_FEEDBACK_REQUEST_MS) {
            damiao_encode_feedback_request_cmd(motor_ids[i], data);
            if (!send_frame(damiao_get_register_can_id(), data)) {
                state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
                set_mode(MOTOR_MODE_DISABLE);
                return;
            }
            state.last_feedback_request_ms[i] = now_ms;
        }
    }

    if (state.snapshot.mode == MOTOR_MODE_DISABLE || !state.have_command) {
        for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
            if (state.motor_enabled[i]) {
                damiao_encode_disable_cmd(data);
                if (!send_frame(damiao_get_mit_can_id(motor_ids[i]), data)) {
                    state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
                }
                state.motor_enabled[i] = false;
            }
            state.enable_pending[i] = false;
        }
        return;
    }

    if (!state.snapshot.feedback_valid) {
        state.snapshot.fault_code = MOTOR_FAULT_FEEDBACK_TIMEOUT;
        set_mode(MOTOR_MODE_DISABLE);
        send_disable();
        return;
    }

    bool all_enabled = true;
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (!state.motor_enabled[i]) {
            all_enabled = false;
            if (!state.enable_pending[i]) {
                state.enable_pending[i] = true;
                state.enable_started_ms[i] = now_ms;
                state.last_enable_ms[i] = now_ms - ENABLE_RETRY_MS;
            } else if ((uint32_t)(now_ms - state.enable_started_ms[i]) >
                       ENABLE_TIMEOUT_MS) {
                state.snapshot.fault_code = MOTOR_FAULT_ENABLE_TIMEOUT;
                set_mode(MOTOR_MODE_DISABLE);
                send_disable();
                return;
            }
            if ((uint32_t)(now_ms - state.last_enable_ms[i]) >=
                    ENABLE_RETRY_MS) {
                damiao_encode_enable_cmd(data);
                if (!send_frame(damiao_get_mit_can_id(motor_ids[i]), data)) {
                    state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
                    set_mode(MOTOR_MODE_DISABLE);
                    send_disable();
                    return;
                }
                state.last_enable_ms[i] = now_ms;
            }
        } else {
            state.enable_pending[i] = false;
        }
    }
    if (!all_enabled) {
        return;
    }

    if (state.snapshot.mode == MOTOR_MODE_HOLD) {
        if (!state.hold_latched) {
            for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
                state.hold_position[i] = state.snapshot.position[i];
            }
            state.hold_latched = true;
        }
    }

    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (state.snapshot.mode == MOTOR_MODE_HOLD) {
            damiao_encode_mit_cmd(state.hold_position[i], 0.0f, 0.0f,
                                  state.command.kp[i], state.command.kd[i],
                                  &motor_limits[i], data);
        } else {
            damiao_encode_mit_cmd(state.command.position[i],
                                  state.command.velocity[i],
                                  state.command.effort[i], state.command.kp[i],
                                  state.command.kd[i], &motor_limits[i], data);
        }
        if (!send_frame(damiao_get_mit_can_id(motor_ids[i]), data)) {
            state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
            set_mode(MOTOR_MODE_DISABLE);
            send_disable();
            return;
        }
    }
}

void motor_bridge_safety_step(uint32_t now_ms)
{
    taskENTER_CRITICAL();
    if (!state.initialized) {
        taskEXIT_CRITICAL();
        return;
    }
    if (state.zero_snapshot.busy) {
        taskEXIT_CRITICAL();
        return;
    }
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if ((state.feedback_valid_mask & (1UL << i)) != 0U &&
            (uint32_t)(now_ms - state.snapshot.last_feedback_ms[i]) >
                FEEDBACK_TIMEOUT_MS) {
            state.feedback_valid_mask &= ~(1UL << i);
            state.snapshot.feedback_valid = false;
            state.snapshot.fault_code = MOTOR_FAULT_FEEDBACK_TIMEOUT;
            set_mode(MOTOR_MODE_DISABLE);
        }
    }
    if (state.have_command) {
        uint32_t age = now_ms - state.last_command_ms;
        if (state.snapshot.mode == MOTOR_MODE_TRACK && age > COMMAND_TIMEOUT_MS) {
            set_mode(MOTOR_MODE_HOLD);
        }
        if (state.snapshot.mode != MOTOR_MODE_DISABLE &&
            age > COMMAND_LOSS_LATCH_MS) {
            state.communication_loss_latched = true;
            state.snapshot.fault_code = MOTOR_FAULT_COMMAND_LOSS_LATCHED;
            set_mode(any_motor_enabled() ? MOTOR_MODE_HOLD : MOTOR_MODE_DISABLE);
        }
    }
    taskEXIT_CRITICAL();
}

void motor_bridge_get_snapshot(MotorBridgeSnapshot *snapshot)
{
    taskENTER_CRITICAL();
    *snapshot = state.snapshot;
    taskEXIT_CRITICAL();
}

bool motor_bridge_request_zero_position(uint8_t operation, uint32_t joint_mask)
{
    joint_mask &= MOTOR_BRIDGE_ACTIVE_JOINT_MASK;
    taskENTER_CRITICAL();
    if (state.zero_pending_operation != MOTOR_ZERO_OPERATION_NONE ||
        state.zero_snapshot.busy) {
        taskEXIT_CRITICAL();
        return false;
    }
    state.zero_snapshot.sequence++;
    state.zero_snapshot.operation = operation;
    state.zero_snapshot.requested_mask = joint_mask;
    state.zero_snapshot.success_mask = 0U;
    state.zero_snapshot.failure_mask = 0U;
    if (!state.initialized) {
        state.zero_snapshot.result_code = MOTOR_ZERO_RESULT_NOT_READY;
        state.zero_snapshot.failure_mask = joint_mask;
        state.zero_snapshot.busy = false;
        taskEXIT_CRITICAL();
        return false;
    }
    if (joint_mask == 0U ||
        (operation != MOTOR_ZERO_OPERATION_GET &&
         operation != MOTOR_ZERO_OPERATION_SET)) {
        state.zero_snapshot.result_code = MOTOR_ZERO_RESULT_INVALID_MASK;
        state.zero_snapshot.busy = false;
        taskEXIT_CRITICAL();
        return false;
    }
    state.zero_pending_operation = operation;
    state.zero_pending_mask = joint_mask;
    state.zero_snapshot.result_code = MOTOR_ZERO_RESULT_NONE;
    state.zero_snapshot.busy = true;
    taskEXIT_CRITICAL();
    return true;
}

void motor_bridge_get_zero_snapshot(MotorBridgeZeroSnapshot *snapshot)
{
    taskENTER_CRITICAL();
    *snapshot = state.zero_snapshot;
    taskEXIT_CRITICAL();
}

void motor_bridge_force_disable(uint16_t fault_code)
{
    taskENTER_CRITICAL();
    state.snapshot.fault_code = fault_code;
    set_mode(MOTOR_MODE_DISABLE);
    taskEXIT_CRITICAL();
}
