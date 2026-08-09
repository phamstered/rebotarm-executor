#include "motor_bridge.h"

#include "FreeRTOS.h"
#include "bsp_fdcan.h"
#include "damiao_protocol.h"
#include "main.h"
#include "task.h"

#include <math.h>
#include <string.h>

#define BASE_MOTOR_ID 1U
#define BASE_FEEDBACK_ID (BASE_MOTOR_ID + 0x10U)
#define COMMAND_TIMEOUT_MS 100U
#define DISABLE_TIMEOUT_MS 500U
#define FEEDBACK_TIMEOUT_MS 100U
#define CONTROL_PERIOD_MS 2U
#define ENABLE_RETRY_MS 20U
#define ENABLE_TIMEOUT_MS 150U
#define DISABLED_FEEDBACK_REQUEST_MS 50U

/* Conservative bring-up limits. Increase only after single-motor evidence. */
#define COMMAND_POSITION_LIMIT_RAD 0.35f
#define COMMAND_VELOCITY_LIMIT_RAD_S 0.5f
#define COMMAND_KP_LIMIT 40.0f
#define COMMAND_KD_LIMIT 5.0f
#define COMMAND_EFFORT_LIMIT_NM 1.0f

extern FDCAN_HandleTypeDef hfdcan1;

static const DamiaoLimits motor_limits = {
    .p_min = -12.5f, .p_max = 12.5f,
    .v_min = -30.0f, .v_max = 30.0f,
    .t_min = -5.0f, .t_max = 5.0f
};

typedef struct {
    MotorBridgeCommand command;
    MotorBridgeSnapshot snapshot;
    uint32_t last_command_ms;
    bool have_command;
    bool motor_enabled;
    bool enable_pending;
    uint32_t enable_started_ms;
    uint32_t last_enable_ms;
    uint32_t last_feedback_request_ms;
    bool hold_latched;
    float hold_position;
    uint8_t drive_status;
} MotorBridgeState;

static MotorBridgeState state;
static DamiaoMotorState motor_state;

static bool sequence_is_newer(uint32_t candidate, uint32_t current)
{
    return (int32_t)(candidate - current) > 0;
}

static bool inactive_values_are_zero(const MotorBridgeCommand *command)
{
    for (uint32_t i = 1U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (command->position[i] != 0.0f || command->velocity[i] != 0.0f ||
            command->kp[i] != 0.0f || command->kd[i] != 0.0f ||
            command->effort[i] != 0.0f) {
            return false;
        }
    }
    return true;
}

static uint16_t validate_command(const MotorBridgeCommand *command)
{
    if (command->mode > MOTOR_MODE_DISABLE) {
        return MOTOR_FAULT_INVALID_MODE;
    }
    if (!inactive_values_are_zero(command)) {
        return MOTOR_FAULT_INACTIVE_JOINT_COMMAND;
    }
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (!isfinite(command->position[i]) || !isfinite(command->velocity[i]) ||
            !isfinite(command->kp[i]) || !isfinite(command->kd[i]) ||
            !isfinite(command->effort[i])) {
            return MOTOR_FAULT_NONFINITE_COMMAND;
        }
    }
    if (fabsf(command->position[0]) > COMMAND_POSITION_LIMIT_RAD ||
        fabsf(command->velocity[0]) > COMMAND_VELOCITY_LIMIT_RAD_S ||
        command->kp[0] < 0.0f || command->kp[0] > COMMAND_KP_LIMIT ||
        command->kd[0] < 0.0f || command->kd[0] > COMMAND_KD_LIMIT ||
        fabsf(command->effort[0]) > COMMAND_EFFORT_LIMIT_NM) {
        return MOTOR_FAULT_COMMAND_LIMIT;
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
    if (!send_frame(damiao_get_mit_can_id(BASE_MOTOR_ID), data)) {
        state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
    }
    state.motor_enabled = false;
}

static void set_mode(uint8_t mode)
{
    if (state.snapshot.mode != mode) {
        state.snapshot.mode = mode;
        state.hold_latched = false;
    }
}

void motor_bridge_init(void)
{
    memset(&state, 0, sizeof(state));
    state.snapshot.mode = MOTOR_MODE_DISABLE;
    state.snapshot.active_joint_mask = MOTOR_BRIDGE_ACTIVE_JOINT_MASK;
    send_disable();

    damiao_motor_state_init(&motor_state, BASE_MOTOR_ID, BASE_FEEDBACK_ID,
                            &motor_limits);
    DamiaoModeSwitchCtx mode_ctx = {
        .motor_id = BASE_MOTOR_ID,
        .send = protocol_send,
        .recv = protocol_recv,
        .sleep = protocol_sleep,
        .user_ctx = NULL,
        .limits = &motor_limits
    };
    if (damiao_motor_request_feedback_state(&motor_state, &mode_ctx, 300U) !=
            DAMIAO_ERR_OK ||
        damiao_ensure_control_mode(&mode_ctx, DAMIAO_MODE_MIT, 300U) !=
            DAMIAO_ERR_OK) {
        state.snapshot.fault_code = MOTOR_FAULT_INIT;
        send_disable();
        return;
    }
    state.snapshot.position = motor_state.last_feedback.pos;
    state.snapshot.velocity = motor_state.last_feedback.vel;
    state.snapshot.effort = motor_state.last_feedback.torq;
    state.snapshot.last_feedback_ms = HAL_GetTick();
    state.snapshot.feedback_valid = true;
    state.drive_status = motor_state.last_feedback.status_code;
    state.last_feedback_request_ms = HAL_GetTick();
}

bool motor_bridge_accept_command(const MotorBridgeCommand *command, uint32_t now_ms)
{
    uint16_t fault = validate_command(command);
    taskENTER_CRITICAL();
    if (fault == MOTOR_FAULT_NONE && state.have_command &&
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
        if (can_id == BASE_FEEDBACK_ID) {
            DamiaoFeedback feedback;
            damiao_decode_sensor_feedback(data, &motor_limits, &feedback);
            state.snapshot.position = feedback.pos;
            state.snapshot.velocity = feedback.vel;
            state.snapshot.effort = feedback.torq;
            state.snapshot.last_feedback_ms = now_ms;
            state.snapshot.feedback_valid = isfinite(feedback.pos) &&
                isfinite(feedback.vel) && isfinite(feedback.torq);
            state.drive_status = feedback.status_code;
            state.motor_enabled = feedback.status_code == DAMIAO_STATUS_ENABLED;
            if (feedback.status_code >= DAMIAO_STATUS_OVER_VOLTAGE) {
                state.snapshot.fault_code = MOTOR_FAULT_DRIVE;
                set_mode(MOTOR_MODE_DISABLE);
            }
        }
    }
}

void motor_bridge_control_step(uint32_t now_ms)
{
    uint8_t data[DAMIAO_DLC];
    consume_feedback(now_ms);

    /* A disabled Damiao drive does not stream feedback autonomously. Poll it
     * so a later ROS command can be safely armed from a fresh snapshot. */
    if (!state.motor_enabled &&
        (uint32_t)(now_ms - state.last_feedback_request_ms) >=
            DISABLED_FEEDBACK_REQUEST_MS) {
        damiao_encode_feedback_request_cmd(BASE_MOTOR_ID, data);
        if (!send_frame(damiao_get_register_can_id(), data)) {
            state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
            set_mode(MOTOR_MODE_DISABLE);
            return;
        }
        state.last_feedback_request_ms = now_ms;
    }

    if (state.snapshot.mode == MOTOR_MODE_DISABLE || !state.have_command) {
        if (state.motor_enabled) {
            send_disable();
        }
        state.enable_pending = false;
        return;
    }

    if (!state.snapshot.feedback_valid) {
        if ((uint32_t)(now_ms - state.last_command_ms) > FEEDBACK_TIMEOUT_MS) {
            state.snapshot.fault_code = MOTOR_FAULT_FEEDBACK_TIMEOUT;
            set_mode(MOTOR_MODE_DISABLE);
            send_disable();
        }
        return;
    }

    if (!state.motor_enabled) {
        if (!state.enable_pending) {
            state.enable_pending = true;
            state.enable_started_ms = now_ms;
            state.last_enable_ms = now_ms - ENABLE_RETRY_MS;
        } else if ((uint32_t)(now_ms - state.enable_started_ms) >
                   ENABLE_TIMEOUT_MS) {
            state.snapshot.fault_code = MOTOR_FAULT_ENABLE_TIMEOUT;
            set_mode(MOTOR_MODE_DISABLE);
            send_disable();
            return;
        }
        if ((uint32_t)(now_ms - state.last_enable_ms) >= ENABLE_RETRY_MS) {
            damiao_encode_enable_cmd(data);
            if (!send_frame(damiao_get_mit_can_id(BASE_MOTOR_ID), data)) {
                state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
                set_mode(MOTOR_MODE_DISABLE);
                return;
            }
            state.last_enable_ms = now_ms;
        }
        return;
    }
    state.enable_pending = false;

    if (state.snapshot.mode == MOTOR_MODE_HOLD) {
        if (!state.hold_latched) {
            state.hold_position = state.snapshot.position;
            state.hold_latched = true;
        }
        damiao_encode_mit_cmd(state.hold_position, 0.0f, 0.0f,
                              state.command.kp[0], state.command.kd[0],
                              &motor_limits, data);
    } else {
        damiao_encode_mit_cmd(state.command.position[0], state.command.velocity[0],
                              state.command.effort[0], state.command.kp[0],
                              state.command.kd[0], &motor_limits, data);
    }
    if (!send_frame(damiao_get_mit_can_id(BASE_MOTOR_ID), data)) {
        state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
        set_mode(MOTOR_MODE_DISABLE);
        send_disable();
    }
}

void motor_bridge_safety_step(uint32_t now_ms)
{
    taskENTER_CRITICAL();
    if (state.snapshot.feedback_valid &&
        (uint32_t)(now_ms - state.snapshot.last_feedback_ms) > FEEDBACK_TIMEOUT_MS) {
        state.snapshot.feedback_valid = false;
        state.snapshot.fault_code = MOTOR_FAULT_FEEDBACK_TIMEOUT;
        set_mode(MOTOR_MODE_DISABLE);
    }
    if (state.have_command) {
        uint32_t age = now_ms - state.last_command_ms;
        if (state.snapshot.mode == MOTOR_MODE_TRACK && age > COMMAND_TIMEOUT_MS) {
            set_mode(MOTOR_MODE_HOLD);
        }
        if (age > DISABLE_TIMEOUT_MS) {
            set_mode(MOTOR_MODE_DISABLE);
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

void motor_bridge_force_disable(uint16_t fault_code)
{
    taskENTER_CRITICAL();
    state.snapshot.fault_code = fault_code;
    set_mode(MOTOR_MODE_DISABLE);
    taskEXIT_CRITICAL();
}
