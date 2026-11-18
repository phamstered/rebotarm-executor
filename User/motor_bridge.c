#include "motor_bridge.h"

#include "FreeRTOS.h"
#include "bsp_fdcan.h"
#include "damiao_protocol.h"
#include "motor_capabilities.h"
#include "motor_bench.h"
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
    MotorCommandV2 command_v2;
    MotorControlV2 control;
    bool using_v2;
    uint32_t disable_ms;
    uint32_t received_feedback_ms[MOTOR_BRIDGE_JOINT_COUNT];
    bool have_disable;
    bool disable_pending, disable_tx_wait;
    uint32_t disable_required_mask, disable_remaining_mask;
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
#if MOTOR_BENCH_JOINT7_POSVEL
/* Retained as a global ELF symbol for read-only SWD inspection after startup. */
volatile MotorBenchDiagnostics motor_bench_diagnostics;
volatile MotorBenchLoopDiagnostics motor_bench_loop_diagnostics;
volatile MotorBenchCalibrationDiagnostics motor_bench_calibration_diagnostics;
/* The linker places only this explicitly initialized array in RAM_D1. */
static MotorBenchTraceRecord motor_bench_trace_records[MOTOR_BENCH_TRACE_CAPACITY]
#if defined(__arm__) || defined(__thumb__)
    __attribute__((section(".motor_bench_trace"), aligned(32)))
#endif
    ;
static MotorBenchTraceMeta motor_bench_trace_meta;
static MotorBenchProfile motor_bench_profile;

static void bench_trace_can(uint32_t kind, uint16_t can_id,
                            const uint8_t *data, uint32_t received_ms)
{
    bool joint7 = kind == MOTOR_BENCH_TRACE_RX ? can_id == 0x17U :
        can_id == 7U || can_id == 0x107U ||
        (can_id == 0x7ffU && data[0] == 7U && data[1] == 0U);
    if (!joint7) return;
    MotorBenchTraceRecord record = {
        received_ms, motor_bench_trace_tag(kind, 7U, can_id, DAMIAO_DLC), 0U, 0U
    };
    memcpy(&record.data0, data, 4U);
    memcpy(&record.data1, data + 4U, 4U);
    taskENTER_CRITICAL();
    motor_bench_trace_append(&motor_bench_trace_meta, motor_bench_trace_records,
                             &record, HAL_GetTick());
    taskEXIT_CRITICAL();
}

static uint16_t bench_latched_fault(void)
{
    return motor_bench_diagnostics.trip_reason == 1U ? MOTOR_FAULT_FEEDBACK_TIMEOUT :
        motor_bench_diagnostics.trip_reason ? MOTOR_FAULT_COMMAND_LIMIT : MOTOR_FAULT_NONE;
}
#else
#define bench_latched_fault() MOTOR_FAULT_NONE
#endif

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
    bool success = fdcanx_send_data(&hfdcan1, can_id, data, DAMIAO_DLC) == 0U;
#if MOTOR_BENCH_JOINT7_POSVEL
    bench_trace_can(success ? MOTOR_BENCH_TRACE_TX_OK : MOTOR_BENCH_TRACE_TX_ERROR,
                    can_id, data, HAL_GetTick());
#endif
    return success;
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
#if MOTOR_BENCH_JOINT7_POSVEL
        uint32_t received_ms;
        if (fdcan1_get_rx_data_at(can_id, data, &received_ms) == DAMIAO_DLC) {
            bench_trace_can(MOTOR_BENCH_TRACE_RX, *can_id, data, received_ms);
#else
        if (fdcan1_get_rx_data(can_id, data) == DAMIAO_DLC) {
#endif
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

static uint16_t control_can_id(uint32_t i)
{
    return state.control.control_modes[i] == 2U ?
        damiao_get_pos_vel_can_id(motor_ids[i]) : damiao_get_mit_can_id(motor_ids[i]);
}

static bool disabled_proven(uint32_t now)
{
    if (!state.have_disable || state.disable_pending || state.disable_tx_wait || state.snapshot.mode != MOTOR_MODE_DISABLE) return false;
    for (uint32_t i = 0; i < MOTOR_BRIDGE_JOINT_COUNT; ++i) {
        if (!(state.feedback_valid_mask & (1U << i)) ||
            state.drive_status[i] != DAMIAO_STATUS_DISABLED ||
            (int32_t)(state.received_feedback_ms[i] - state.disable_ms) <= 0 ||
            (uint32_t)(now - state.received_feedback_ms[i]) > FEEDBACK_TIMEOUT_MS) return false;
    }
    return true;
}

#if MOTOR_BENCH_JOINT7_POSVEL
static MotorBenchProfileEnvironment bench_profile_environment(uint32_t now)
{
    return (MotorBenchProfileEnvironment){
        .boot_id = state.control.boot_id, .control_epoch = state.control.control_epoch,
        .disable_sequence = state.snapshot.last_command_sequence,
        .initialized = state.initialized, .disabled_verified = disabled_proven(now),
        .owner_free = state.control.owner_source_id == 0U,
        .lease_free = state.control.lease_id == 0U, .not_switching = !state.control.switching,
        .fault_free = state.snapshot.fault_code == MOTOR_FAULT_NONE &&
            !state.communication_loss_latched && !bench_latched_fault(),
        .zero_idle = !state.zero_snapshot.busy
    };
}

static int bench_profile_send(uint8_t rid, bool write, uint32_t value, void *context)
{
    (void)context;
    /* Independent allowlist: state-machine mistakes must never become arbitrary
     * writes, DEC changes, persistence, or motion commands. */
    if ((rid != 4U && rid != 5U) || (write && (rid != 4U ||
        (value != MOTOR_BENCH_PROFILE_ACC_TARGET && value != MOTOR_BENCH_PROFILE_ACC_ORIGINAL))))
        return -1;
    if (fdcan1_tx_free() == 0U) return 1;
    uint8_t data[DAMIAO_DLC];
    if (write) damiao_encode_register_write_cmd(7U, 4U, value, data);
    else damiao_encode_register_read_cmd(7U, rid, data);
    return send_frame(damiao_get_register_can_id(), data) ? 0 : -1;
}
#endif

static uint64_t boot_nonce(void)
{
    /* HSI48 is already enabled for USB. RNG failure disables only V2 admission. */
    __HAL_RCC_RNG_CONFIG(RCC_RNGCLKSOURCE_HSI48);
    __HAL_RCC_RNG_CLK_ENABLE();
    /* Follow STM32H7 RNG v3.2 conditioning reset; preserve reset tuning fields. */
    RNG->CR &= ~RNG_CR_RNGEN;
    RNG->CR = (RNG->CR & ~RNG_CR_CED) | RNG_CR_CONDRST;
    RNG->HTCR = 0x17590ABCU;
    RNG->HTCR = 0x00007274U;
    RNG->CR &= ~RNG_CR_CONDRST;
    uint32_t reset_start = HAL_GetTick();
    while (RNG->CR & RNG_CR_CONDRST)
        if (HAL_GetTick() - reset_start > 10U) return 0;
    RNG->CR |= RNG_CR_RNGEN;
    uint64_t nonce = 0;
    for (unsigned i = 0; i < 2; ++i) {
        uint32_t start = HAL_GetTick();
        while (!(RNG->SR & RNG_SR_DRDY)) {
            if ((RNG->SR & (RNG_SR_SECS | RNG_SR_CECS | RNG_SR_SEIS | RNG_SR_CEIS)) || HAL_GetTick() - start > 10U) {
                RNG->CR &= ~RNG_CR_RNGEN; return 0;
            }
        }
        if (RNG->SR & (RNG_SR_SECS | RNG_SR_CECS | RNG_SR_SEIS | RNG_SR_CEIS)) { RNG->CR &= ~RNG_CR_RNGEN; return 0; }
        nonce = (nonce << 32) | RNG->DR;
    }
    RNG->CR &= ~RNG_CR_RNGEN;
    return nonce;
}

static bool runtime_register_send(uint8_t axis, bool write, uint32_t value, void *context)
{
    (void)context;
    uint8_t data[DAMIAO_DLC];
    if (write) damiao_encode_register_write_cmd(motor_ids[axis], DAMIAO_REG_CTRL_MODE, value, data);
    else damiao_encode_register_read_cmd(motor_ids[axis], DAMIAO_REG_CTRL_MODE, data);
    return send_frame(damiao_get_register_can_id(), data);
}

static void request_disable(void)
{
    uint32_t required = 0U;
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (state.control.control_modes[i] != 2U) required |= 1U << i;
        if (state.control.control_modes[i] != 1U) required |= 1U << (i + MOTOR_BRIDGE_JOINT_COUNT);
        state.enable_pending[i] = false;
    }
    if (!state.disable_pending && !state.disable_tx_wait) {
        state.disable_required_mask = required;
        state.disable_remaining_mask = required;
    } else {
        /* Coalesce repeated stops without replaying the already queued prefix.
         * A newly unknown mode can only add targets, never remove old targets. */
        state.disable_remaining_mask |= required & ~state.disable_required_mask;
        state.disable_required_mask |= required;
    }
    if (state.disable_remaining_mask) {
        state.disable_pending = true;
        state.disable_tx_wait = false;
    }
}

static bool send_disable(void)
{
    if (!state.disable_pending) return true;
    uint8_t data[DAMIAO_DLC];
    damiao_encode_disable_cmd(data);
    for (uint32_t bit = 0; bit < 2U * MOTOR_BRIDGE_JOINT_COUNT; ++bit) {
        uint32_t mask = 1U << bit;
        if (!(state.disable_remaining_mask & mask)) continue;
        /* A full FIFO is backpressure, not a failed CAN transmission. */
        if (fdcan1_tx_free() == 0U) return false;
        uint32_t axis = bit % MOTOR_BRIDGE_JOINT_COUNT;
        uint16_t can_id = bit < MOTOR_BRIDGE_JOINT_COUNT ?
            damiao_get_mit_can_id(motor_ids[axis]) : damiao_get_pos_vel_can_id(motor_ids[axis]);
        if (!send_frame(can_id, data)) {
            state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
            return false;
        }
        state.disable_remaining_mask &= ~mask;
    }
    state.disable_pending = false;
    state.disable_tx_wait = true;
    return true;
}

static bool initialize_disabled_outputs(uint32_t timeout_ms)
{
    /* Startup only, before command admission, outside any critical section.
     * Leave runtime flushing to the nonblocking 4 ms control steps. */
    uint32_t start = HAL_GetTick();
    request_disable();
    while (state.disable_pending || state.disable_tx_wait) {
        if (state.disable_pending) (void)send_disable();
        if (state.snapshot.fault_code == MOTOR_FAULT_CAN_TX) return false;
        if (!state.disable_pending && fdcan1_tx_empty()) {
            state.disable_tx_wait = false;
            state.disable_ms = HAL_GetTick();
            return true;
        }
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms) {
            state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return true;
}

static void set_mode(uint8_t mode)
{
    if (mode == MOTOR_MODE_DISABLE) motor_v2_disable(&state.control);
    if (state.snapshot.mode != mode) {
#if MOTOR_BENCH_JOINT7_POSVEL
        if (mode == MOTOR_MODE_DISABLE)
            motor_bench_trace_trigger(&motor_bench_trace_meta, motor_bench_trace_records,
                MOTOR_BENCH_TRACE_STOP, state.snapshot.last_command_sequence,
                state.snapshot.fault_code, HAL_GetTick());
#endif
        if (mode == MOTOR_MODE_DISABLE) request_disable();
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

#if MOTOR_BENCH_JOINT7_POSVEL
static int bench_read_register(uint8_t rid, uint32_t *value, void *context)
{
    (void)context;
    uint8_t data[DAMIAO_DLC];
    damiao_encode_register_read_cmd(motor_ids[6], rid, data);
    if (!send_frame(damiao_get_register_can_id(), data)) return DAMIAO_ERR_CAN_WRITE;
    uint32_t started = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started) < 100U) {
        uint16_t can_id;
        if (protocol_recv(&can_id, data, 5U, NULL) == 0 &&
            can_id == feedback_ids[6] && damiao_is_register_reply(data) &&
            data[0] == motor_ids[6] && data[1] == 0U && data[3] == rid) {
            memcpy(value, data + 4, sizeof(*value));
            return DAMIAO_ERR_OK;
        }
    }
    return DAMIAO_ERR_TIMEOUT;
}

static void initialize_joint7_bench(void)
{
    MotorBenchDiagnostics diagnostics;
    MotorBenchLoopDiagnostics loop;
    MotorBenchCalibrationDiagnostics calibration;
    state.control.pos_vel_supported_mask = 0U;
    memset(state.control.position_min, 0, sizeof(state.control.position_min));
    memset(state.control.position_max, 0, sizeof(state.control.position_max));
    memset(state.control.velocity_limit_max, 0, sizeof(state.control.velocity_limit_max));
    bool disabled = true;
    for (uint32_t i = 0; i < MOTOR_BRIDGE_JOINT_COUNT; ++i)
        disabled = disabled && state.drive_status[i] == DAMIAO_STATUS_DISABLED;
    if (motor_bench_collect(&diagnostics, disabled, state.snapshot.position[6],
                            bench_read_register, NULL)) {
        state.control.pos_vel_supported_mask = 0x40U;
        state.control.position_min[6] = MOTOR_BENCH_POSITION_MIN;
        state.control.position_max[6] = MOTOR_BENCH_POSITION_MAX;
        state.control.velocity_limit_max[6] = MOTOR_BENCH_VELOCITY_MAX;
    }
    uint32_t started_ms = HAL_GetTick();
    motor_bench_collect_loop(&loop, disabled, bench_read_register, NULL);
    loop.started_ms = started_ms;
    loop.completed_ms = HAL_GetTick();
    started_ms = HAL_GetTick();
    motor_bench_collect_calibration(&calibration, disabled, bench_read_register, NULL);
    calibration.started_ms = started_ms;
    calibration.completed_ms = HAL_GetTick();
    taskENTER_CRITICAL();
    motor_bench_diagnostics = diagnostics;
    motor_bench_loop_diagnostics = loop;
    motor_bench_calibration_diagnostics = calibration;
    taskEXIT_CRITICAL();
}
#endif

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
#if MOTOR_BENCH_JOINT7_POSVEL
    /* NOLOAD storage contains arbitrary bytes after reset. No reader uses it
     * until this initialization and meta version become visible. */
    memset((void *)&motor_bench_diagnostics, 0, sizeof(motor_bench_diagnostics));
    memset((void *)&motor_bench_loop_diagnostics, 0, sizeof(motor_bench_loop_diagnostics));
    /* Preserve explicit not-read errors if initialization returns before the
     * bounded calibration collection can run. */
    motor_bench_init_calibration((MotorBenchCalibrationDiagnostics *)&motor_bench_calibration_diagnostics);
    motor_bench_trace_init(&motor_bench_trace_meta, motor_bench_trace_records);
#endif
    motor_v2_init(&state.control, boot_nonce());
#if MOTOR_BENCH_JOINT7_POSVEL
    motor_bench_profile_init(&motor_bench_profile, state.control.boot_id);
#endif
    /* POS capabilities stay zero until each model/firmware is bench-qualified. */
    for (uint32_t i = 0; i < MOTOR_BRIDGE_JOINT_COUNT; ++i) {
        const MotorQualifiedCapability *cap = &motor_qualified_capabilities[i];
        if (cap->pos_vel_qualified && isfinite(cap->position_min) &&
            isfinite(cap->position_max) && isfinite(cap->velocity_limit_max) &&
            cap->position_min < cap->position_max && cap->velocity_limit_max > 0) {
            state.control.pos_vel_supported_mask |= 1U << i;
            state.control.position_min[i] = cap->position_min;
            state.control.position_max[i] = cap->position_max;
            state.control.velocity_limit_max[i] = cap->velocity_limit_max;
        }
    }
    state.snapshot.mode = MOTOR_MODE_DISABLE;
    state.snapshot.active_joint_mask = MOTOR_BRIDGE_ACTIVE_JOINT_MASK;
    if (!initialize_disabled_outputs(100U)) return;

    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        damiao_motor_state_init(&motor_state[i], motor_ids[i], feedback_ids[i],
                                &motor_limits[i]);
        DamiaoModeSwitchCtx mode_ctx = make_mode_context(i);
        if (damiao_motor_request_feedback_state(&motor_state[i], &mode_ctx,
                                                300U) != DAMIAO_ERR_OK ||
            damiao_ensure_control_mode(&mode_ctx, DAMIAO_MODE_MIT, 300U) !=
                DAMIAO_ERR_OK) {
            state.snapshot.fault_code = MOTOR_FAULT_INIT;
            request_disable();
            (void)send_disable();
            return;
        }
        state.control.control_modes[i] = 1U;
        state.control.verified_joint_mask |= 1U << i;
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
#if MOTOR_BENCH_JOINT7_POSVEL
    /* Startup only: no enabled outputs, before admitting any motion. Normal
     * control-loop register switching remains asynchronous and unchanged. */
    initialize_joint7_bench();
#endif
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
    taskENTER_CRITICAL();
    bool disable = command->mode == MOTOR_MODE_DISABLE;
#if MOTOR_BENCH_JOINT7_POSVEL
    if (!disable && motor_bench_profile_inhibits(&motor_bench_profile)) {
        state.control.rejected_sequence = command->sequence;
        state.control.reject_code = MV2_BUSY;
        taskEXIT_CRITICAL(); return false;
    }
#endif
    uint16_t fault = disable ? MOTOR_FAULT_NONE : validate_command(command);
    if (!state.initialized && !disable) fault = MOTOR_FAULT_INIT;
    else if (!disable && !motor_v2_mit_allowed(&state.control)) fault = MOTOR_FAULT_CONTROL_MODE;
    else if (state.have_command && !sequence_is_newer(command->sequence, state.snapshot.last_command_sequence))
        fault = MOTOR_FAULT_STALE_SEQUENCE;
    else if (!disable && state.zero_snapshot.busy) fault = MOTOR_FAULT_ZERO_POSITION_BUSY;
    else if (!disable && state.have_command && state.snapshot.mode != MOTOR_MODE_DISABLE &&
        (uint32_t)(now_ms - state.last_command_ms) > COMMAND_LOSS_LATCH_MS)
        state.communication_loss_latched = true;
    if (!disable && state.communication_loss_latched) fault = MOTOR_FAULT_COMMAND_LOSS_LATCHED;
    if (!disable && bench_latched_fault()) fault = bench_latched_fault();
    if (fault != MOTOR_FAULT_NONE) {
        /* Wrong-protocol traffic is diagnostic only and cannot disturb its owner. */
        if (fault == MOTOR_FAULT_CONTROL_MODE) {
            state.control.rejected_sequence = command->sequence;
            state.control.reject_code = MV2_MODE;
        } else if (!state.communication_loss_latched) state.snapshot.fault_code = fault;
        taskEXIT_CRITICAL(); return false;
    }
    state.command = *command;
    if (disable) {
#if MOTOR_BENCH_JOINT7_POSVEL
        motor_bench_profile_cancel(&motor_bench_profile, now_ms);
#endif
        memset(&state.command, 0, sizeof(state.command));
        state.command.sequence = command->sequence;
        state.command.mode = MOTOR_MODE_DISABLE;
        state.disable_ms = now_ms;
        state.have_disable = true;
        request_disable();
        motor_v2_disable(&state.control);
    } else { state.using_v2 = false; state.have_disable = false; }
    state.have_command = true;
    state.last_command_ms = now_ms;
    state.snapshot.last_command_sequence = command->sequence;
    state.snapshot.fault_code = state.communication_loss_latched ? MOTOR_FAULT_COMMAND_LOSS_LATCHED : bench_latched_fault();
    set_mode(command->mode);
    taskEXIT_CRITICAL(); return true;
}

bool motor_bridge_accept_command_v2(const MotorCommandV2 *command, uint32_t now_ms)
{
    taskENTER_CRITICAL();
    uint16_t fault = state.communication_loss_latched ? MOTOR_FAULT_COMMAND_LOSS_LATCHED : state.snapshot.fault_code;
    if (bench_latched_fault()) fault = bench_latched_fault();
    if (!state.initialized || state.zero_snapshot.busy) fault = MOTOR_FAULT_INIT;
    uint16_t error = MV2_OK;
#if MOTOR_BENCH_JOINT7_POSVEL
    if (motor_bench_profile_inhibits(&motor_bench_profile)) error = MV2_BUSY;
    else if (!motor_bench_profile_ready(&motor_bench_profile)) error = MV2_UNSUPPORTED;
#endif
    if (command->mode == MOTOR_MODE_HOLD) {
        for (uint32_t i = 0; i < MOTOR_BRIDGE_JOINT_COUNT; ++i) {
            if (!(command->active_joint_mask & (1U << i))) continue;
            if (!(state.feedback_valid_mask & (1U << i)) ||
                (uint32_t)(now_ms - state.received_feedback_ms[i]) > FEEDBACK_TIMEOUT_MS)
                error = MV2_FAULT;
            else if (!isfinite(state.snapshot.position[i]) ||
                state.snapshot.position[i] < state.control.position_min[i] ||
                state.snapshot.position[i] > state.control.position_max[i]) error = MV2_LIMITS;
        }
    }
    if (!error) error = motor_v2_command(&state.control, command, fault, state.have_command,
                                        state.snapshot.last_command_sequence, now_ms);
    else { state.control.rejected_sequence = command->sequence; state.control.reject_code = error; }
    if (!error) {
#if MOTOR_BENCH_JOINT7_POSVEL
        if (command->active_joint_mask == 0x40U) {
            motor_bench_trace_meta.armed = 1U;
            MotorBenchTraceRecord marker = {now_ms,
                motor_bench_trace_tag(MOTOR_BENCH_TRACE_ACCEPTED_V2, 7U, 0U, 0U),
                command->sequence, command->mode | ((uint32_t)command->control_mode << 8) |
                    (command->active_joint_mask << 16)};
            motor_bench_trace_append(&motor_bench_trace_meta, motor_bench_trace_records,
                                     &marker, now_ms);
        }
#endif
        state.have_disable = false;
        state.command_v2 = *command;
        state.using_v2 = true;
        state.have_command = true;
        state.last_command_ms = now_ms;
        state.snapshot.last_command_sequence = command->sequence;
        set_mode(command->mode);
    }
    taskEXIT_CRITICAL(); return error == MV2_OK;
}

MotorControlResponse motor_bridge_control_request(const MotorControlRequest *request, uint32_t now_ms)
{
    taskENTER_CRITICAL();
#if MOTOR_BENCH_JOINT7_POSVEL
    uint16_t profile_error = MV2_OK;
    if (request->action != 4U && motor_bench_profile_inhibits(&motor_bench_profile)) profile_error = MV2_BUSY;
    else if (request->action == 2U && request->control_mode == 2U &&
             !motor_bench_profile_ready(&motor_bench_profile)) profile_error = MV2_UNSUPPORTED;
    if (profile_error) {
        MotorControlResponse denied = {false, profile_error, state.control.control_epoch,
            state.control.lease_id, request->operation_id};
        taskEXIT_CRITICAL(); return denied;
    }
#endif
    uint16_t fault = state.communication_loss_latched ? MOTOR_FAULT_COMMAND_LOSS_LATCHED : state.snapshot.fault_code;
    if (bench_latched_fault()) fault = bench_latched_fault();
    if (!state.initialized || state.zero_snapshot.busy) fault = MOTOR_FAULT_INIT;
    MotorControlResponse response = motor_v2_request(&state.control, request,
        disabled_proven(now_ms), fault, state.snapshot.last_command_sequence, now_ms);
    if (response.accepted && request->action == 4U) {
#if MOTOR_BENCH_JOINT7_POSVEL
        motor_bench_profile_cancel(&motor_bench_profile, now_ms);
#endif
        request_disable();
        set_mode(MOTOR_MODE_DISABLE);
    }
    taskEXIT_CRITICAL(); return response;
}

void motor_bridge_get_control_snapshot(MotorControlSnapshot *snapshot)
{
    taskENTER_CRITICAL();
    snapshot->control = state.control;
    snapshot->disabled_verified = disabled_proven(HAL_GetTick());
    snapshot->enabled_joint_mask = 0;
    for (uint32_t i = 0; i < MOTOR_BRIDGE_JOINT_COUNT; ++i)
        if (state.motor_enabled[i]) snapshot->enabled_joint_mask |= 1U << i;
    snapshot->last_command_sequence = state.snapshot.last_command_sequence;
    snapshot->mode = state.snapshot.mode;
    snapshot->fault_code = state.communication_loss_latched ? MOTOR_FAULT_COMMAND_LOSS_LATCHED : state.snapshot.fault_code;
    taskEXIT_CRITICAL();
}

static void consume_feedback(uint32_t now_ms)
{
    uint16_t can_id;
    uint32_t received_ms;
    uint8_t data[DAMIAO_DLC];
    while (fdcan1_get_rx_data_at(&can_id, data, &received_ms) == DAMIAO_DLC) {
#if MOTOR_BENCH_JOINT7_POSVEL
        bench_trace_can(MOTOR_BENCH_TRACE_RX, can_id, data, received_ms);
#endif
        for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
            if (can_id == feedback_ids[i]) {
                if (damiao_is_register_reply(data) || damiao_is_register_write_ack(data)) {
#if MOTOR_BENCH_JOINT7_POSVEL
                    if (i == 6U)
                        motor_bench_profile_reply(&motor_bench_profile, can_id, data, received_ms, now_ms);
#endif
                    if (data[0] == motor_ids[i] && data[1] == 0U && data[3] == DAMIAO_REG_CTRL_MODE) {
                        uint32_t value; memcpy(&value, data + 4, sizeof(value));
                        motor_v2_register_reply(&state.control, (uint8_t)i,
                            damiao_is_register_write_ack(data), value, now_ms);
                    }
                    break;
                }
                DamiaoFeedback feedback;
                damiao_decode_sensor_feedback(data, &motor_limits[i], &feedback);
                state.snapshot.position[i] = feedback.pos;
                state.snapshot.velocity[i] = feedback.vel;
                state.snapshot.effort[i] = feedback.torq;
                state.snapshot.last_feedback_ms[i] = now_ms;
                state.received_feedback_ms[i] = received_ms;
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

#if MOTOR_BENCH_JOINT7_POSVEL
static bool enforce_joint7_bench_feedback(uint32_t now_ms, bool was_active)
{
    if (!state.using_v2 || state.command_v2.active_joint_mask != 0x40U ||
        (!was_active && state.snapshot.mode == MOTOR_MODE_DISABLE)) return false;
    uint32_t reason = motor_bench_feedback_reason((state.feedback_valid_mask & 0x40U) != 0,
        now_ms - state.received_feedback_ms[6], state.snapshot.position[6],
        state.snapshot.velocity[6], state.snapshot.effort[6]);
    if (!reason) return false;
    motor_bench_trace_trigger(&motor_bench_trace_meta, motor_bench_trace_records,
        MOTOR_BENCH_TRACE_TRIP, state.snapshot.last_command_sequence, reason, now_ms);
    if (!motor_bench_diagnostics.trip_reason) {
        motor_bench_diagnostics.trip_ms = now_ms;
        motor_bench_diagnostics.trip_position = state.snapshot.position[6];
        motor_bench_diagnostics.trip_velocity = state.snapshot.velocity[6];
        motor_bench_diagnostics.trip_effort = state.snapshot.effort[6];
        motor_bench_diagnostics.trip_reason = reason;
    }
    state.control.pos_vel_supported_mask = 0;
    state.snapshot.fault_code = bench_latched_fault();
    set_mode(MOTOR_MODE_DISABLE); /* Revokes the lease through the normal path. */
    (void)send_disable();
    return true;
}
#endif

static void control_step_locked(uint32_t now_ms)
{
    uint8_t data[DAMIAO_DLC];
#if MOTOR_BENCH_JOINT7_POSVEL
    motor_bench_trace_tick(&motor_bench_trace_meta, now_ms);
    (void)motor_bench_profile_deadline(&motor_bench_profile, now_ms);
#endif
    if (state.disable_pending) (void)send_disable();
    /* Wait for actual CAN transmission before requesting post-stop feedback.
     * This is a nonblocking FIFO query, never a busy wait in the 4 ms loop. */
    if (state.disable_tx_wait && fdcan1_tx_empty()) {
        state.disable_tx_wait = false;
        state.disable_ms = now_ms;
        for (uint32_t i = 0; i < MOTOR_BRIDGE_JOINT_COUNT; ++i)
            state.last_feedback_request_ms[i] = now_ms - DISABLED_FEEDBACK_REQUEST_MS;
    }
#if MOTOR_BENCH_JOINT7_POSVEL
    bool bench_was_active = state.snapshot.mode != MOTOR_MODE_DISABLE;
#endif
    consume_feedback(now_ms);
#if MOTOR_BENCH_JOINT7_POSVEL
    if (enforce_joint7_bench_feedback(now_ms, bench_was_active)) return;
    if ((state.disable_pending || state.disable_tx_wait) && motor_bench_profile.status.busy) {
        MotorBenchProfileEnvironment environment = bench_profile_environment(now_ms);
        motor_bench_profile_step(&motor_bench_profile, &environment, now_ms,
                                  bench_profile_send, NULL);
    }
#endif
    if (state.disable_pending || state.disable_tx_wait) return;
    motor_v2_step(&state.control, disabled_proven(now_ms), now_ms, runtime_register_send, NULL);

    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        /* A disabled Damiao drive does not stream feedback autonomously. */
        if (!state.motor_enabled[i] &&
            (uint32_t)(now_ms - state.last_feedback_request_ms[i]) >=
                DISABLED_FEEDBACK_REQUEST_MS) {
            damiao_encode_feedback_request_cmd(motor_ids[i], data);
            if (!send_frame(damiao_get_register_can_id(), data)) {
                state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
                request_disable();
                set_mode(MOTOR_MODE_DISABLE);
                return;
            }
            state.last_feedback_request_ms[i] = now_ms;
        }
    }

#if MOTOR_BENCH_JOINT7_POSVEL
    if (motor_bench_profile.status.busy) {
        MotorBenchProfileEnvironment environment = bench_profile_environment(now_ms);
        motor_bench_profile_step(&motor_bench_profile, &environment, now_ms,
                                  bench_profile_send, NULL);
        /* Outputs were off at admission; any unexpected enabled feedback keeps
         * the existing whole-arm STOP queue above this maintenance operation. */
        if (any_motor_enabled()) { request_disable(); (void)send_disable(); }
        return;
    }
#endif

    if (state.snapshot.mode == MOTOR_MODE_DISABLE || !state.have_command) {
        if (any_motor_enabled()) {
            request_disable();
            (void)send_disable();
        }
        for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
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

    if (state.snapshot.mode == MOTOR_MODE_HOLD) {
        if (!state.hold_latched) {
            for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
                if (state.using_v2 && (state.command_v2.active_joint_mask & (1U << i)) &&
                    (!isfinite(state.snapshot.position[i]) ||
                     state.snapshot.position[i] < state.control.position_min[i] ||
                     state.snapshot.position[i] > state.control.position_max[i])) {
                    state.snapshot.fault_code = MOTOR_FAULT_COMMAND_LIMIT;
                    set_mode(MOTOR_MODE_DISABLE); send_disable(); return;
                }
                state.hold_position[i] = state.snapshot.position[i];
            }
            state.hold_latched = true;
        }
    }

    uint32_t selected = state.using_v2 ? state.command_v2.active_joint_mask : MOTOR_BRIDGE_ACTIVE_JOINT_MASK;
    bool all_enabled = true;
    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (!(selected & (1U << i))) {
            if (state.motor_enabled[i]) {
                /* Unexpected output on an unselected axis stops the whole
                 * transaction; use the same FIFO-aware unknown-mode queue. */
                set_mode(MOTOR_MODE_DISABLE);
                (void)send_disable();
                return;
            }
            state.enable_pending[i] = false;
            continue;
        }
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
                /* A drive may retain its previous target while disabled. Load
                 * the admitted target before enable so old goals cannot run. */
                if (state.using_v2) {
                    damiao_encode_pos_vel_cmd(state.snapshot.mode == MOTOR_MODE_HOLD ?
                        state.hold_position[i] : state.command_v2.position[i],
                        state.command_v2.velocity_limit[i], data);
                    if (!send_frame(control_can_id(i), data)) {
                        state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
                        set_mode(MOTOR_MODE_DISABLE); send_disable(); return;
                    }
                }
                damiao_encode_enable_cmd(data);
                if (!send_frame(control_can_id(i), data)) {
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


    for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
        if (!(selected & (1U << i))) continue;
        if (state.using_v2) {
            damiao_encode_pos_vel_cmd(state.snapshot.mode == MOTOR_MODE_HOLD ?
                state.hold_position[i] : state.command_v2.position[i],
                state.command_v2.velocity_limit[i], data);
        } else if (state.snapshot.mode == MOTOR_MODE_HOLD) {
            damiao_encode_mit_cmd(state.hold_position[i], 0.0f, 0.0f,
                                  state.command.kp[i], state.command.kd[i],
                                  &motor_limits[i], data);
        } else {
            damiao_encode_mit_cmd(state.command.position[i],
                                  state.command.velocity[i],
                                  state.command.effort[i], state.command.kp[i],
                                  state.command.kd[i], &motor_limits[i], data);
        }
        if (!send_frame(control_can_id(i), data)) {
            state.snapshot.fault_code = MOTOR_FAULT_CAN_TX;
            set_mode(MOTOR_MODE_DISABLE);
            send_disable();
            return;
        }
    }
}

void motor_bridge_control_step(uint32_t now_ms)
{
    /* Maintenance uses blocking register helpers and stays outside this lock.
     * The normal path contains bounded RX dequeue and nonblocking CAN enqueues;
     * serialize it with callbacks so DISABLE cannot race a late enable/frame. */
    if (state.zero_snapshot.busy) { process_zero_request(); return; }
    taskENTER_CRITICAL();
    control_step_locked(now_ms);
    taskEXIT_CRITICAL();
}

void motor_bridge_safety_step(uint32_t now_ms)
{
    taskENTER_CRITICAL();
    if (!state.initialized) {
        taskEXIT_CRITICAL();
        return;
    }
#if MOTOR_BENCH_JOINT7_POSVEL
    if (enforce_joint7_bench_feedback(now_ms, false) || bench_latched_fault()) {
        state.snapshot.fault_code = bench_latched_fault();
        set_mode(MOTOR_MODE_DISABLE);
        taskEXIT_CRITICAL();
        return;
    }
#endif
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
    motor_v2_watchdog(&state.control, now_ms);
    if (state.have_command) {
        uint32_t age = now_ms - state.last_command_ms;
        if (state.snapshot.mode == MOTOR_MODE_TRACK && age > COMMAND_TIMEOUT_MS) {
            set_mode(MOTOR_MODE_HOLD);
        }
        if (state.snapshot.mode != MOTOR_MODE_DISABLE &&
            age > COMMAND_LOSS_LATCH_MS) {
            state.communication_loss_latched = true;
            motor_v2_fault(&state.control);
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
    if (state.communication_loss_latched) snapshot->fault_code = MOTOR_FAULT_COMMAND_LOSS_LATCHED;
    taskEXIT_CRITICAL();
}

bool motor_bridge_request_zero_position(uint8_t operation, uint32_t joint_mask)
{
    joint_mask &= MOTOR_BRIDGE_ACTIVE_JOINT_MASK;
    taskENTER_CRITICAL();
#if MOTOR_BENCH_JOINT7_POSVEL
    if (motor_bench_profile_inhibits(&motor_bench_profile)) {
        taskEXIT_CRITICAL(); return false;
    }
#endif
    if (!motor_v2_mit_allowed(&state.control) || state.zero_pending_operation != MOTOR_ZERO_OPERATION_NONE ||
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

#if MOTOR_BENCH_JOINT7_POSVEL
bool motor_bridge_bench_profile_request(const MotorBenchProfileRequest *request, uint32_t now_ms)
{
    taskENTER_CRITICAL();
    MotorBenchProfileEnvironment environment = bench_profile_environment(now_ms);
    bool accepted = motor_bench_profile_request(&motor_bench_profile, request, &environment, now_ms);
    taskEXIT_CRITICAL();
    return accepted;
}

void motor_bridge_get_bench_profile_status(MotorBenchProfileStatus *status)
{
    taskENTER_CRITICAL();
    *status = motor_bench_profile.status;
    taskEXIT_CRITICAL();
}

void motor_bridge_get_bench_snapshot(MotorBenchSnapshot *snapshot)
{
    taskENTER_CRITICAL();
    snapshot->admission = motor_bench_diagnostics;
    snapshot->loop = motor_bench_loop_diagnostics;
    snapshot->trace = motor_bench_trace_meta;
    snapshot->calibration = motor_bench_calibration_diagnostics;
    taskEXIT_CRITICAL();
}

uint32_t motor_bridge_copy_bench_trace(uint32_t start_sequence,
    MotorBenchTraceRecord *output, uint32_t capacity, MotorBenchTracePage *page)
{
    taskENTER_CRITICAL();
    uint32_t count = motor_bench_trace_copy(&motor_bench_trace_meta,
        motor_bench_trace_records, start_sequence, output, capacity, page);
    taskEXIT_CRITICAL();
    return count;
}
#endif

void motor_bridge_force_disable(uint16_t fault_code)
{
    taskENTER_CRITICAL();
#if MOTOR_BENCH_JOINT7_POSVEL
    motor_bench_profile_cancel(&motor_bench_profile, HAL_GetTick());
#endif
    state.snapshot.fault_code = fault_code;
    request_disable();
    set_mode(MOTOR_MODE_DISABLE);
    taskEXIT_CRITICAL();
}
