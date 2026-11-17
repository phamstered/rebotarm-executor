/* Model the real eight-entry FDCAN TX FIFO, including frames still on the bus.
 * No HAL, RTOS, serial port, or physical motor is used by this test. */
#include <assert.h>
#include <stdio.h>
#include "../User/motor_bridge.c"

FDCAN_HandleTypeDef hfdcan1;
HostRng host_rng = {0, 1, 42, 0};
enum { TX_CAPACITY = 8, HISTORY_CAPACITY = 256 };
static BspFdcanFrame tx_queue[TX_CAPACITY], accepted[HISTORY_CAPACITY];
static BspFdcanFrame rx_queue[HISTORY_CAPACITY];
static unsigned tx_head, tx_count, accepted_count, rx_head, rx_count;
static unsigned stop_transmitted, full_send_attempts;
static uint32_t clock_ms;
static bool advance_clock, drain_on_tick, reply_to_requests, fail_send, boot_order_check;

static bool is_stop(const BspFdcanFrame *frame)
{
    for (unsigned i = 0; i < 7; ++i) if (frame->data[i] != 0xff) return false;
    return frame->data[7] == 0xfd;
}

static void push_feedback(unsigned axis, uint32_t received_ms)
{
    assert(rx_count < HISTORY_CAPACITY);
    BspFdcanFrame *frame = &rx_queue[(rx_head + rx_count++) % HISTORY_CAPACITY];
    memset(frame, 0, sizeof(*frame));
    frame->id = feedback_ids[axis];
    frame->received_ms = received_ms;
    frame->data[0] = motor_ids[axis]; /* Actual DISABLED status. */
    frame->data[1] = 0x80;
    frame->data[3] = 0x80;
    frame->data[4] = 0x08;
}

static void drain(unsigned count)
{
    while (count-- && tx_count) {
        BspFdcanFrame frame = tx_queue[tx_head];
        tx_head = (tx_head + 1) % TX_CAPACITY;
        --tx_count;
        if (is_stop(&frame)) ++stop_transmitted;
        if (!reply_to_requests || frame.id != damiao_get_register_can_id()) continue;
        unsigned axis = frame.data[0] - 1U;
        assert(axis < MOTOR_BRIDGE_JOINT_COUNT);
        if (frame.data[2] == DAMIAO_REG_FEEDBACK_REQ) {
            push_feedback(axis, clock_ms);
        } else if (frame.data[2] == DAMIAO_REG_READ_MARKER) {
            assert(frame.data[3] == DAMIAO_REG_CTRL_MODE);
            assert(rx_count < HISTORY_CAPACITY);
            BspFdcanFrame *reply = &rx_queue[(rx_head + rx_count++) % HISTORY_CAPACITY];
            *reply = frame;
            reply->id = feedback_ids[axis];
            reply->received_ms = clock_ms;
            uint32_t value = DAMIAO_MODE_MIT;
            memcpy(reply->data + 4, &value, sizeof(value));
        } else {
            assert(!"Boot model must not write motor parameters");
        }
    }
}

uint32_t HAL_GetTick(void)
{
    if (advance_clock) {
        ++clock_ms;
        if (drain_on_tick) drain(1);
    }
    return clock_ms;
}

uint32_t fdcan1_tx_free(void) { return TX_CAPACITY - tx_count; }
bool fdcan1_tx_empty(void) { return tx_count == 0; }

uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *handle, uint16_t id,
                        uint8_t *data, uint32_t length)
{
    (void)handle;
    assert(length == DAMIAO_DLC && accepted_count < HISTORY_CAPACITY);
    if (fail_send) return 1;
    if (tx_count == TX_CAPACITY) { ++full_send_attempts; return 1; }
    BspFdcanFrame frame = {.id = id, .received_ms = clock_ms};
    memcpy(frame.data, data, DAMIAO_DLC);
    if (boot_order_check && !is_stop(&frame)) assert(stop_transmitted >= 14);
    accepted[accepted_count++] = frame;
    tx_queue[(tx_head + tx_count++) % TX_CAPACITY] = frame;
    return 0;
}

uint8_t fdcan1_get_rx_data_at(uint16_t *id, uint8_t *data, uint32_t *received_ms)
{
    if (!rx_count) return 0;
    BspFdcanFrame frame = rx_queue[rx_head];
    rx_head = (rx_head + 1) % HISTORY_CAPACITY;
    --rx_count;
    *id = frame.id;
    memcpy(data, frame.data, DAMIAO_DLC);
    if (received_ms) *received_ms = frame.received_ms;
    return DAMIAO_DLC;
}

uint8_t fdcan1_get_rx_data(uint16_t *id, uint8_t *data)
{
    return fdcan1_get_rx_data_at(id, data, NULL);
}

static void fixture(void)
{
    memset(&state, 0, sizeof(state));
    motor_v2_init(&state.control, 42);
    state.initialized = true;
    state.snapshot.mode = MOTOR_MODE_DISABLE;
    state.snapshot.feedback_valid = true;
    state.feedback_valid_mask = MV2_ALL;
    for (unsigned i = 0; i < MOTOR_BRIDGE_JOINT_COUNT; ++i) {
        state.snapshot.last_feedback_ms[i] = 100;
        state.received_feedback_ms[i] = 100;
    }
    tx_head = tx_count = accepted_count = rx_head = rx_count = 0;
    stop_transmitted = full_send_attempts = 0;
    clock_ms = 100;
    advance_clock = drain_on_tick = reply_to_requests = fail_send = boot_order_check = false;
}

static MotorBridgeCommand disable(uint32_t sequence)
{
    MotorBridgeCommand command = {0};
    command.sequence = sequence;
    command.mode = MOTOR_MODE_DISABLE;
    return command;
}

static void assert_all_unknown_stops(unsigned count)
{
    unsigned seen[14] = {0};
    assert(count == 14);
    for (unsigned i = 0; i < count; ++i) {
        assert(is_stop(&accepted[i]));
        unsigned channel = accepted[i].id >= 0x100 ? 1 : 0;
        unsigned motor = accepted[i].id - (channel ? 0x100 : 0);
        assert(motor >= 1 && motor <= 7);
        ++seen[2 * (motor - 1) + channel];
    }
    for (unsigned i = 0; i < 14; ++i) assert(seen[i] == 1);
}

static void test_partial_stop_and_repeated_requests(void)
{
    fixture();
    MotorBridgeCommand command = disable(1);
    assert(motor_bridge_accept_command(&command, clock_ms));
    motor_bridge_control_step(clock_ms);
    assert(accepted_count == TX_CAPACITY && tx_count == TX_CAPACITY);
    assert(state.disable_pending && !disabled_proven(clock_ms));
    assert(state.snapshot.fault_code == MOTOR_FAULT_NONE && full_send_attempts == 0);
    /* A full bus produces no busy-wait or additional send attempt. */
    motor_bridge_control_step(clock_ms);
    assert(clock_ms == 100 && accepted_count == TX_CAPACITY && full_send_attempts == 0);
    for (unsigned i = 0; i < 6; ++i) {
        drain(1);
        ++clock_ms;
        command.sequence++;
        assert(motor_bridge_accept_command(&command, clock_ms));
        motor_bridge_control_step(clock_ms);
        assert(accepted_count == TX_CAPACITY + i + 1);
        assert(!disabled_proven(clock_ms));
    }
    assert_all_unknown_stops(accepted_count);
    assert(!state.disable_pending && state.disable_tx_wait);
    assert(full_send_attempts == 0 && state.snapshot.fault_code == MOTOR_FAULT_NONE);
}

static void test_transmit_and_feedback_proof(void)
{
    fixture();
    for (unsigned i = 0; i < 7; ++i) state.control.control_modes[i] = 1;
    MotorBridgeCommand command = disable(10);
    assert(motor_bridge_accept_command(&command, 100));
    motor_bridge_control_step(100);
    assert(tx_count == 7 && state.disable_tx_wait && !disabled_proven(101));
    /* Reception newer than command acceptance but before TX completion is insufficient. */
    for (unsigned i = 0; i < 7; ++i) push_feedback(i, 101);
    clock_ms = 102;
    command.sequence = 11;
    assert(motor_bridge_accept_command(&command, clock_ms));
    motor_bridge_control_step(clock_ms);
    assert(accepted_count == 7 && !disabled_proven(clock_ms));
    assert(state.snapshot.last_command_sequence == 11);
    drain(TX_CAPACITY);
    clock_ms = 103;
    motor_bridge_control_step(clock_ms);
    assert(!state.disable_pending && !state.disable_tx_wait);
    assert(state.disable_ms == clock_ms && !disabled_proven(clock_ms));
    for (unsigned i = 0; i < 7; ++i) push_feedback(i, 103);
    clock_ms = 104;
    motor_bridge_control_step(clock_ms);
    assert(!disabled_proven(clock_ms));
    for (unsigned i = 0; i < 7; ++i) push_feedback(i, 104);
    clock_ms = 105;
    motor_bridge_control_step(clock_ms);
    assert(disabled_proven(clock_ms));
    assert(state.snapshot.last_command_sequence == 11);
}

static void test_hard_error_keeps_stop_pending(void)
{
    fixture();
    request_disable();
    fail_send = true;
    assert(!send_disable());
    assert(accepted_count == 0 && state.disable_pending);
    assert(state.snapshot.fault_code == MOTOR_FAULT_CAN_TX);
    fail_send = false;
    assert(!send_disable());
    assert(accepted_count == TX_CAPACITY && state.disable_pending);
    drain(TX_CAPACITY);
    assert(send_disable());
    assert_all_unknown_stops(accepted_count);
    assert(state.snapshot.fault_code == MOTOR_FAULT_CAN_TX);
}

static void test_new_unknown_mode_adds_stop_channel(void)
{
    fixture();
    for (unsigned i = 0; i < 7; ++i) state.control.control_modes[i] = 1;
    request_disable();
    assert(send_disable() && accepted_count == 7 && state.disable_tx_wait);
    /* A readback becoming unknown during the in-flight stop must add its POS
     * channel without losing or replaying the seven queued MIT targets. */
    state.control.control_modes[6] = 0;
    request_disable();
    assert(state.disable_pending && !state.disable_tx_wait);
    assert(send_disable() && accepted_count == 8);
    assert(accepted[7].id == 0x107 && is_stop(&accepted[7]));
    request_disable();
    assert(send_disable() && accepted_count == 8 && state.disable_tx_wait);
    /* Later narrowing of the readback cannot cancel already required stops. */
    state.control.control_modes[6] = 1;
    request_disable();
    assert(send_disable() && accepted_count == 8);
    assert(full_send_attempts == 0 && state.snapshot.fault_code == MOTOR_FAULT_NONE);
}

static void test_unselected_enabled_axis_stops_with_backpressure(void)
{
    fixture();
    for (unsigned i = 0; i < 7; ++i) state.control.control_modes[i] = 1;
    state.control.control_modes[0] = 0;
    state.control.control_modes[6] = 2;
    state.using_v2 = state.have_command = true;
    state.snapshot.mode = MOTOR_MODE_TRACK;
    state.command_v2.active_joint_mask = 0x40;
    state.control.lease_id = 9;
    state.motor_enabled[0] = true; /* Unselected, unexpectedly enabled axis. */
    motor_bridge_control_step(clock_ms);
    assert(state.snapshot.mode == MOTOR_MODE_DISABLE && !state.control.lease_id);
    assert(state.disable_pending && full_send_attempts == 0);
    assert(state.snapshot.fault_code == MOTOR_FAULT_NONE);
    drain(TX_CAPACITY);
    motor_bridge_control_step(++clock_ms);
    assert(!state.disable_pending && state.disable_tx_wait);
    unsigned stops = 0, unknown_mit = 0, unknown_pos = 0;
    for (unsigned i = 0; i < accepted_count; ++i) {
        if (!is_stop(&accepted[i])) continue;
        ++stops;
        if (accepted[i].id == 1) ++unknown_mit;
        if (accepted[i].id == 0x101) ++unknown_pos;
    }
    assert(stops == 8 && unknown_mit == 1 && unknown_pos == 1);
    assert(full_send_attempts == 0 && state.snapshot.fault_code == MOTOR_FAULT_NONE);
}

static void test_startup_wait_is_bounded(void)
{
    fixture();
    advance_clock = drain_on_tick = true;
    assert(initialize_disabled_outputs(100));
    assert_all_unknown_stops(accepted_count);
    assert(stop_transmitted == 14 && fdcan1_tx_empty() && full_send_attempts == 0);
    assert(clock_ms < 200 && state.snapshot.fault_code == MOTOR_FAULT_NONE);

    fixture();
    advance_clock = true; /* The bus never drains. */
    assert(!initialize_disabled_outputs(20));
    assert(clock_ms >= 120 && clock_ms < 140);
    assert(accepted_count == TX_CAPACITY && state.snapshot.fault_code == MOTOR_FAULT_CAN_TX);
    assert(!state.initialized || !disabled_proven(clock_ms));
    assert(full_send_attempts == 0);
}

static void test_real_boot_orders_stop_before_feedback(void)
{
    fixture();
    advance_clock = drain_on_tick = reply_to_requests = boot_order_check = true;
    motor_bridge_init();
    assert(state.initialized && state.control.verified_joint_mask == MV2_ALL);
    assert(state.snapshot.mode == MOTOR_MODE_DISABLE && state.snapshot.fault_code == MOTOR_FAULT_NONE);
    assert(stop_transmitted == 14 && full_send_attempts == 0);
    assert_all_unknown_stops(14);
    assert(!disabled_proven(clock_ms)); /* Startup alone cannot authorize a mode switch. */
    for (unsigned i = 0; i < accepted_count; ++i) assert(accepted[i].data[7] != 0xfc);
}

static void test_failed_boot_still_completes_stop_at_runtime(void)
{
    fixture();
    advance_clock = true;
    motor_bridge_init();
    assert(!state.initialized && state.control.verified_joint_mask == 0);
    assert(state.snapshot.fault_code == MOTOR_FAULT_CAN_TX);
    assert(accepted_count == TX_CAPACITY && state.disable_pending);
    assert(clock_ms < 250 && full_send_attempts == 0);
    /* Bus recovery may finish the required stop, but must not silently mark
     * the failed firmware initialization successful or enable any drive. */
    advance_clock = false;
    drain(TX_CAPACITY);
    motor_bridge_control_step(++clock_ms);
    assert_all_unknown_stops(accepted_count);
    assert(state.disable_tx_wait && !state.initialized && !disabled_proven(clock_ms));
    drain(TX_CAPACITY);
    motor_bridge_control_step(++clock_ms);
    assert(stop_transmitted == 14 && !state.disable_pending && !state.disable_tx_wait);
    assert(!state.initialized && state.control.verified_joint_mask == 0);
    assert(state.snapshot.fault_code == MOTOR_FAULT_CAN_TX && !disabled_proven(clock_ms));
    for (unsigned i = 0; i < accepted_count; ++i) assert(accepted[i].data[7] != 0xfc);
}

int main(void)
{
    test_partial_stop_and_repeated_requests();
    test_transmit_and_feedback_proof();
    test_hard_error_keeps_stop_pending();
    test_new_unknown_mode_adds_stop_channel();
    test_unselected_enabled_axis_stops_with_backpressure();
    test_startup_wait_is_bounded();
    test_real_boot_orders_stop_before_feedback();
    test_failed_boot_still_completes_stop_at_runtime();
    puts("eight-slot motor STOP FIFO tests passed");
    return 0;
}
