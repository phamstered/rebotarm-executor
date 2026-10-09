#ifndef MOTOR_BRIDGE_H
#define MOTOR_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_BRIDGE_JOINT_COUNT 7U
#define MOTOR_BRIDGE_ACTIVE_JOINT_MASK 0x7FUL
#define MOTOR_BRIDGE_CONTROL_PERIOD_MS 4U

typedef enum {
    MOTOR_MODE_HOLD = 0,
    MOTOR_MODE_TRACK = 1,
    MOTOR_MODE_RETURN = 2,
    MOTOR_MODE_DISABLE = 3
} MotorBridgeMode;

typedef enum {
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_INVALID_MODE = 1,
    MOTOR_FAULT_NONFINITE_COMMAND = 2,
    MOTOR_FAULT_COMMAND_LIMIT = 3,
    MOTOR_FAULT_STALE_SEQUENCE = 4,
    MOTOR_FAULT_INACTIVE_JOINT_COMMAND = 5,
    MOTOR_FAULT_FEEDBACK_TIMEOUT = 6,
    MOTOR_FAULT_DRIVE = 7,
    MOTOR_FAULT_CAN_TX = 8,
    MOTOR_FAULT_INIT = 9,
    MOTOR_FAULT_ENABLE_TIMEOUT = 10,
    MOTOR_FAULT_COMMAND_LOSS_LATCHED = 11,
    MOTOR_FAULT_ZERO_POSITION_BUSY = 12
} MotorBridgeFault;

typedef enum {
    MOTOR_ZERO_OPERATION_NONE = 0,
    MOTOR_ZERO_OPERATION_GET = 1,
    MOTOR_ZERO_OPERATION_SET = 2
} MotorBridgeZeroOperation;

typedef enum {
    MOTOR_ZERO_RESULT_NONE = 0,
    MOTOR_ZERO_RESULT_OK = 1,
    MOTOR_ZERO_RESULT_INVALID_MASK = 2,
    MOTOR_ZERO_RESULT_BUSY = 3,
    MOTOR_ZERO_RESULT_UNSAFE_MODE = 4,
    MOTOR_ZERO_RESULT_UNSAFE_MOTION = 5,
    MOTOR_ZERO_RESULT_IO = 6,
    MOTOR_ZERO_RESULT_VERIFY = 7,
    MOTOR_ZERO_RESULT_NOT_READY = 8
} MotorBridgeZeroResultCode;

typedef struct {
    uint32_t sequence;
    uint8_t mode;
    float position[MOTOR_BRIDGE_JOINT_COUNT];
    float velocity[MOTOR_BRIDGE_JOINT_COUNT];
    float kp[MOTOR_BRIDGE_JOINT_COUNT];
    float kd[MOTOR_BRIDGE_JOINT_COUNT];
    float effort[MOTOR_BRIDGE_JOINT_COUNT];
} MotorBridgeCommand;

typedef struct {
    uint32_t last_command_sequence;
    uint8_t mode;
    uint32_t active_joint_mask;
    uint16_t fault_code;
    bool feedback_valid;
    float position[MOTOR_BRIDGE_JOINT_COUNT];
    float velocity[MOTOR_BRIDGE_JOINT_COUNT];
    float effort[MOTOR_BRIDGE_JOINT_COUNT];
    uint32_t last_feedback_ms[MOTOR_BRIDGE_JOINT_COUNT];
} MotorBridgeSnapshot;

typedef struct {
    uint32_t sequence;
    uint8_t operation;
    uint32_t requested_mask;
    uint32_t success_mask;
    uint32_t failure_mask;
    uint8_t result_code;
    bool busy;
    float output_offset[MOTOR_BRIDGE_JOINT_COUNT];
} MotorBridgeZeroSnapshot;

void motor_bridge_init(void);
bool motor_bridge_accept_command(const MotorBridgeCommand *command, uint32_t now_ms);
void motor_bridge_control_step(uint32_t now_ms);
void motor_bridge_safety_step(uint32_t now_ms);
void motor_bridge_get_snapshot(MotorBridgeSnapshot *snapshot);
bool motor_bridge_request_zero_position(uint8_t operation, uint32_t joint_mask);
void motor_bridge_get_zero_snapshot(MotorBridgeZeroSnapshot *snapshot);
void motor_bridge_force_disable(uint16_t fault_code);

#endif
