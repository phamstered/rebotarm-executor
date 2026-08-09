#ifndef MOTOR_BRIDGE_H
#define MOTOR_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_BRIDGE_JOINT_COUNT 7U
#define MOTOR_BRIDGE_ACTIVE_JOINT_MASK 0x01UL

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
    MOTOR_FAULT_ENABLE_TIMEOUT = 10
} MotorBridgeFault;

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
    float position;
    float velocity;
    float effort;
    uint32_t last_feedback_ms;
} MotorBridgeSnapshot;

void motor_bridge_init(void);
bool motor_bridge_accept_command(const MotorBridgeCommand *command, uint32_t now_ms);
void motor_bridge_control_step(uint32_t now_ms);
void motor_bridge_safety_step(uint32_t now_ms);
void motor_bridge_get_snapshot(MotorBridgeSnapshot *snapshot);
void motor_bridge_force_disable(uint16_t fault_code);

#endif
