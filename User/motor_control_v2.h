#ifndef MOTOR_CONTROL_V2_H
#define MOTOR_CONTROL_V2_H
#include <stdbool.h>
#include <stdint.h>
#define MV2_JOINTS 7U
#define MV2_ALL 0x7fU
#define MV2_REG_MODE 0x0aU
#define MV2_REG_PRECISE_POSITION 0x50U
#define MV2_REGISTER_TIMEOUT_MS 100U
enum { MV2_OK, MV2_INVALID, MV2_IDENTITY, MV2_NOT_DISABLED, MV2_BUSY,
       MV2_UNSUPPORTED, MV2_MODE, MV2_LEASE, MV2_SEQUENCE, MV2_NONFINITE,
       MV2_LIMITS, MV2_FAULT, MV2_READBACK, MV2_TIMEOUT };
enum { MV2_IDLE, MV2_PENDING, MV2_SUCCESS, MV2_FAILURE, MV2_CANCELLED };
typedef struct {
    uint8_t action;
    uint32_t operation_id;
    uint64_t boot_id;
    uint32_t expected_control_epoch;
    uint64_t source_id;
    uint32_t active_joint_mask;
    uint8_t control_mode;
    uint32_t lease_id;
    uint32_t disable_sequence;
} MotorControlRequest;
typedef struct {
    bool accepted;
    uint16_t error_code;
    uint32_t control_epoch, lease_id, operation_id;
} MotorControlResponse;
typedef struct {
    uint64_t boot_id, source_id;
    uint32_t control_epoch, lease_id, sequence;
    uint8_t mode, control_mode;
    uint32_t active_joint_mask;
    float position[MV2_JOINTS], velocity_limit[MV2_JOINTS];
} MotorCommandV2;
typedef struct {
    uint64_t boot_id, owner_source_id;
    uint32_t control_epoch, lease_id, selected_joint_mask;
    uint32_t verified_joint_mask, pos_vel_supported_mask;
    uint8_t control_modes[MV2_JOINTS];
    float velocity_limit_max[MV2_JOINTS], position_min[MV2_JOINTS], position_max[MV2_JOINTS];
    bool switching;
    uint32_t operation_id;
    uint8_t operation_state;
    uint16_t error_code;
    uint32_t rejected_sequence;
    uint16_t reject_code;
    uint32_t next_lease, phase_at, last_frame_ms, lease_granted_at;
    uint8_t axis, phase, target_mode;
    uint32_t config_mask;
    MotorControlRequest last_request;
    MotorControlResponse last_response;
    bool have_request, lease_has_frame;
} MotorControlV2;
typedef bool (*MotorRegisterSend)(uint8_t axis, uint8_t rid, bool write, uint32_t value, void *context);
void motor_v2_init(MotorControlV2 *s, uint64_t boot_id);
bool motor_v2_mit_allowed(const MotorControlV2 *s);
MotorControlResponse motor_v2_request(MotorControlV2 *s, const MotorControlRequest *r,
    bool disabled_proven, uint16_t fault, uint32_t last_sequence, uint32_t now);
uint16_t motor_v2_command(MotorControlV2 *s, const MotorCommandV2 *c,
    uint16_t fault, bool have_sequence, uint32_t last_sequence, uint32_t now);
void motor_v2_disable(MotorControlV2 *s);
void motor_v2_fault(MotorControlV2 *s);
void motor_v2_watchdog(MotorControlV2 *s, uint32_t now);
void motor_v2_step(MotorControlV2 *s, bool disabled_proven, uint32_t now,
    MotorRegisterSend send, void *context);
void motor_v2_register_reply(MotorControlV2 *s, uint8_t axis, uint8_t rid, bool write,
    uint32_t value, uint32_t received_ms, uint32_t now);
#endif
