/**
 * @file damiao_protocol.h
 * @brief Damiao CAN Protocol Implementation for STM32
 *
 * This header defines the CAN protocol interface for Damiao motors,
 * matching the motorbridge Rust implementation exactly.
 *
 * @author motorbridge contributors
 * @version 1.0.0
 */

#ifndef DAMIAO_PROTOCOL_H
#define DAMIAO_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * Constants
 *===========================================================================*/

/** CAN frame data length */
#define DAMIAO_DLC 8

/** Control command last byte markers */
#define DAMIAO_CMD_ENABLE       0xFC
#define DAMIAO_CMD_DISABLE      0xFD
#define DAMIAO_CMD_SET_ZERO     0xFE
#define DAMIAO_REG_OUTPUT_POS_OFFSET 54U
#define DAMIAO_CMD_CLEAR_ERROR  0xFB

/** Register command markers */
#define DAMIAO_REG_READ_MARKER  0x33
#define DAMIAO_REG_WRITE_MARKER 0x55
#define DAMIAO_REG_STORE_MARKER 0xAA
#define DAMIAO_REG_FEEDBACK_REQ 0xCC

/** Status codes in feedback frame */
#define DAMIAO_STATUS_DISABLED      0x0
#define DAMIAO_STATUS_ENABLED       0x1
#define DAMIAO_STATUS_OVER_VOLTAGE  0x8
#define DAMIAO_STATUS_UNDER_VOLTAGE 0x9
#define DAMIAO_STATUS_OVER_CURRENT  0xA
#define DAMIAO_STATUS_MOS_OVER_TEMP 0xB
#define DAMIAO_STATUS_ROTOR_OVER_TEMP 0xC
#define DAMIAO_STATUS_LOST_COMM     0xD
#define DAMIAO_STATUS_OVERLOAD      0xE

/** Parameter ranges (default for 4340P, adjust per motor model) */
#define DAMIAO_KP_MIN  0.0f
#define DAMIAO_KP_MAX  500.0f
#define DAMIAO_KD_MIN  0.0f
#define DAMIAO_KD_MAX  5.0f

/** Mode switch timing constants (ms) */
#define DAMIAO_ENSURE_MODE_SETTLE_MS           20
#define DAMIAO_ENSURE_MODE_VERIFY_ATTEMPTS     3
#define DAMIAO_ENSURE_MODE_VERIFY_RETRY_GAP_MS 10
#define DAMIAO_ENSURE_MODE_READ_TIMEOUT_MS     100

/*===========================================================================
 * Type Definitions
 *===========================================================================*/

/**
 * @brief Position/velocity/torque limits for a motor model
 */
typedef struct {
    float p_min;  /**< Minimum position (rad) */
    float p_max;  /**< Maximum position (rad) */
    float v_min;  /**< Minimum velocity (rad/s) */
    float v_max;  /**< Maximum velocity (rad/s) */
    float t_min;  /**< Minimum torque (N·m) */
    float t_max;  /**< Maximum torque (N·m) */
} DamiaoLimits;

/**
 * @brief Sensor feedback decoded from CAN frame
 */
typedef struct {
    uint8_t motor_id;     /**< Motor ID from feedback frame */
    uint8_t status_code;  /**< Status code (see DAMIAO_STATUS_*) */
    float pos;            /**< Position (rad) */
    float vel;            /**< Velocity (rad/s) */
    float torq;           /**< Torque (N·m) */
    float t_mos;          /**< MOS temperature (°C) */
    float t_rotor;        /**< Rotor temperature (°C) */
} DamiaoFeedback;

/**
 * @brief Register info decoded from register reply
 */
typedef struct {
    uint8_t rid;          /**< Register ID */
    uint8_t data[4];      /**< Raw 4-byte data */
} DamiaoRegisterValue;

/**
 * @brief Control modes (register RID=10 values)
 *
 * These values match the motor's CTRL_MODE register exactly.
 */
typedef enum {
    DAMIAO_MODE_MIT = 1,      /**< MIT mode (position-velocity-torque) */
    DAMIAO_MODE_POS_VEL = 2,  /**< Position-velocity mode */
    DAMIAO_MODE_VEL = 3,      /**< Velocity mode */
    DAMIAO_MODE_FORCE_POS = 4 /**< Force-position mode */
} DamiaoControlMode;

/** Register ID for control mode */
#define DAMIAO_REG_CTRL_MODE    10
/** Register ID for motor position (used to hold position during mode switch) */
#define DAMIAO_REG_MOTOR_POS    80

/**
 * @brief Error codes for motor operations
 */
typedef enum {
    DAMIAO_ERR_OK = 0,             /**< Success */
    DAMIAO_ERR_CAN_READ = -1,      /**< CAN read callback failed */
    DAMIAO_ERR_CAN_WRITE = -2,     /**< CAN write callback failed */
    DAMIAO_ERR_TIMEOUT = -3,       /**< Operation timed out */
    DAMIAO_ERR_MODE_VERIFY = -4,   /**< Mode verification failed after write */
    DAMIAO_ERR_INVALID_ARG = -5,   /**< Invalid argument */
    DAMIAO_ERR_NOT_DISABLED = -6,  /**< Motor must be disabled before this operation */
    DAMIAO_ERR_REGISTER_RO = -7,   /**< Attempted to write a read-only register */
    DAMIAO_ERR_TYPE_MISMATCH = -8  /**< Register data type mismatch */
} DamiaoError;

/**
 * @brief Callback: send a CAN frame
 *
 * @param can_id  CAN arbitration ID
 * @param data    8-byte frame data
 * @param ctx     User context pointer
 * @return 0 on success, negative on error
 */
typedef int (*DamiaoSendFn)(uint16_t can_id, const uint8_t *data, void *ctx);

/**
 * @brief Callback: receive a CAN frame (blocking with timeout)
 *
 * @param can_id_out Output: received CAN arbitration ID
 * @param data       Output: 8-byte frame data
 * @param timeout_ms Timeout in milliseconds
 * @param ctx        User context pointer
 * @return 0 on success, negative on error/timeout
 */
typedef int (*DamiaoRecvFn)(uint16_t *can_id_out, uint8_t *data,
                            uint32_t timeout_ms, void *ctx);

/**
 * @brief Callback: sleep for specified milliseconds
 *
 * @param ms  Milliseconds to sleep
 * @param ctx User context pointer
 */
typedef void (*DamiaoSleepFn)(uint32_t ms, void *ctx);

/**
 * @brief Context for ensure_control_mode operation
 */
typedef struct {
    uint16_t        motor_id;     /**< Motor ID */
    DamiaoSendFn    send;         /**< CAN send callback */
    DamiaoRecvFn    recv;         /**< CAN receive callback */
    DamiaoSleepFn   sleep;        /**< Sleep callback */
    void           *user_ctx;     /**< User context passed to callbacks */
    const DamiaoLimits *limits;   /**< Motor limits (for initial command) */
} DamiaoModeSwitchCtx;

/*===========================================================================
 * Utility Functions
 *===========================================================================*/

/**
 * @brief Convert float to unsigned integer with linear mapping
 *
 * @param x     Input float value
 * @param x_min Minimum of float range
 * @param x_max Maximum of float range
 * @param bits  Number of bits for integer representation
 * @return Mapped unsigned integer value
 */
uint32_t damiao_float_to_uint(float x, float x_min, float x_max, uint8_t bits);

/**
 * @brief Convert unsigned integer to float with linear mapping
 *
 * @param x     Input unsigned integer
 * @param x_min Minimum of float range
 * @param x_max Maximum of float range
 * @param bits  Number of bits for integer representation
 * @return Mapped float value
 */
float damiao_uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits);

/*===========================================================================
 * Command Encoding Functions
 *===========================================================================*/

/**
 * @brief Encode MIT control command
 *
 * @param pos    Target position (rad)
 * @param vel    Target velocity (rad/s)
 * @param torq   Feedforward torque (N·m)
 * @param kp     Position gain
 * @param kd     Damping gain
 * @param limits Motor limits
 * @param data   Output 8-byte CAN data
 */
void damiao_encode_mit_cmd(float pos, float vel, float torq, float kp, float kd,
                           const DamiaoLimits *limits, uint8_t *data);

/**
 * @brief Encode position-velocity control command
 *
 * @param target_position Target position (rad)
 * @param velocity_limit  Velocity limit (rad/s)
 * @param data            Output 8-byte CAN data
 */
void damiao_encode_pos_vel_cmd(float target_position, float velocity_limit,
                               uint8_t *data);

/**
 * @brief Encode velocity control command
 *
 * @param target_velocity Target velocity (rad/s)
 * @param data            Output 8-byte CAN data
 */
void damiao_encode_vel_cmd(float target_velocity, uint8_t *data);

/**
 * @brief Encode force-position control command
 *
 * @param target_position  Target position (rad)
 * @param velocity_limit   Velocity limit (0-100)
 * @param torque_limit_ratio Torque limit ratio (0-1)
 * @param data             Output 8-byte CAN data
 */
void damiao_encode_force_pos_cmd(float target_position, float velocity_limit,
                                 float torque_limit_ratio, uint8_t *data);

/*===========================================================================
 * Control Command Encoding
 *===========================================================================*/

/**
 * @brief Encode enable command
 * @param data Output 8-byte CAN data
 */
void damiao_encode_enable_cmd(uint8_t *data);

/**
 * @brief Encode disable command
 * @param data Output 8-byte CAN data
 */
void damiao_encode_disable_cmd(uint8_t *data);

/**
 * @brief Encode set zero position command
 * @param data Output 8-byte CAN data
 */
void damiao_encode_set_zero_cmd(uint8_t *data);

/**
 * @brief Encode clear error command
 * @param data Output 8-byte CAN data
 */
void damiao_encode_clear_error_cmd(uint8_t *data);

/*===========================================================================
 * Register Command Encoding
 *===========================================================================*/

/**
 * @brief Encode register read command
 *
 * @param motor_id Motor ID
 * @param rid      Register ID
 * @param data     Output 8-byte CAN data
 */
void damiao_encode_register_read_cmd(uint16_t motor_id, uint8_t rid, uint8_t *data);

/**
 * @brief Encode register write command
 *
 * @param motor_id Motor ID
 * @param rid      Register ID
 * @param value    4-byte value (float or uint32, little-endian)
 * @param data     Output 8-byte CAN data
 */
void damiao_encode_register_write_cmd(uint16_t motor_id, uint8_t rid,
                                      uint32_t value, uint8_t *data);

/**
 * @brief Encode store parameters command
 *
 * @param motor_id Motor ID
 * @param data     Output 8-byte CAN data
 */
void damiao_encode_store_params_cmd(uint16_t motor_id, uint8_t *data);

/**
 * @brief Encode feedback request command
 *
 * @param motor_id Motor ID
 * @param data     Output 8-byte CAN data
 */
void damiao_encode_feedback_request_cmd(uint16_t motor_id, uint8_t *data);

/*===========================================================================
 * Feedback Decoding
 *===========================================================================*/

/**
 * @brief Decode sensor feedback frame
 *
 * @param data   8-byte CAN data from feedback frame
 * @param limits Motor limits for position/velocity/torque scaling
 * @param fb     Output feedback structure
 */
void damiao_decode_sensor_feedback(const uint8_t *data, const DamiaoLimits *limits,
                                   DamiaoFeedback *fb);

/*===========================================================================
 * Register Reply Decoding
 *===========================================================================*/

/**
 * @brief Check if frame is a register read reply
 *
 * @param data 8-byte CAN data
 * @return true if register reply, false otherwise
 */
bool damiao_is_register_reply(const uint8_t *data);

/**
 * @brief Check if frame is a register write acknowledgment
 *
 * @param data 8-byte CAN data
 * @return true if write ACK, false otherwise
 */
bool damiao_is_register_write_ack(const uint8_t *data);

/**
 * @brief Decode register value from reply frame
 *
 * @param data 8-byte CAN data
 * @param rv   Output register value structure
 * @return true if valid register reply, false otherwise
 */
bool damiao_decode_register_value(const uint8_t *data, DamiaoRegisterValue *rv);

/*===========================================================================
 * CAN ID Calculation Helpers
 *===========================================================================*/

/**
 * @brief Get CAN ID for MIT mode
 * @param motor_id Motor ID
 * @return CAN ID for MIT command
 */
static inline uint16_t damiao_get_mit_can_id(uint16_t motor_id) {
    return motor_id;
}

/**
 * @brief Get CAN ID for POS_VEL mode
 * @param motor_id Motor ID
 * @return CAN ID for POS_VEL command
 */
static inline uint16_t damiao_get_pos_vel_can_id(uint16_t motor_id) {
    return 0x100 + motor_id;
}

/**
 * @brief Get CAN ID for VEL mode
 * @param motor_id Motor ID
 * @return CAN ID for VEL command
 */
static inline uint16_t damiao_get_vel_can_id(uint16_t motor_id) {
    return 0x200 + motor_id;
}

/**
 * @brief Get CAN ID for FORCE_POS mode
 * @param motor_id Motor ID
 * @return CAN ID for FORCE_POS command
 */
static inline uint16_t damiao_get_force_pos_can_id(uint16_t motor_id) {
    return 0x300 + motor_id;
}

/**
 * @brief Get CAN ID for register access
 * @return Fixed CAN ID 0x7FF
 */
static inline uint16_t damiao_get_register_can_id(void) {
    return 0x7FF;
}

/*===========================================================================
 * Register Value Helpers
 *===========================================================================*/

/**
 * @brief Decode uint32 value from register reply data bytes
 *
 * @param data 4 raw data bytes from register reply (little-endian)
 * @return Decoded uint32 value
 */
static inline uint32_t damiao_decode_register_u32(const uint8_t *data) {
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8)
         | ((uint32_t)data[2] << 16)
         | ((uint32_t)data[3] << 24);
}

/**
 * @brief Decode float value from register reply data bytes
 *
 * @param data 4 raw data bytes from register reply (little-endian)
 * @return Decoded float value
 */
static inline float damiao_decode_register_f32(const uint8_t *data) {
    float val;
    memcpy(&val, data, 4);
    return val;
}

/*===========================================================================
 * Control Mode Switching
 *===========================================================================*/

/**
 * @brief Read a register value from the motor
 *
 * Sends a register read request and waits for the reply.
 *
 * @param ctx      Mode switch context (provides callbacks)
 * @param rid      Register ID to read
 * @param value    Output: decoded uint32 value
 * @param timeout_ms  Timeout in milliseconds
 * @return DAMIAO_ERR_OK on success, negative DamiaoError on failure
 */
int damiao_read_register_u32(const DamiaoModeSwitchCtx *ctx, uint8_t rid,
                             uint32_t *value, uint32_t timeout_ms);

/**
 * @brief Write a register value to the motor
 *
 * Sends a register write command and waits for write ACK.
 *
 * @param ctx      Mode switch context (provides callbacks)
 * @param rid      Register ID to write
 * @param value    uint32 value to write (little-endian)
 * @param timeout_ms  Timeout in milliseconds
 * @return DAMIAO_ERR_OK on success, negative DamiaoError on failure
 */
int damiao_write_register_u32(const DamiaoModeSwitchCtx *ctx, uint8_t rid,
                              uint32_t value, uint32_t timeout_ms);

/**
 * @brief Ensure the motor is in the specified control mode
 *
 * Reads current CTRL_MODE register. If already in the desired mode, returns
 * immediately. Otherwise:
 *   1. Caches current motor position (for MIT/PosVel/ForcePos)
 *   2. Clears all command channels to safe defaults
 *   3. Writes CTRL_MODE register with desired mode
 *   4. Waits for settle, then sends initial hold command for the new mode
 *   5. Verifies the mode by reading CTRL_MODE register back
 *   6. Retries on verification failure
 *
 * @param ctx          Mode switch context (provides motor_id, callbacks, limits)
 * @param mode         Desired control mode
 * @param timeout_ms   Total timeout budget in milliseconds
 * @return DAMIAO_ERR_OK on success, negative DamiaoError on failure
 */
int damiao_ensure_control_mode(const DamiaoModeSwitchCtx *ctx,
                               DamiaoControlMode mode,
                               uint32_t timeout_ms);

/*===========================================================================
 * Status Code Helpers
 *===========================================================================*/

/**
 * @brief Get human-readable status name
 *
 * @param status_code Status code from feedback
 * @return Constant string with status name
 */
const char *damiao_status_name(uint8_t status_code);

/*===========================================================================
 * Register Table & Validation
 *===========================================================================*/

/**
 * @brief Register data type
 */
typedef enum {
    DAMIAO_DATA_FLOAT = 0,  /**< IEEE 754 float */
    DAMIAO_DATA_UINT32 = 1  /**< Unsigned 32-bit integer */
} DamiaoDataType;

/**
 * @brief Register access mode
 */
typedef enum {
    DAMIAO_ACCESS_RO = 0,  /**< Read-only */
    DAMIAO_ACCESS_RW = 1   /**< Read-write */
} DamiaoAccess;

/**
 * @brief Register metadata
 */
typedef struct {
    uint8_t         rid;        /**< Register ID */
    DamiaoDataType  data_type;  /**< Data type */
    DamiaoAccess    access;     /**< Access mode */
} DamiaoRegisterInfo;

/**
 * @brief Write retry / ACK configuration
 */
#define DAMIAO_WRITE_ACK_ATTEMPTS      3
#define DAMIAO_WRITE_ACK_TIMEOUT_MS    50
#define DAMIAO_WRITE_ACK_RETRY_GAP_MS  20

/**
 * @brief Look up register info by RID
 *
 * @param rid Register ID
 * @return Pointer to register info, or NULL if unknown
 */
const DamiaoRegisterInfo *damiao_register_info(uint8_t rid);

/**
 * @brief Validate that a register can be written with the given data type
 *
 * Checks: RID exists, access is ReadWrite, data type matches.
 *
 * @param rid          Register ID
 * @param expected_type Expected data type (Float or UInt32)
 * @return DAMIAO_ERR_OK on success, negative DamiaoError on failure
 */
int damiao_validate_register_write(uint8_t rid, DamiaoDataType expected_type);

/*===========================================================================
 * Write Register with Retry & ACK Validation
 *===========================================================================*/

/**
 * @brief Write a uint32 register with retry and ACK value verification
 *
 * Sends write command up to DAMIAO_WRITE_ACK_ATTEMPTS times. Each attempt
 * waits for a write-ACK frame and verifies the ACK data bytes match the
 * written value. Retries on timeout or mismatch.
 *
 * Also validates the register via damiao_validate_register_write() first.
 *
 * @param ctx        Mode switch context (provides motor_id, callbacks)
 * @param rid        Register ID
 * @param value      uint32 value to write
 * @param timeout_ms Per-attempt timeout in milliseconds
 * @return DAMIAO_ERR_OK on success, negative DamiaoError on failure
 */
int damiao_write_register_u32_retry(const DamiaoModeSwitchCtx *ctx, uint8_t rid,
                                     uint32_t value, uint32_t timeout_ms);

/**
 * @brief Write a float register with retry and ACK value verification
 *
 * Same as damiao_write_register_u32_retry() but for float values.
 * The float is transmitted as its little-endian uint32 bit representation.
 *
 * @param ctx        Mode switch context
 * @param rid        Register ID
 * @param value      float value to write
 * @param timeout_ms Per-attempt timeout in milliseconds
 * @return DAMIAO_ERR_OK on success, negative DamiaoError on failure
 */
int damiao_write_register_f32_retry(const DamiaoModeSwitchCtx *ctx, uint8_t rid,
                                     float value, uint32_t timeout_ms);

/*===========================================================================
 * Motor State Tracking
 *===========================================================================*/

/**
 * @brief Lightweight motor state for STM32
 *
 * Tracks the disabled hint (for set_zero guard), last feedback, and
 * motor configuration. Intended to be a static/global variable on STM32.
 */
typedef struct {
    uint16_t        motor_id;
    uint16_t        feedback_id;
    DamiaoLimits    limits;
    bool            disabled;       /**< true after disable, false after enable */
    bool            has_feedback;   /**< true once first feedback received */
    DamiaoFeedback  last_feedback;  /**< most recent decoded feedback */
} DamiaoMotorState;

/**
 * @brief Initialize motor state
 *
 * @param state       Motor state to initialize
 * @param motor_id    Motor CAN ID
 * @param feedback_id Motor feedback CAN ID
 * @param limits      Motor parameter limits
 */
void damiao_motor_state_init(DamiaoMotorState *state, uint16_t motor_id,
                              uint16_t feedback_id, const DamiaoLimits *limits);

/**
 * @brief Process an incoming CAN frame for this motor
 *
 * Call from your CAN RX callback. If the frame is a feedback frame matching
 * this motor's feedback_id, decodes and caches the feedback.
 *
 * @param state   Motor state
 * @param can_id  Received CAN arbitration ID
 * @param data    8-byte CAN data
 */
void damiao_motor_state_process_frame(DamiaoMotorState *state,
                                       uint16_t can_id, const uint8_t *data);

/*===========================================================================
 * Motor Control Functions with Safety Guards
 *===========================================================================*/

/**
 * @brief Enable motor (updates disabled hint)
 *
 * @param state Motor state
 * @param send  CAN send callback
 * @param ctx   User context
 * @return DAMIAO_ERR_OK on success
 */
int damiao_motor_enable(DamiaoMotorState *state, DamiaoSendFn send, void *ctx);

/**
 * @brief Disable motor (updates disabled hint)
 *
 * @param state Motor state (modified: disabled=true)
 * @param send  CAN send callback
 * @param ctx   User context
 * @return DAMIAO_ERR_OK on success
 */
int damiao_motor_disable(DamiaoMotorState *state, DamiaoSendFn send, void *ctx);

/**
 * @brief Set zero position (requires motor to be disabled first)
 *
 * Returns DAMIAO_ERR_NOT_DISABLED if the motor has not been disabled
 * via damiao_motor_disable() since the last damiao_motor_enable().
 *
 * @param state Motor state
 * @param send  CAN send callback
 * @param sleep Sleep callback (for post-command settle)
 * @param ctx   User context
 * @return DAMIAO_ERR_OK on success, DAMIAO_ERR_NOT_DISABLED if not disabled
 */
int damiao_motor_set_zero_position(DamiaoMotorState *state, DamiaoSendFn send,
                                    DamiaoSleepFn sleep, void *ctx);

/**
 * @brief Store parameters to flash (smart disable)
 *
 * If the motor is currently enabled, disables it first. Then sends the
 * store command. Does NOT re-enable the motor afterward.
 *
 * @param state Motor state
 * @param ctx   Mode switch context (provides callbacks)
 * @return DAMIAO_ERR_OK on success
 */
int damiao_motor_store_parameters(DamiaoMotorState *state,
                                   const DamiaoModeSwitchCtx *ctx);

/**
 * @brief Request feedback and wait for a fresh response
 *
 * Sends a feedback-request command (0xCC) and waits until a new feedback
 * frame arrives (or timeout). Updates state->last_feedback on success.
 *
 * @param state     Motor state (modified on success)
 * @param ctx       Mode switch context (provides callbacks)
 * @param timeout_ms Maximum wait time in milliseconds
 * @return DAMIAO_ERR_OK if fresh feedback received, DAMIAO_ERR_TIMEOUT on timeout
 */
int damiao_motor_request_feedback_state(DamiaoMotorState *state,
                                         const DamiaoModeSwitchCtx *ctx,
                                         uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* DAMIAO_PROTOCOL_H */
