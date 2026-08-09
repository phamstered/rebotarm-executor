/**
 * @file damiao_protocol.c
 * @brief Damiao CAN Protocol Implementation for STM32
 *
 * This file implements the CAN protocol for Damiao motors,
 * matching the motorbridge Rust implementation exactly.
 *
 * @author motorbridge contributors
 * @version 1.0.0
 */

#include "damiao_protocol.h"
#include <string.h>

/*===========================================================================
 * Utility Functions
 *===========================================================================*/

uint32_t damiao_float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float clipped = x;

    /* Clamp to range */
    if (clipped < x_min) clipped = x_min;
    if (clipped > x_max) clipped = x_max;

    return (uint32_t)((clipped - x_min) * ((float)((1u << bits) - 1)) / span);
}

float damiao_uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    return (float)x * span / ((float)((1u << bits) - 1)) + x_min;
}

/*===========================================================================
 * MIT Command Encoding
 *===========================================================================*/

void damiao_encode_mit_cmd(float pos, float vel, float torq, float kp, float kd,
                           const DamiaoLimits *limits, uint8_t *data) {
    uint16_t pos_uint;
    uint16_t vel_uint;
    uint16_t kp_uint;
    uint16_t kd_uint;
    uint16_t torq_uint;

    /* Convert to unsigned integers */
    pos_uint = (uint16_t)damiao_float_to_uint(pos, limits->p_min, limits->p_max, 16);
    vel_uint = (uint16_t)damiao_float_to_uint(vel, limits->v_min, limits->v_max, 12);
    kp_uint  = (uint16_t)damiao_float_to_uint(kp, DAMIAO_KP_MIN, DAMIAO_KP_MAX, 12);
    kd_uint  = (uint16_t)damiao_float_to_uint(kd, DAMIAO_KD_MIN, DAMIAO_KD_MAX, 12);
    torq_uint = (uint16_t)damiao_float_to_uint(torq, limits->t_min, limits->t_max, 12);

    /* Pack into 8 bytes */
    data[0] = (pos_uint >> 8) & 0xFF;
    data[1] = pos_uint & 0xFF;
    data[2] = (vel_uint >> 4) & 0xFF;
    data[3] = ((vel_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F);
    data[4] = kp_uint & 0xFF;
    data[5] = (kd_uint >> 4) & 0xFF;
    data[6] = ((kd_uint & 0x0F) << 4) | ((torq_uint >> 8) & 0x0F);
    data[7] = torq_uint & 0xFF;
}

/*===========================================================================
 * POS_VEL Command Encoding
 *===========================================================================*/

void damiao_encode_pos_vel_cmd(float target_position, float velocity_limit,
                               uint8_t *data) {
    /* Copy as little-endian floats */
    memcpy(&data[0], &target_position, 4);
    memcpy(&data[4], &velocity_limit, 4);
}

/*===========================================================================
 * VEL Command Encoding
 *===========================================================================*/

void damiao_encode_vel_cmd(float target_velocity, uint8_t *data) {
    /* Copy as little-endian float */
    memcpy(&data[0], &target_velocity, 4);
    /* Remaining bytes are zero */
    memset(&data[4], 0, 4);
}

/*===========================================================================
 * FORCE_POS Command Encoding
 *===========================================================================*/

void damiao_encode_force_pos_cmd(float target_position, float velocity_limit,
                                 float torque_limit_ratio, uint8_t *data) {
    uint16_t v_des;
    uint16_t i_des;

    /* Copy position as little-endian float */
    memcpy(&data[0], &target_position, 4);

    /* Encode velocity limit (0-100 → 0-10000) */
    v_des = (uint16_t)(velocity_limit * 100.0f);
    if (v_des > 10000) v_des = 10000;
    data[4] = v_des & 0xFF;
    data[5] = (v_des >> 8) & 0xFF;

    /* Encode torque ratio (0-1 → 0-10000) */
    i_des = (uint16_t)(torque_limit_ratio * 10000.0f);
    if (i_des > 10000) i_des = 10000;
    data[6] = i_des & 0xFF;
    data[7] = (i_des >> 8) & 0xFF;
}

/*===========================================================================
 * Control Command Encoding
 *===========================================================================*/

void damiao_encode_enable_cmd(uint8_t *data) {
    memset(data, 0xFF, 7);
    data[7] = DAMIAO_CMD_ENABLE;
}

void damiao_encode_disable_cmd(uint8_t *data) {
    memset(data, 0xFF, 7);
    data[7] = DAMIAO_CMD_DISABLE;
}

void damiao_encode_set_zero_cmd(uint8_t *data) {
    memset(data, 0xFF, 7);
    data[7] = DAMIAO_CMD_SET_ZERO;
}

void damiao_encode_clear_error_cmd(uint8_t *data) {
    memset(data, 0xFF, 7);
    data[7] = DAMIAO_CMD_CLEAR_ERROR;
}

/*===========================================================================
 * Register Command Encoding
 *===========================================================================*/

void damiao_encode_register_read_cmd(uint16_t motor_id, uint8_t rid, uint8_t *data) {
    data[0] = motor_id & 0xFF;
    data[1] = (motor_id >> 8) & 0xFF;
    data[2] = DAMIAO_REG_READ_MARKER;
    data[3] = rid;
    memset(&data[4], 0, 4);
}

void damiao_encode_register_write_cmd(uint16_t motor_id, uint8_t rid,
                                      uint32_t value, uint8_t *data) {
    data[0] = motor_id & 0xFF;
    data[1] = (motor_id >> 8) & 0xFF;
    data[2] = DAMIAO_REG_WRITE_MARKER;
    data[3] = rid;
    memcpy(&data[4], &value, 4);  /* Little-endian */
}

void damiao_encode_store_params_cmd(uint16_t motor_id, uint8_t *data) {
    data[0] = motor_id & 0xFF;
    data[1] = (motor_id >> 8) & 0xFF;
    data[2] = DAMIAO_REG_STORE_MARKER;
    data[3] = 0x01;
    memset(&data[4], 0, 4);
}

void damiao_encode_feedback_request_cmd(uint16_t motor_id, uint8_t *data) {
    data[0] = motor_id & 0xFF;
    data[1] = (motor_id >> 8) & 0xFF;
    data[2] = DAMIAO_REG_FEEDBACK_REQ;
    memset(&data[3], 0, 5);
}

/*===========================================================================
 * Feedback Decoding
 *===========================================================================*/

void damiao_decode_sensor_feedback(const uint8_t *data, const DamiaoLimits *limits,
                                   DamiaoFeedback *fb) {
    uint16_t pos_int;
    uint16_t vel_int;
    uint16_t torq_int;

    /* Extract motor_id and status from byte 0 */
    fb->motor_id = data[0] & 0x0F;
    fb->status_code = (data[0] >> 4) & 0x0F;

    /* Extract packed position, velocity, torque */
    pos_int = ((uint16_t)data[1] << 8) | (uint16_t)data[2];
    vel_int = ((uint16_t)data[3] << 4) | ((uint16_t)data[4] >> 4);
    torq_int = (((uint16_t)data[4] & 0x0F) << 8) | (uint16_t)data[5];

    /* Convert to float */
    fb->pos = damiao_uint_to_float(pos_int, limits->p_min, limits->p_max, 16);
    fb->vel = damiao_uint_to_float(vel_int, limits->v_min, limits->v_max, 12);
    fb->torq = damiao_uint_to_float(torq_int, limits->t_min, limits->t_max, 12);

    /* Temperature (directly as Celsius) */
    fb->t_mos = (float)data[6];
    fb->t_rotor = (float)data[7];
}

/*===========================================================================
 * Register Reply Decoding
 *===========================================================================*/

static bool is_register_frame(const uint8_t *data, uint8_t marker) {
    /* Check: data[1] <= 0x0F, data[2] == marker, and we have a valid register */
    return (data[1] <= 0x0F) && (data[2] == marker);
}

bool damiao_is_register_reply(const uint8_t *data) {
    return is_register_frame(data, DAMIAO_REG_READ_MARKER);
}

bool damiao_is_register_write_ack(const uint8_t *data) {
    return is_register_frame(data, DAMIAO_REG_WRITE_MARKER);
}

bool damiao_decode_register_value(const uint8_t *data, DamiaoRegisterValue *rv) {
    if (!damiao_is_register_reply(data)) {
        return false;
    }

    rv->rid = data[3];
    rv->data[0] = data[4];
    rv->data[1] = data[5];
    rv->data[2] = data[6];
    rv->data[3] = data[7];

    return true;
}

/*===========================================================================
 * Status Code Helpers
 *===========================================================================*/

const char *damiao_status_name(uint8_t status_code) {
    switch (status_code) {
        case DAMIAO_STATUS_DISABLED:      return "DISABLED";
        case DAMIAO_STATUS_ENABLED:       return "ENABLED";
        case DAMIAO_STATUS_OVER_VOLTAGE:  return "OVER_VOLTAGE";
        case DAMIAO_STATUS_UNDER_VOLTAGE: return "UNDER_VOLTAGE";
        case DAMIAO_STATUS_OVER_CURRENT:  return "OVER_CURRENT";
        case DAMIAO_STATUS_MOS_OVER_TEMP: return "MOS_OVER_TEMP";
        case DAMIAO_STATUS_ROTOR_OVER_TEMP: return "ROTOR_OVER_TEMP";
        case DAMIAO_STATUS_LOST_COMM:     return "LOST_COMM";
        case DAMIAO_STATUS_OVERLOAD:      return "OVERLOAD";
        default:                          return "UNKNOWN";
    }
}

/*===========================================================================
 * Register Read/Write Helpers
 *===========================================================================*/

int damiao_read_register_u32(const DamiaoModeSwitchCtx *ctx, uint8_t rid,
                             uint32_t *value, uint32_t timeout_ms) {
    uint8_t cmd[8];
    uint16_t recv_id;
    uint8_t recv_data[8];
    DamiaoRegisterValue rv;

    if (!ctx || !ctx->send || !ctx->recv || !value) {
        return DAMIAO_ERR_INVALID_ARG;
    }

    /* Send register read request */
    damiao_encode_register_read_cmd(ctx->motor_id, rid, cmd);
    if (ctx->send(damiao_get_register_can_id(), cmd, ctx->user_ctx) != 0) {
        return DAMIAO_ERR_CAN_WRITE;
    }

    /* Feedback and register replies share the RX queue. Skip unrelated frames
     * until the requested RID arrives or the timeout budget is exhausted. */
    uint32_t elapsed = 0U;
    while (elapsed < timeout_ms) {
        uint32_t wait_ms = timeout_ms - elapsed;
        if (wait_ms > 5U) wait_ms = 5U;
        if (ctx->recv(&recv_id, recv_data, wait_ms, ctx->user_ctx) == 0 &&
            damiao_decode_register_value(recv_data, &rv) && rv.rid == rid) {
            *value = damiao_decode_register_u32(rv.data);
            return DAMIAO_ERR_OK;
        }
        elapsed += wait_ms;
    }
    return DAMIAO_ERR_TIMEOUT;
}

int damiao_write_register_u32(const DamiaoModeSwitchCtx *ctx, uint8_t rid,
                              uint32_t value, uint32_t timeout_ms) {
    uint8_t cmd[8];
    uint16_t recv_id;
    uint8_t recv_data[8];

    if (!ctx || !ctx->send || !ctx->recv) {
        return DAMIAO_ERR_INVALID_ARG;
    }

    /* Send register write command */
    damiao_encode_register_write_cmd(ctx->motor_id, rid, value, cmd);
    if (ctx->send(damiao_get_register_can_id(), cmd, ctx->user_ctx) != 0) {
        return DAMIAO_ERR_CAN_WRITE;
    }

    uint32_t elapsed = 0U;
    while (elapsed < timeout_ms) {
        uint32_t wait_ms = timeout_ms - elapsed;
        if (wait_ms > 5U) wait_ms = 5U;
        if (ctx->recv(&recv_id, recv_data, wait_ms, ctx->user_ctx) == 0 &&
            damiao_is_register_write_ack(recv_data) && recv_data[3] == rid) {
            return DAMIAO_ERR_OK;
        }
        elapsed += wait_ms;
    }
    return DAMIAO_ERR_TIMEOUT;
}

/*===========================================================================
 * Control Mode Switching
 *===========================================================================*/

/**
 * @brief Try to read current motor position from register 80
 *
 * @param ctx      Mode switch context
 * @param pos_out  Output: position in radians
 * @return DAMIAO_ERR_OK on success, negative on failure (non-fatal)
 */
static int try_read_position(const DamiaoModeSwitchCtx *ctx, float *pos_out) {
    uint32_t raw;
    int rc = damiao_read_register_u32(ctx, DAMIAO_REG_MOTOR_POS, &raw, 10U);
    if (rc == DAMIAO_ERR_OK) {
        memcpy(pos_out, &raw, sizeof(raw));
    }
    return rc;
}

/**
 * @brief Clear all command channels and send initial hold command
 *
 * Before switching modes, the motor docs require zeroing position, velocity,
 * torque feedforward, kp, and kd across all command channels.
 *
 * After the mode switch, sends the appropriate initial command for the new mode.
 */
static void clear_channels_and_send_initial(const DamiaoModeSwitchCtx *ctx,
                                            DamiaoControlMode mode,
                                            float hold_position) {
    uint8_t data[8];
    DamiaoLimits zero_limits = {0};

    /* Clear all command channels to safe defaults */
    damiao_encode_vel_cmd(0.0f, data);
    ctx->send(damiao_get_vel_can_id(ctx->motor_id), data, ctx->user_ctx);

    damiao_encode_pos_vel_cmd(hold_position, 0.0f, data);
    ctx->send(damiao_get_pos_vel_can_id(ctx->motor_id), data, ctx->user_ctx);

    damiao_encode_force_pos_cmd(hold_position, 0.0f, 0.0f, data);
    ctx->send(damiao_get_force_pos_can_id(ctx->motor_id), data, ctx->user_ctx);

    /* Send initial command for the target mode */
    switch (mode) {
        case DAMIAO_MODE_MIT: {
            const DamiaoLimits *lim = ctx->limits ? ctx->limits : &zero_limits;
            damiao_encode_mit_cmd(hold_position, 0.0f, 0.0f, 0.0f, 0.0f, lim, data);
            ctx->send(damiao_get_mit_can_id(ctx->motor_id), data, ctx->user_ctx);
            break;
        }
        case DAMIAO_MODE_POS_VEL:
            damiao_encode_pos_vel_cmd(hold_position, 0.0f, data);
            ctx->send(damiao_get_pos_vel_can_id(ctx->motor_id), data, ctx->user_ctx);
            break;
        case DAMIAO_MODE_VEL:
            damiao_encode_vel_cmd(0.0f, data);
            ctx->send(damiao_get_vel_can_id(ctx->motor_id), data, ctx->user_ctx);
            break;
        case DAMIAO_MODE_FORCE_POS:
            damiao_encode_force_pos_cmd(hold_position, 0.0f, 0.0f, data);
            ctx->send(damiao_get_force_pos_can_id(ctx->motor_id), data, ctx->user_ctx);
            break;
    }
}

/* Mode switch constants are defined in damiao_protocol.h */

int damiao_ensure_control_mode(const DamiaoModeSwitchCtx *ctx,
                               DamiaoControlMode mode,
                               uint32_t timeout_ms) {
    uint32_t current_mode;
    float hold_position = 0.0f;
    int rc;
    uint32_t remaining;

    if (!ctx || !ctx->send || !ctx->recv || !ctx->sleep) {
        return DAMIAO_ERR_INVALID_ARG;
    }
    if (mode < DAMIAO_MODE_MIT || mode > DAMIAO_MODE_FORCE_POS) {
        return DAMIAO_ERR_INVALID_ARG;
    }

    /* Step 1: Read current control mode */
    rc = damiao_read_register_u32(ctx, DAMIAO_REG_CTRL_MODE, &current_mode,
                                  DAMIAO_ENSURE_MODE_READ_TIMEOUT_MS);
    if (rc != DAMIAO_ERR_OK) {
        return rc;
    }

    /* Already in desired mode? */
    if (current_mode == (uint32_t)mode) {
        return DAMIAO_ERR_OK;
    }

    /* Step 2: Prepare for mode switch — cache position and clear channels */
    if (mode == DAMIAO_MODE_MIT || mode == DAMIAO_MODE_POS_VEL ||
        mode == DAMIAO_MODE_FORCE_POS) {
        /* Best-effort read; if it fails, hold at 0.0 */
        try_read_position(ctx, &hold_position);
    }

    /* Clear all command channels */
    {
        uint8_t data[8];
        damiao_encode_vel_cmd(0.0f, data);
        ctx->send(damiao_get_vel_can_id(ctx->motor_id), data, ctx->user_ctx);

        damiao_encode_pos_vel_cmd(hold_position, 0.0f, data);
        ctx->send(damiao_get_pos_vel_can_id(ctx->motor_id), data, ctx->user_ctx);

        damiao_encode_force_pos_cmd(hold_position, 0.0f, 0.0f, data);
        ctx->send(damiao_get_force_pos_can_id(ctx->motor_id), data, ctx->user_ctx);

        {
            const DamiaoLimits *lim = ctx->limits ? ctx->limits : &(DamiaoLimits){0};
            damiao_encode_mit_cmd(hold_position, 0.0f, 0.0f, 0.0f, 0.0f, lim, data);
            ctx->send(damiao_get_mit_can_id(ctx->motor_id), data, ctx->user_ctx);
        }
    }

    /* Step 3: Write control mode register */
    rc = damiao_write_register_u32(ctx, DAMIAO_REG_CTRL_MODE, (uint32_t)mode,
                                   DAMIAO_ENSURE_MODE_READ_TIMEOUT_MS);
    if (rc != DAMIAO_ERR_OK) {
        return rc;
    }

    /* Step 4: Wait for motor to settle */
    ctx->sleep(DAMIAO_ENSURE_MODE_SETTLE_MS, ctx->user_ctx);

    /* Step 5: Send initial command for the new mode */
    clear_channels_and_send_initial(ctx, mode, hold_position);

    /* Step 6: Verify mode (with retries) */
    remaining = timeout_ms > DAMIAO_ENSURE_MODE_SETTLE_MS + DAMIAO_ENSURE_MODE_READ_TIMEOUT_MS
              ? timeout_ms - DAMIAO_ENSURE_MODE_SETTLE_MS - DAMIAO_ENSURE_MODE_READ_TIMEOUT_MS
              : 0;

    for (int attempt = 0; attempt < DAMIAO_ENSURE_MODE_VERIFY_ATTEMPTS; attempt++) {
        uint32_t verify_timeout = remaining < DAMIAO_ENSURE_MODE_READ_TIMEOUT_MS
                                ? remaining : DAMIAO_ENSURE_MODE_READ_TIMEOUT_MS;
        if (verify_timeout == 0) {
            break;
        }

        rc = damiao_read_register_u32(ctx, DAMIAO_REG_CTRL_MODE, &current_mode,
                                      verify_timeout);
        if (rc == DAMIAO_ERR_OK && current_mode == (uint32_t)mode) {
            return DAMIAO_ERR_OK;
        }

        /* Retry: re-write mode register and re-send initial command */
        if (attempt + 1 < DAMIAO_ENSURE_MODE_VERIFY_ATTEMPTS) {
            uint32_t gap = remaining < DAMIAO_ENSURE_MODE_VERIFY_RETRY_GAP_MS
                         ? remaining : DAMIAO_ENSURE_MODE_VERIFY_RETRY_GAP_MS;
            damiao_write_register_u32(ctx, DAMIAO_REG_CTRL_MODE, (uint32_t)mode,
                                      DAMIAO_ENSURE_MODE_READ_TIMEOUT_MS);
            ctx->sleep(gap, ctx->user_ctx);
            clear_channels_and_send_initial(ctx, mode, hold_position);
            if (remaining > gap) {
                remaining -= gap;
            } else {
                remaining = 0;
            }
        }
    }

    return DAMIAO_ERR_MODE_VERIFY;
}

/*===========================================================================
 * Register Table
 *===========================================================================*/

static const DamiaoRegisterInfo DAMIAO_REGISTER_TABLE[] = {
    /* RW registers */
    { 0,  DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* UV_Value */
    { 1,  DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* KT_Value */
    { 2,  DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* OT_Value */
    { 3,  DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* OC_Value */
    { 4,  DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* ACC */
    { 5,  DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* DEC */
    { 6,  DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* MAX_SPD */
    { 7,  DAMIAO_DATA_UINT32, DAMIAO_ACCESS_RW },  /* MST_ID */
    { 8,  DAMIAO_DATA_UINT32, DAMIAO_ACCESS_RW },  /* ESC_ID */
    { 9,  DAMIAO_DATA_UINT32, DAMIAO_ACCESS_RW },  /* TIMEOUT */
    { 10, DAMIAO_DATA_UINT32, DAMIAO_ACCESS_RW },  /* CTRL_MODE */
    { 21, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* PMAX */
    { 22, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* VMAX */
    { 23, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* TMAX */
    { 24, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* I_BW */
    { 25, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* KP_ASR */
    { 26, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* KI_ASR */
    { 27, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* KP_APR */
    { 28, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* KI_APR */
    { 29, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* OV_Value */
    { 30, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* GREF */
    { 31, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* Deta */
    { 32, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* V_BW */
    { 33, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* IQ_c1 */
    { 34, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RW },  /* VL_c1 */
    { 35, DAMIAO_DATA_UINT32, DAMIAO_ACCESS_RW },  /* can_br */
    /* RO registers (for completeness — not writable) */
    { 11, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Damp */
    { 12, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Inertia */
    { 13, DAMIAO_DATA_UINT32, DAMIAO_ACCESS_RO },  /* hw_ver */
    { 14, DAMIAO_DATA_UINT32, DAMIAO_ACCESS_RO },  /* sw_ver */
    { 15, DAMIAO_DATA_UINT32, DAMIAO_ACCESS_RO },  /* SN */
    { 16, DAMIAO_DATA_UINT32, DAMIAO_ACCESS_RO },  /* NPP */
    { 17, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Rs */
    { 18, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Ls */
    { 19, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Flux */
    { 20, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Gr */
    { 36, DAMIAO_DATA_UINT32, DAMIAO_ACCESS_RO },  /* sub_ver */
    { 50, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* u_off */
    { 51, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* v_off */
    { 52, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* k1 */
    { 53, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* k2 */
    { 54, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* m_off */
    { 55, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* dir */
    { 56, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* m_off (motor side) */
    { 59, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Imax */
    { 60, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* VBus */
    { 61, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Tpcb */
    { 62, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Tmtr */
    { 63, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Iu_off */
    { 64, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Iv_off */
    { 65, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* Iw_off */
    { 80, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* p_m */
    { 81, DAMIAO_DATA_FLOAT,  DAMIAO_ACCESS_RO },  /* xout */
};

#define DAMIAO_REGISTER_TABLE_SIZE \
    (sizeof(DAMIAO_REGISTER_TABLE) / sizeof(DAMIAO_REGISTER_TABLE[0]))

const DamiaoRegisterInfo *damiao_register_info(uint8_t rid) {
    size_t i;
    for (i = 0; i < DAMIAO_REGISTER_TABLE_SIZE; i++) {
        if (DAMIAO_REGISTER_TABLE[i].rid == rid) {
            return &DAMIAO_REGISTER_TABLE[i];
        }
    }
    return NULL;
}

int damiao_validate_register_write(uint8_t rid, DamiaoDataType expected_type) {
    const DamiaoRegisterInfo *info = damiao_register_info(rid);
    if (!info) {
        return DAMIAO_ERR_INVALID_ARG;
    }
    if (info->access != DAMIAO_ACCESS_RW) {
        return DAMIAO_ERR_REGISTER_RO;
    }
    if (info->data_type != expected_type) {
        return DAMIAO_ERR_TYPE_MISMATCH;
    }
    return DAMIAO_ERR_OK;
}

/*===========================================================================
 * Write Register with Retry & ACK Validation
 *===========================================================================*/

/**
 * @brief Internal: write uint32 register once and wait for ACK with value check
 */
static int write_register_u32_once_and_verify(const DamiaoModeSwitchCtx *ctx,
                                               uint8_t rid, uint32_t value,
                                               uint32_t timeout_ms) {
    uint8_t cmd[8];
    uint16_t recv_id;
    uint8_t recv_data[8];

    /* Send register write command */
    damiao_encode_register_write_cmd(ctx->motor_id, rid, value, cmd);
    if (ctx->send(damiao_get_register_can_id(), cmd, ctx->user_ctx) != 0) {
        return DAMIAO_ERR_CAN_WRITE;
    }

    uint32_t elapsed = 0U;
    while (elapsed < timeout_ms) {
        uint32_t wait_ms = timeout_ms - elapsed;
        if (wait_ms > 5U) wait_ms = 5U;
        if (ctx->recv(&recv_id, recv_data, wait_ms, ctx->user_ctx) == 0 &&
            damiao_is_register_write_ack(recv_data) && recv_data[3] == rid) {
            uint32_t ack_value = (uint32_t)recv_data[4]
                               | ((uint32_t)recv_data[5] << 8)
                               | ((uint32_t)recv_data[6] << 16)
                               | ((uint32_t)recv_data[7] << 24);
            return (ack_value == value) ? DAMIAO_ERR_OK : DAMIAO_ERR_CAN_READ;
        }
        elapsed += wait_ms;
    }
    return DAMIAO_ERR_TIMEOUT;
}

int damiao_write_register_u32_retry(const DamiaoModeSwitchCtx *ctx, uint8_t rid,
                                     uint32_t value, uint32_t timeout_ms) {
    int rc;
    int attempt;

    if (!ctx || !ctx->send || !ctx->recv) {
        return DAMIAO_ERR_INVALID_ARG;
    }

    /* Validate register before attempting write */
    rc = damiao_validate_register_write(rid, DAMIAO_DATA_UINT32);
    if (rc != DAMIAO_ERR_OK) {
        return rc;
    }

    for (attempt = 0; attempt < DAMIAO_WRITE_ACK_ATTEMPTS; attempt++) {
        rc = write_register_u32_once_and_verify(ctx, rid, value, timeout_ms);
        if (rc == DAMIAO_ERR_OK) {
            return DAMIAO_ERR_OK;
        }
        /* Retry gap (except after last attempt) */
        if (attempt + 1 < DAMIAO_WRITE_ACK_ATTEMPTS && ctx->sleep) {
            ctx->sleep(DAMIAO_WRITE_ACK_RETRY_GAP_MS, ctx->user_ctx);
        }
    }

    return rc; /* Return last error */
}

int damiao_write_register_f32_retry(const DamiaoModeSwitchCtx *ctx, uint8_t rid,
                                     float value, uint32_t timeout_ms) {
    uint32_t raw;
    int rc;
    int attempt;

    if (!ctx || !ctx->send || !ctx->recv) {
        return DAMIAO_ERR_INVALID_ARG;
    }

    /* Validate register before attempting write */
    rc = damiao_validate_register_write(rid, DAMIAO_DATA_FLOAT);
    if (rc != DAMIAO_ERR_OK) {
        return rc;
    }

    /* Reinterpret float bits as uint32 for the raw CAN frame */
    memcpy(&raw, &value, sizeof(raw));

    for (attempt = 0; attempt < DAMIAO_WRITE_ACK_ATTEMPTS; attempt++) {
        rc = write_register_u32_once_and_verify(ctx, rid, raw, timeout_ms);
        if (rc == DAMIAO_ERR_OK) {
            return DAMIAO_ERR_OK;
        }
        if (attempt + 1 < DAMIAO_WRITE_ACK_ATTEMPTS && ctx->sleep) {
            ctx->sleep(DAMIAO_WRITE_ACK_RETRY_GAP_MS, ctx->user_ctx);
        }
    }

    return rc;
}

/*===========================================================================
 * Motor State Tracking
 *===========================================================================*/

void damiao_motor_state_init(DamiaoMotorState *state, uint16_t motor_id,
                              uint16_t feedback_id, const DamiaoLimits *limits) {
    if (!state) return;
    state->motor_id = motor_id;
    state->feedback_id = feedback_id;
    state->disabled = true;  /* Start in disabled state */
    state->has_feedback = false;
    memset(&state->last_feedback, 0, sizeof(state->last_feedback));
    if (limits) {
        state->limits = *limits;
    } else {
        memset(&state->limits, 0, sizeof(state->limits));
    }
}

void damiao_motor_state_process_frame(DamiaoMotorState *state,
                                       uint16_t can_id, const uint8_t *data) {
    if (!state || !data) return;

    /* Match by feedback_id */
    if (can_id != state->feedback_id) return;

    /* Decode sensor feedback */
    damiao_decode_sensor_feedback(data, &state->limits, &state->last_feedback);
    state->has_feedback = true;
}

/*===========================================================================
 * Motor Control Functions with Safety Guards
 *===========================================================================*/

int damiao_motor_enable(DamiaoMotorState *state, DamiaoSendFn send, void *ctx) {
    uint8_t data[8];
    int rc;

    if (!state || !send) return DAMIAO_ERR_INVALID_ARG;

    damiao_encode_enable_cmd(data);
    rc = send(state->motor_id, data, ctx);
    if (rc != 0) return DAMIAO_ERR_CAN_WRITE;

    state->disabled = false;
    return DAMIAO_ERR_OK;
}

int damiao_motor_disable(DamiaoMotorState *state, DamiaoSendFn send, void *ctx) {
    uint8_t data[8];
    int rc;

    if (!state || !send) return DAMIAO_ERR_INVALID_ARG;

    damiao_encode_disable_cmd(data);
    rc = send(state->motor_id, data, ctx);
    if (rc != 0) return DAMIAO_ERR_CAN_WRITE;

    state->disabled = true;
    return DAMIAO_ERR_OK;
}

int damiao_motor_set_zero_position(DamiaoMotorState *state, DamiaoSendFn send,
                                    DamiaoSleepFn sleep, void *ctx) {
    uint8_t data[8];
    int rc;

    if (!state || !send) return DAMIAO_ERR_INVALID_ARG;

    /* Guard: motor must be disabled first */
    if (!state->disabled) {
        return DAMIAO_ERR_NOT_DISABLED;
    }

    damiao_encode_set_zero_cmd(data);
    rc = send(state->motor_id, data, ctx);
    if (rc != 0) return DAMIAO_ERR_CAN_WRITE;

    /* Settle wait (matches Rust SET_ZERO_SETTLE_MS = 20ms) */
    if (sleep) {
        sleep(20, ctx);
    }

    return DAMIAO_ERR_OK;
}

int damiao_motor_store_parameters(DamiaoMotorState *state,
                                   const DamiaoModeSwitchCtx *ctx) {
    uint8_t cmd[8];
    int rc;

    if (!state || !ctx || !ctx->send) return DAMIAO_ERR_INVALID_ARG;

    /* If currently enabled, disable first */
    if (!state->disabled) {
        rc = damiao_motor_disable(state, ctx->send, ctx->user_ctx);
        if (rc != DAMIAO_ERR_OK) return rc;
        /* Settle after disable (matches Rust STORE_PARAMS_DISABLE_SETTLE_MS = 20ms) */
        if (ctx->sleep) {
            ctx->sleep(20, ctx->user_ctx);
        }
    }

    /* Send store command */
    damiao_encode_store_params_cmd(state->motor_id, cmd);
    rc = ctx->send(damiao_get_register_can_id(), cmd, ctx->user_ctx);
    if (rc != 0) return DAMIAO_ERR_CAN_WRITE;

    /* Post-store settle (matches Rust STORE_PARAMS_SETTLE_MS = 30ms) */
    if (ctx->sleep) {
        ctx->sleep(30, ctx->user_ctx);
    }

    return DAMIAO_ERR_OK;
}

int damiao_motor_request_feedback_state(DamiaoMotorState *state,
                                         const DamiaoModeSwitchCtx *ctx,
                                         uint32_t timeout_ms) {
    uint8_t cmd[8];
    uint16_t recv_id;
    uint8_t recv_data[8];
    uint32_t elapsed = 0;
    uint32_t poll_interval = 2; /* ms per poll iteration */

    if (!state || !ctx || !ctx->send || !ctx->recv) {
        return DAMIAO_ERR_INVALID_ARG;
    }

    /* Send feedback request */
    damiao_encode_feedback_request_cmd(state->motor_id, cmd);
    if (ctx->send(damiao_get_register_can_id(), cmd, ctx->user_ctx) != 0) {
        return DAMIAO_ERR_CAN_WRITE;
    }

    /* Poll for feedback frame */
    while (elapsed < timeout_ms) {
        uint32_t wait = timeout_ms - elapsed;
        if (wait > poll_interval) wait = poll_interval;

        if (ctx->recv(&recv_id, recv_data, wait, ctx->user_ctx) == 0) {
            /* Check if this is a feedback frame for our motor */
            if (recv_id == state->feedback_id) {
                damiao_decode_sensor_feedback(recv_data, &state->limits,
                                              &state->last_feedback);
                state->has_feedback = true;
                return DAMIAO_ERR_OK;
            }
        }
        elapsed += poll_interval;
    }

    return DAMIAO_ERR_TIMEOUT;
}
