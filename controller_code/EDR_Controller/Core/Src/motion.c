/* motion.c */
#include "motion.h"
#include "app_config.h"
#include "kinematics.h"
#include "trapezoid.h"
#include "odrive_link.h"
#include <math.h>
#include "app_log.h"

static float joint_deg_to_turns(axis_cfg_t cfg, float home_deg, float angle_deg)
{
    float normalized = angle_deg - home_deg;
    return cfg.offset_turns + cfg.direction * (normalized / 360.0f) * cfg.gear_ratio;
}

static float turns_to_joint_deg(axis_cfg_t cfg, float home_deg, float turns)
{
    float normalized = ((turns - cfg.offset_turns) / cfg.gear_ratio) * 360.0f / cfg.direction;
    return normalized + home_deg;
}

static sync_profile_t s_profile;
static float s_t1_start, s_t2_start;
static float s_dt;                 /* seconds per control tick */
static uint32_t s_step_index;
static uint32_t s_total_steps;
static axis_cfg_t s_axis0_cfg, s_axis1_cfg;

motion_err_t Motion_PrepareMove(void)
{
    s_axis0_cfg = AXIS0_CFG;
    s_axis1_cfg = AXIS1_CFG;

    char ping_reply[32];
    if (ODriveLink_Ping(ping_reply, sizeof(ping_reply))) {
        cdc_log("Ping OK, vbus_voltage = %s\r\n", ping_reply);
    } else {
        cdc_log("Ping FAILED - no reply from ODrive on UART5 at all.\r\n");
        return MOTION_ERR_CLOSED_LOOP_TIMEOUT;
    }

    float err0, err1, motor_err0, motor_err1, enc_err0, enc_err1;
    ODriveLink_ReadProperty("axis0.error", &err0);
    ODriveLink_ReadProperty("axis0.motor.error", &motor_err0);
    ODriveLink_ReadProperty("axis0.encoder.error", &enc_err0);
    ODriveLink_ReadProperty("axis1.error", &err1);
    ODriveLink_ReadProperty("axis1.motor.error", &motor_err1);
    ODriveLink_ReadProperty("axis1.encoder.error", &enc_err1);
    cdc_log("axis0: err=%.0f motor.err=%.0f enc.err=%.0f\r\n", err0, motor_err0, enc_err0);
    cdc_log("axis1: err=%.0f motor.err=%.0f enc.err=%.0f\r\n", err1, motor_err1, enc_err1);

    /* 1) request closed-loop control on both axes (mirrors enable_closed_loop_both) */
    ODriveLink_RequestClosedLoop(ODRIVE_AXIS0_IDX);
    ODriveLink_RequestClosedLoop(ODRIVE_AXIS1_IDX);

    uint32_t start = HAL_GetTick();
    bool axis0_ready = false, axis1_ready = false;
    while ((HAL_GetTick() - start) < 3000) {
        int st0, st1;
        bool got0 = ODriveLink_ReadAxisState(ODRIVE_AXIS0_IDX, &st0);
        bool got1 = ODriveLink_ReadAxisState(ODRIVE_AXIS1_IDX, &st1);
        cdc_log("poll: axis0 got=%d state=%d, axis1 got=%d state=%d\r\n",
                got0, got0 ? st0 : -1, got1, got1 ? st1 : -1);
        if (got0 && st0 == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) axis0_ready = true;
        if (got1 && st1 == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) axis1_ready = true;
        if (axis0_ready && axis1_ready) break;
        HAL_Delay(200);
    }
    if (!axis0_ready || !axis1_ready) return MOTION_ERR_CLOSED_LOOP_TIMEOUT;

    /* 2) read current encoder turns -> current joint angles */
    float turns0, turns1;
    if (!ODriveLink_ReadEncoderPos(ODRIVE_AXIS0_IDX, &turns0)) return MOTION_ERR_ENCODER_READ_FAILED;
    if (!ODriveLink_ReadEncoderPos(ODRIVE_AXIS1_IDX, &turns1)) return MOTION_ERR_ENCODER_READ_FAILED;

    s_t1_start = turns_to_joint_deg(s_axis0_cfg, HOME_ANGLE_DEG_AXIS0, turns0);
    s_t2_start = turns_to_joint_deg(s_axis1_cfg, HOME_ANGLE_DEG_AXIS1, turns1);

    /* 3) IK for the hardcoded target */
    float t1_target, t2_target;
    if (!inverse_kinematics(TARGET_X_MM, TARGET_Y_MM, &t1_target, &t2_target))
        return MOTION_ERR_TARGET_UNREACHABLE;

    /* 4) build synchronized trapezoidal profile (joint space) */
    build_sync_profile(t1_target - s_t1_start, t2_target - s_t2_start,
                        TRAJ_MAX_VEL_DEG_S, TRAJ_MAX_ACCEL_DEG_S2, &s_profile);

    s_dt = 1.0f / TRAJ_CONTROL_RATE_HZ;
    s_total_steps = (s_profile.T <= 0.0f) ? 0 : (uint32_t)ceilf(s_profile.T / s_dt);
    s_step_index = 0;

    return MOTION_ERR_NONE;
}

bool Motion_StreamTick(void)
{
    if (s_step_index > s_total_steps) return false; /* already done */

    float t_elapsed = s_step_index * s_dt;
    if (t_elapsed > s_profile.T) t_elapsed = s_profile.T;

    float t1 = s_t1_start + sync_profile_pos1(&s_profile, t_elapsed);
    float t2 = s_t2_start + sync_profile_pos2(&s_profile, t_elapsed);
    float w1_deg_s = sync_profile_vel1(&s_profile, t_elapsed);
    float w2_deg_s = sync_profile_vel2(&s_profile, t_elapsed);

    float turns0 = joint_deg_to_turns(s_axis0_cfg, HOME_ANGLE_DEG_AXIS0, t1);
    float turns1 = joint_deg_to_turns(s_axis1_cfg, HOME_ANGLE_DEG_AXIS1, t2);

    /* deg/s -> turns/s feed-forward, same conversion factor as position */
    float vel_ff0 = (s_axis0_cfg.direction / 360.0f) * s_axis0_cfg.gear_ratio * w1_deg_s;
    float vel_ff1 = (s_axis1_cfg.direction / 360.0f) * s_axis1_cfg.gear_ratio * w2_deg_s;

    ODriveLink_SetPosition(ODRIVE_AXIS0_IDX, turns0, vel_ff0);
    ODriveLink_SetPosition(ODRIVE_AXIS1_IDX, turns1, vel_ff1);

    s_step_index++;
    return (t_elapsed < s_profile.T) || (s_step_index <= s_total_steps);
}
