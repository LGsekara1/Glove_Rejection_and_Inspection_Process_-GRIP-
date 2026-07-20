/* motion.c */
#include "motion.h"
#include "app_config.h"
#include "kinematics.h"
#include "trapezoid.h"
#include "odrive_link.h"
#include "app_log.h"
#include <math.h>

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

/* ---------------- move queue ----------------
 * Assumes Motion_MoveTo() and Motion_Update() are both called from the
 * same context (the main loop) - true in this project (Update() is
 * called from the main while(1) in response to a flag set by the TIM6
 * ISR, not from the ISR itself). If you ever call Motion_MoveTo() from
 * an interrupt, this needs a critical section around the push/pop.
 */
#define MOTION_QUEUE_SIZE 16
typedef struct { float x, y; } move_target_t;

static move_target_t s_queue[MOTION_QUEUE_SIZE];
static uint8_t s_queue_head = 0;
static uint8_t s_queue_tail = 0;
static uint8_t s_queue_count = 0;

static bool queue_push(float x, float y)
{
    if (s_queue_count >= MOTION_QUEUE_SIZE) return false;
    s_queue[s_queue_tail].x = x;
    s_queue[s_queue_tail].y = y;
    s_queue_tail = (uint8_t)((s_queue_tail + 1) % MOTION_QUEUE_SIZE);
    s_queue_count++;
    return true;
}

static bool queue_pop(move_target_t *out)
{
    if (s_queue_count == 0) return false;
    *out = s_queue[s_queue_head];
    s_queue_head = (uint8_t)((s_queue_head + 1) % MOTION_QUEUE_SIZE);
    s_queue_count--;
    return true;
}

/* ---------------- streaming state ---------------- */
static axis_cfg_t s_axis0_cfg, s_axis1_cfg;
static sync_profile_t s_profile;

static float s_t1_now, s_t2_now;               /* last known/settled joint pose (deg) */
static float s_move_t1_start, s_move_t2_start;  /* pose at the start of the active move */
static float s_move_t1_target, s_move_t2_target;/* target of the active move */

static float s_dt;
static uint32_t s_step_index;
static uint32_t s_total_steps;
static bool s_streaming = false;
static bool s_initialized = false;

motion_init_err_t Motion_Init(void)
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

    float turns0, turns1;
    if (!ODriveLink_ReadEncoderPos(ODRIVE_AXIS0_IDX, &turns0)) return MOTION_ERR_ENCODER_READ_FAILED;
    if (!ODriveLink_ReadEncoderPos(ODRIVE_AXIS1_IDX, &turns1)) return MOTION_ERR_ENCODER_READ_FAILED;

    s_t1_now = turns_to_joint_deg(s_axis0_cfg, HOME_ANGLE_DEG_AXIS0, turns0);
    s_t2_now = turns_to_joint_deg(s_axis1_cfg, HOME_ANGLE_DEG_AXIS1, turns1);

    s_dt = 1.0f / TRAJ_CONTROL_RATE_HZ;
    s_streaming = false;
    s_queue_head = s_queue_tail = s_queue_count = 0;
    s_initialized = true;

    cdc_log("Motion_Init OK. start pose t1=%.2f t2=%.2f deg\r\n", (double)s_t1_now, (double)s_t2_now);
    return MOTION_ERR_NONE;
}

bool Motion_MoveTo(float x_mm, float y_mm)
{
    if (!s_initialized) {
        cdc_log("Motion_MoveTo: not initialized - call Motion_Init() first.\r\n");
        return false;
    }
    /* fail fast on an unreachable target instead of silently queueing it */
    float t1_test, t2_test;
    if (!inverse_kinematics(x_mm, y_mm, &t1_test, &t2_test)) {
        cdc_log("Motion_MoveTo(%.2f, %.2f): unreachable, rejected.\r\n", (double)x_mm, (double)y_mm);
        return false;
    }
    if (!queue_push(x_mm, y_mm)) {
        cdc_log("Motion_MoveTo(%.2f, %.2f): queue full (%d), rejected.\r\n",
                (double)x_mm, (double)y_mm, (int)MOTION_QUEUE_SIZE);
        return false;
    }
    return true;
}

static void start_next_move_if_idle(void)
{
    if (s_streaming) return;

    move_target_t tgt;
    if (!queue_pop(&tgt)) return; /* nothing queued, stay idle */

    float t1_target, t2_target;
    if (!inverse_kinematics(tgt.x, tgt.y, &t1_target, &t2_target)) {
        /* Motion_MoveTo() already checked this, so this only fires if
           something changed the geometry mid-flight - skip and try the
           next queued item instead of getting stuck. */
        cdc_log("Motion_Update: queued target (%.2f, %.2f) is now unreachable, skipping.\r\n",
                (double)tgt.x, (double)tgt.y);
        start_next_move_if_idle();
        return;
    }

    s_move_t1_start = s_t1_now;
    s_move_t2_start = s_t2_now;
    s_move_t1_target = t1_target;
    s_move_t2_target = t2_target;


build_sync_profile(t1_target - s_move_t1_start, t2_target - s_move_t2_start,
                        TRAJ_MAX_VEL_DEG_S, TRAJ_MAX_ACCEL_DEG_S2,
                        TRAJ_MOTION_PROFILE, &s_profile);

    s_total_steps = (s_profile.T <= 0.0f) ? 0 : (uint32_t)ceilf(s_profile.T / s_dt);
    s_step_index = 0;

    if (s_total_steps == 0) {
        /* already at this target - snap state and go straight to the
           next queued move rather than burning a tick doing nothing */
        s_t1_now = t1_target;
        s_t2_now = t2_target;
        start_next_move_if_idle();
        return;
    }

    s_streaming = true;
    cdc_log("Motion: -> (%.2f, %.2f)  T=%.3fs  [queue depth now %d]\r\n",
            (double)tgt.x, (double)tgt.y, (double)s_profile.T, (int)s_queue_count);
}

void Motion_Update(void)
{
    if (!s_initialized) return;

    if (!s_streaming) {
        start_next_move_if_idle();
        if (!s_streaming) return; /* queue empty, nothing to do */
    }

    float t_elapsed = s_step_index * s_dt;
    if (t_elapsed > s_profile.T) t_elapsed = s_profile.T;

    float t1 = s_move_t1_start + sync_profile_pos1(&s_profile, t_elapsed);
    float t2 = s_move_t2_start + sync_profile_pos2(&s_profile, t_elapsed);
    float w1_deg_s = sync_profile_vel1(&s_profile, t_elapsed);
    float w2_deg_s = sync_profile_vel2(&s_profile, t_elapsed);

    float turns0 = joint_deg_to_turns(s_axis0_cfg, HOME_ANGLE_DEG_AXIS0, t1);
    float turns1 = joint_deg_to_turns(s_axis1_cfg, HOME_ANGLE_DEG_AXIS1, t2);
    float vel_ff0 = (s_axis0_cfg.direction / 360.0f) * s_axis0_cfg.gear_ratio * w1_deg_s;
    float vel_ff1 = (s_axis1_cfg.direction / 360.0f) * s_axis1_cfg.gear_ratio * w2_deg_s;

    ODriveLink_SetPosition(ODRIVE_AXIS0_IDX, turns0, vel_ff0);
    ODriveLink_SetPosition(ODRIVE_AXIS1_IDX, turns1, vel_ff1);

    s_step_index++;

    if (t_elapsed >= s_profile.T) {
        /* Move finished exactly on target. Snap the known pose and drop
           back to idle - the VERY NEXT call to Motion_Update() will pick
           up the next queued move immediately (no wasted tick). */
        s_t1_now = s_move_t1_target;
        s_t2_now = s_move_t2_target;
        s_streaming = false;
    }
}

bool Motion_IsMoving(void)
{
    return s_streaming || (s_queue_count > 0);
}

uint8_t Motion_QueueDepth(void)
{
    return s_queue_count;
}

void Motion_ClearQueue(void)
{
    s_queue_head = s_queue_tail = s_queue_count = 0;
}
