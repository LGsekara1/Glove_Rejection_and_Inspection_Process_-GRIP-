/* app_config.h */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ---------------- Hardcoded move target (mm) ---------------- */
/* EDIT THESE to your real pick point. */
#define TARGET_X_MM     0.0f
#define TARGET_Y_MM     500.0f

/* ---------------- Linkage geometry (mm) ---------------------- */
#define GEOM_L0   300.0f
#define GEOM_L1A  300.0f
#define GEOM_L2A  450.0f
#define GEOM_L1B  300.0f
#define GEOM_L2B  450.0f

typedef enum { ELBOW_UP, ELBOW_DOWN } elbow_t;
#define GEOM_ELBOW1  ELBOW_UP     /* axis0 arm, matches params.elbow1 = "up"   */
#define GEOM_ELBOW2  ELBOW_DOWN   /* axis1 arm, matches params.elbow2 = "down" */
#define FK_BRANCH_UPPER  1        /* matches params.fk_branch = "upper" */

/* ---------------- Motor <-> joint conversion ------------------ */
typedef struct {
    float gear_ratio;
    float offset_turns;
    float direction;   /* +1.0 or -1.0 */
} axis_cfg_t;

#define AXIS0_CFG  ((axis_cfg_t){ .gear_ratio = 1.0f, .offset_turns = 0.0f, .direction = 1.0f })
#define AXIS1_CFG  ((axis_cfg_t){ .gear_ratio = 1.0f, .offset_turns = 0.0f, .direction = 1.0f })

/* home_angle_deg: joint angle (deg, from +X axis) that corresponds to
   "Sync Now" reference. From your saved dashboard config. */
#define HOME_ANGLE_DEG_AXIS0   222.57888793945312f
#define HOME_ANGLE_DEG_AXIS1   64.32117462158203f

/* ---------------- Trajectory limits (joint space) -------------- */
#include "trapezoid.h"   /* for motion_profile_type_t */

/* PROFILE_SCURVE (recommended, matches the dashboard's new default) or
   PROFILE_TRAPEZOID (legacy, reaches speed slightly quicker but with an
   instantaneous jerk at the start of each accel/decel ramp). */
#define TRAJ_MOTION_PROFILE   PROFILE_SCURVE
#define TRAJ_MAX_VEL_DEG_S     60.0f //60.0f
#define TRAJ_MAX_ACCEL_DEG_S2  120.0f //120.f
#define TRAJ_CONTROL_RATE_HZ   100.0f   /* must match TIM6 ISR rate */

/* ---------------- ODrive UART link ----------------------------- */
#define ODRIVE_UART_BAUD   115200
#define ODRIVE_AXIS0_IDX   0
#define ODRIVE_AXIS1_IDX   1
#define ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL  8
#define ODRIVE_CONTROL_MODE_POSITION_CONTROL   3
#define ODRIVE_INPUT_MODE_PASSTHROUGH          1
#define ODRIVE_AXIS_STATE_IDLE 1

/* ---------------- Conveyor pick-timing ---------------- */
#define CAMERA_TO_PICK_DISTANCE_MM   500.0f  /* EDIT: distance along travel direction, camera FOV -> pick point */
#define CONVEYOR_VELOCITY_MM_S        80.0f  /* EDIT: fixed conveyor speed */
#define PICK_TIME_OFFSET_MS             0    /* EDIT: tune empirically once picking live; +delays grab, -advances it */
#define MIN_LEAD_TIME_MS             1500    /* warn (not block) if less than this much time before predicted arrival */

/* Fixed pick / drop locations for the conveyor cycle */
#define PICK_X_MM     0.0f
#define PICK_Y_MM   400.0f
#define DROP_X_MM   0.0f
#define DROP_Y_MM   650.0f

#endif /* APP_CONFIG_H */
