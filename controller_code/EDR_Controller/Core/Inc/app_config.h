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
#define TRAJ_MAX_VEL_DEG_S     60.0f
#define TRAJ_MAX_ACCEL_DEG_S2  120.0f
#define TRAJ_CONTROL_RATE_HZ   100.0f   /* must match TIM6 ISR rate */

/* ---------------- ODrive UART link ----------------------------- */
#define ODRIVE_UART_BAUD   115200
#define ODRIVE_AXIS0_IDX   0
#define ODRIVE_AXIS1_IDX   1
#define ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL  8
#define ODRIVE_CONTROL_MODE_POSITION_CONTROL   3
#define ODRIVE_INPUT_MODE_PASSTHROUGH          1

#endif /* APP_CONFIG_H */
