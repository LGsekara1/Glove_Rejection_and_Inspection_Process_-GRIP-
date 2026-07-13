/*
 * kinematics.h
 *
 * C port of the forward-kinematics + turns<->joint-degrees conversion from
 * your Python NiceGUI dashboard (five_bar_dashboard_nicegui.py), trimmed to
 * what the status screen needs. Inverse kinematics / trapezoidal motion
 * profiling from that script aren't ported yet since this drop is
 * homing + status only - add them here the same way when you build the
 * Joint/IK screens.
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float L0;   /* base separation, mm */
    float l1a;  /* proximal link A, mm */
    float l2a;  /* distal link A, mm */
    float l1b;  /* proximal link B, mm */
    float l2b;  /* distal link B, mm */
    bool  fk_branch_upper; /* true = "upper" circle-intersection branch */
} link_params_t;

typedef struct {
    float gear_ratio;
    float offset_turns;
    float direction; /* +1.0 or -1.0 */
} axis_cfg_t;

/* turns -> joint angle (deg), mirrors Python turns_to_joint_deg() */
float turns_to_joint_deg(float turns, const axis_cfg_t *cfg, float home_angle_deg);

/* joint angle (deg) -> turns, mirrors Python joint_deg_to_turns() */
float joint_deg_to_turns(float angle_deg, const axis_cfg_t *cfg, float home_angle_deg);

/* Forward kinematics: given both joint angles (deg), returns end-effector
 * (x, y) in mm. Returns false if the two distal-link circles don't
 * intersect (geometrically invalid pose - shouldn't happen in practice
 * once encoders are homed correctly, but checked defensively). */
bool forward_kinematics(float theta1_deg, float theta2_deg, const link_params_t *params,
                         float *out_x, float *out_y);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */
