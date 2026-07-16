/* kinematics.h */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>
#include "app_config.h"

typedef struct { float x, y; } vec2_t;

/* Returns false if target unreachable for that arm. */
bool solve_arm_angle(vec2_t anchor, vec2_t target, float l1, float l2,
                      elbow_t elbow, float *theta_rad_out);

/* Full IK: returns false if either arm is unreachable. */
bool inverse_kinematics(float x, float y, float *theta1_deg, float *theta2_deg);

/* Circle intersection (for FK verification), branch: 1 = upper, 0 = lower */
bool circle_intersection(vec2_t p1, float r1, vec2_t p2, float r2,
                          int branch_upper, vec2_t *out);

/* Full FK: returns false if links don't form a valid triangle at this pose. */
bool forward_kinematics(float theta1_deg, float theta2_deg,
                         vec2_t *E_out, vec2_t *P1_out, vec2_t *P2_out);

#endif
