/* kinematics.c */
#include "kinematics.h"
#include <math.h>

bool solve_arm_angle(vec2_t anchor, vec2_t target, float l1, float l2,
                      elbow_t elbow, float *theta_rad_out)
{
    float dx = target.x - anchor.x;
    float dy = target.y - anchor.y;
    float d  = sqrtf(dx * dx + dy * dy);

    if (d > (l1 + l2) || d < fabsf(l1 - l2) || d < 1e-6f) {
        return false; /* unreachable */
    }

    float base_angle = atan2f(dy, dx);
    float cos_val = (l1 * l1 + d * d - l2 * l2) / (2.0f * l1 * d);
    if (cos_val > 1.0f)  cos_val = 1.0f;
    if (cos_val < -1.0f) cos_val = -1.0f;
    float elbow_angle = acosf(cos_val);

    *theta_rad_out = (elbow == ELBOW_UP) ? (base_angle + elbow_angle)
                                          : (base_angle - elbow_angle);
    return true;
}

bool inverse_kinematics(float x, float y, float *theta1_deg, float *theta2_deg)
{
    vec2_t A = { -GEOM_L0 / 2.0f, 0.0f };
    vec2_t B = {  GEOM_L0 / 2.0f, 0.0f };
    vec2_t target = { x, y };

    float t1_rad, t2_rad;
    if (!solve_arm_angle(A, target, GEOM_L1A, GEOM_L2A, GEOM_ELBOW1, &t1_rad))
        return false;
    if (!solve_arm_angle(B, target, GEOM_L1B, GEOM_L2B, GEOM_ELBOW2, &t2_rad))
        return false;

    *theta1_deg = t1_rad * 180.0f / (float)M_PI;
    *theta2_deg = t2_rad * 180.0f / (float)M_PI;
    return true;
}

bool circle_intersection(vec2_t p1, float r1, vec2_t p2, float r2,
                          int branch_upper, vec2_t *out)
{
    float dx = p2.x - p1.x, dy = p2.y - p1.y;
    float d = sqrtf(dx * dx + dy * dy);
    if (d > (r1 + r2) || d < fabsf(r1 - r2) || d < 1e-6f) return false;

    float a = (r1 * r1 - r2 * r2 + d * d) / (2.0f * d);
    float h_sq = r1 * r1 - a * a;
    float h = sqrtf(h_sq > 0.0f ? h_sq : 0.0f);

    float xm = p1.x + a * dx / d;
    float ym = p1.y + a * dy / d;
    float rx = -dy * (h / d);
    float ry =  dx * (h / d);

    vec2_t s1 = { xm + rx, ym + ry };
    vec2_t s2 = { xm - rx, ym - ry };

    if (branch_upper) *out = (s1.y >= s2.y) ? s1 : s2;
    else               *out = (s1.y <  s2.y) ? s1 : s2;
    return true;
}

bool forward_kinematics(float theta1_deg, float theta2_deg,
                         vec2_t *E_out, vec2_t *P1_out, vec2_t *P2_out)
{
    vec2_t A = { -GEOM_L0 / 2.0f, 0.0f };
    vec2_t B = {  GEOM_L0 / 2.0f, 0.0f };

    float t1 = theta1_deg * (float)M_PI / 180.0f;
    float t2 = theta2_deg * (float)M_PI / 180.0f;

    vec2_t P1 = { A.x + GEOM_L1A * cosf(t1), A.y + GEOM_L1A * sinf(t1) };
    vec2_t P2 = { B.x + GEOM_L1B * cosf(t2), B.y + GEOM_L1B * sinf(t2) };

    vec2_t E;
    if (!circle_intersection(P1, GEOM_L2A, P2, GEOM_L2B, FK_BRANCH_UPPER, &E))
        return false;

    if (E_out)  *E_out  = E;
    if (P1_out) *P1_out = P1;
    if (P2_out) *P2_out = P2;
    return true;
}
