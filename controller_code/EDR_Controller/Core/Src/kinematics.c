#include "kinematics.h"
#include <math.h>

float turns_to_joint_deg(float turns, const axis_cfg_t *cfg, float home_angle_deg)
{
    float normalized_angle_deg = ((turns - cfg->offset_turns) / cfg->gear_ratio) * 360.0f / cfg->direction;
    return normalized_angle_deg + home_angle_deg;
}

float joint_deg_to_turns(float angle_deg, const axis_cfg_t *cfg, float home_angle_deg)
{
    float normalized_angle_deg = angle_deg - home_angle_deg;
    return cfg->offset_turns + cfg->direction * (normalized_angle_deg / 360.0f) * cfg->gear_ratio;
}

/* Same circle-intersection construction as the Python circle_intersection(),
 * specialized for forward_kinematics' fixed "upper"/"lower" branch choice. */
static bool circle_intersection(float x1, float y1, float r1, float x2, float y2, float r2,
                                 bool upper_branch, float *out_x, float *out_y)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float d = sqrtf(dx * dx + dy * dy);

    if (d > (r1 + r2) || d < fabsf(r1 - r2) || d < 1e-6f) {
        return false; /* no intersection / degenerate */
    }

    float a = (r1 * r1 - r2 * r2 + d * d) / (2.0f * d);
    float h_sq = r1 * r1 - a * a;
    float h = sqrtf(h_sq > 0.0f ? h_sq : 0.0f);

    float xm = x1 + a * dx / d;
    float ym = y1 + a * dy / d;

    float rx = -dy * (h / d);
    float ry = dx * (h / d);

    float sol1_x = xm + rx, sol1_y = ym + ry;
    float sol2_x = xm - rx, sol2_y = ym - ry;

    if (upper_branch) {
        if (sol1_y >= sol2_y) { *out_x = sol1_x; *out_y = sol1_y; }
        else                  { *out_x = sol2_x; *out_y = sol2_y; }
    } else {
        if (sol1_y < sol2_y) { *out_x = sol1_x; *out_y = sol1_y; }
        else                 { *out_x = sol2_x; *out_y = sol2_y; }
    }
    return true;
}

bool forward_kinematics(float theta1_deg, float theta2_deg, const link_params_t *params,
                         float *out_x, float *out_y)
{
    float Ax = -params->L0 / 2.0f, Ay = 0.0f;
    float Bx = params->L0 / 2.0f, By = 0.0f;

    float t1 = theta1_deg * (float)M_PI / 180.0f;
    float t2 = theta2_deg * (float)M_PI / 180.0f;

    float P1x = Ax + params->l1a * cosf(t1);
    float P1y = Ay + params->l1a * sinf(t1);
    float P2x = Bx + params->l1b * cosf(t2);
    float P2y = By + params->l1b * sinf(t2);

    return circle_intersection(P1x, P1y, params->l2a, P2x, P2y, params->l2b,
                                params->fk_branch_upper, out_x, out_y);
}
