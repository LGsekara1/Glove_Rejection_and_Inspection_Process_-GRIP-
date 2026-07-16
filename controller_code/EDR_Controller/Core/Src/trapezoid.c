/* trapezoid.c */
#include "trapezoid.h"
#include <math.h>

static float fabs_f(float v) { return v < 0.0f ? -v : v; }
static float copysign_f(float mag, float sign_src) {
    return (sign_src < 0.0f) ? -mag : mag;
}

void trapezoid_timing(float distance, float vmax, float amax,
                       float *total_out, float *t_acc_out, float *vpeak_out)
{
    distance = fabs_f(distance);
    if (distance <= 1e-6f || vmax <= 1e-6f || amax <= 1e-6f) {
        *total_out = 0.0f; *t_acc_out = 0.0f; *vpeak_out = 0.0f;
        return;
    }

    float t_acc = vmax / amax;
    float d_acc = 0.5f * amax * t_acc * t_acc;

    if (2.0f * d_acc >= distance) {
        /* triangular profile */
        t_acc = sqrtf(distance / amax);
        *vpeak_out = amax * t_acc;
        *total_out = 2.0f * t_acc;
        *t_acc_out = t_acc;
    } else {
        float d_flat = distance - 2.0f * d_acc;
        float t_flat = d_flat / vmax;
        *vpeak_out = vmax;
        *total_out = 2.0f * t_acc + t_flat;
        *t_acc_out = t_acc;
    }
}

float trapezoid_sample(const axis_profile_t *p, float t)
{
    float distance_abs = fabs_f(p->distance);
    if (p->total_time <= 0.0f) return 0.0f;

    if (t < 0.0f) t = 0.0f;
    if (t > p->total_time) t = p->total_time;

    float t_dec_start = p->total_time - p->t_acc;
    float s;
    if (t <= p->t_acc) {
        s = 0.5f * p->amax * t * t;
    } else if (t <= t_dec_start) {
        s = 0.5f * p->amax * p->t_acc * p->t_acc + p->vpeak * (t - p->t_acc);
    } else {
        float td = p->total_time - t;
        s = distance_abs - 0.5f * p->amax * td * td;
    }
    return s;
}

float trapezoid_velocity(const axis_profile_t *p, float t)
{
    if (p->total_time <= 0.0f) return 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > p->total_time) t = p->total_time;

    float t_dec_start = p->total_time - p->t_acc;
    float v_mag;
    if (t <= p->t_acc) {
        v_mag = p->amax * t;
    } else if (t <= t_dec_start) {
        v_mag = p->vpeak;
    } else {
        v_mag = p->amax * (p->total_time - t);
    }
    return copysign_f(v_mag, p->distance);
}

static void build_axis_profile(axis_profile_t *p, float distance, float vmax, float amax)
{
    p->distance = distance;
    p->vmax = vmax;
    p->amax = amax;
    trapezoid_timing(distance, vmax, amax, &p->total_time, &p->t_acc, &p->vpeak);
}

void build_sync_profile(float d1, float d2, float vmax, float amax, sync_profile_t *out)
{
    build_axis_profile(&out->p1, d1, vmax, amax);
    build_axis_profile(&out->p2, d2, vmax, amax);
    out->T1 = out->p1.total_time;
    out->T2 = out->p2.total_time;
    out->T  = (out->T1 > out->T2) ? out->T1 : out->T2;
}

float sync_profile_pos1(const sync_profile_t *sp, float t_global)
{
    if (sp->T <= 0.0f) return 0.0f;
    float t_local = (sp->T1 > 0.0f) ? t_global * (sp->T1 / sp->T) : 0.0f;
    float mag = trapezoid_sample(&sp->p1, t_local);
    return (sp->p1.distance != 0.0f) ? copysign_f(mag, sp->p1.distance) : 0.0f;
}

float sync_profile_pos2(const sync_profile_t *sp, float t_global)
{
    if (sp->T <= 0.0f) return 0.0f;
    float t_local = (sp->T2 > 0.0f) ? t_global * (sp->T2 / sp->T) : 0.0f;
    float mag = trapezoid_sample(&sp->p2, t_local);
    return (sp->p2.distance != 0.0f) ? copysign_f(mag, sp->p2.distance) : 0.0f;
}

float sync_profile_vel1(const sync_profile_t *sp, float t_global)
{
    if (sp->T <= 0.0f || sp->T1 <= 0.0f) return 0.0f;
    float t_local = t_global * (sp->T1 / sp->T);
    /* chain rule: d(pos1)/dt_global = d(pos1)/dt_local * (T1/T) */
    return trapezoid_velocity(&sp->p1, t_local) * (sp->T1 / sp->T);
}

float sync_profile_vel2(const sync_profile_t *sp, float t_global)
{
    if (sp->T <= 0.0f || sp->T2 <= 0.0f) return 0.0f;
    float t_local = t_global * (sp->T2 / sp->T);
    return trapezoid_velocity(&sp->p2, t_local) * (sp->T2 / sp->T);
}
