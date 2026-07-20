/* trapezoid.c */
#include "trapezoid.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float fabs_f(float v) { return v < 0.0f ? -v : v; }
static float copysign_f(float mag, float sign_src) {
    return (sign_src < 0.0f) ? -mag : mag;
}

/* ---------------- Classic bang-bang trapezoid ---------------- */

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

static float trapezoid_sample_impl(const axis_profile_t *p, float t)
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

static float trapezoid_velocity_impl(const axis_profile_t *p, float t)
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

/* ---------------- Cycloidal (raised-cosine) S-curve ----------------
 * Same peak vmax / amax and same overall shape as the trapezoid (accel
 * ramp -> cruise -> decel ramp), but acceleration itself ramps up and
 * back down smoothly across each ramp phase instead of snapping straight
 * to amax - removes the instantaneous jerk step that tends to excite
 * ringing on a real mechanism. Ramp phase takes pi/2 (~1.57x) longer to
 * cover the same speed change as a straight-line ramp would. Mirrors
 * _scurve_timing / _scurve_sample in the Python dashboard exactly.
 */

void scurve_timing(float distance, float vmax, float amax,
                    float *total_out, float *t_acc_out, float *vpeak_out)
{
    distance = fabs_f(distance);
    if (distance <= 1e-6f || vmax <= 1e-6f || amax <= 1e-6f) {
        *total_out = 0.0f; *t_acc_out = 0.0f; *vpeak_out = 0.0f;
        return;
    }

    /* peak accel = vpeak * pi / (2 * t_acc)  =>  t_acc = vpeak*pi/(2*amax) */
    float t_acc = vmax * (float)M_PI / (2.0f * amax);
    float d_acc = vmax * t_acc / 2.0f; /* cycloidal ramp averages to vpeak/2 */

    if (2.0f * d_acc >= distance) {
        /* triangular: vpeak^2 * pi/(2*amax) = distance */
        float vpeak = sqrtf(distance * 2.0f * amax / (float)M_PI);
        t_acc = vpeak * (float)M_PI / (2.0f * amax);
        *total_out = 2.0f * t_acc;
        *t_acc_out = t_acc;
        *vpeak_out = vpeak;
    } else {
        float d_flat = distance - 2.0f * d_acc;
        float t_flat = d_flat / vmax;
        *vpeak_out = vmax;
        *total_out = 2.0f * t_acc + t_flat;
        *t_acc_out = t_acc;
    }
}

/* Position covered tt seconds into a cycloidal ramp from 0 to vpeak over
   duration t_acc: integral of vpeak/2 * (1 - cos(pi*t/t_acc)) dt */
static float scurve_ramp_dist(float vpeak, float t_acc, float tt)
{
    if (t_acc <= 1e-6f) return 0.0f;
    return vpeak / 2.0f * (tt - (t_acc / (float)M_PI) * sinf((float)M_PI * tt / t_acc));
}

static float scurve_sample_impl(const axis_profile_t *p, float t)
{
    float distance_abs = fabs_f(p->distance);
    if (p->total_time <= 0.0f) return 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > p->total_time) t = p->total_time;

    float t_dec_start = p->total_time - p->t_acc;
    float d_acc_total = scurve_ramp_dist(p->vpeak, p->t_acc, p->t_acc);

    float s;
    if (t <= p->t_acc) {
        s = scurve_ramp_dist(p->vpeak, p->t_acc, t);
    } else if (t <= t_dec_start) {
        s = d_acc_total + p->vpeak * (t - p->t_acc);
    } else {
        float td = p->total_time - t;
        s = distance_abs - scurve_ramp_dist(p->vpeak, p->t_acc, td);
    }
    return s;
}

static float scurve_velocity_impl(const axis_profile_t *p, float t)
{
    if (p->total_time <= 0.0f) return 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > p->total_time) t = p->total_time;

    float t_dec_start = p->total_time - p->t_acc;
    float v_mag;
    if (t <= p->t_acc) {
        v_mag = (p->t_acc > 1e-6f)
              ? p->vpeak / 2.0f * (1.0f - cosf((float)M_PI * t / p->t_acc))
              : p->vpeak;
    } else if (t <= t_dec_start) {
        v_mag = p->vpeak;
    } else {
        float td = p->total_time - t;
        v_mag = (p->t_acc > 1e-6f)
              ? p->vpeak / 2.0f * (1.0f - cosf((float)M_PI * td / p->t_acc))
              : p->vpeak;
    }
    return copysign_f(v_mag, p->distance);
}

/* ---------------- Dispatchers ---------------- */

float trapezoid_sample(const axis_profile_t *p, float t)
{
    return (p->type == PROFILE_SCURVE) ? scurve_sample_impl(p, t) : trapezoid_sample_impl(p, t);
}

float trapezoid_velocity(const axis_profile_t *p, float t)
{
    return (p->type == PROFILE_SCURVE) ? scurve_velocity_impl(p, t) : trapezoid_velocity_impl(p, t);
}

/* ---------------- Two-axis sync (profile-agnostic) ---------------- */

static void build_axis_profile(axis_profile_t *p, float distance, float vmax, float amax,
                                motion_profile_type_t type)
{
    p->distance = distance;
    p->vmax = vmax;
    p->amax = amax;
    p->type = type;
    if (type == PROFILE_SCURVE) {
        scurve_timing(distance, vmax, amax, &p->total_time, &p->t_acc, &p->vpeak);
    } else {
        trapezoid_timing(distance, vmax, amax, &p->total_time, &p->t_acc, &p->vpeak);
    }
}

void build_sync_profile(float d1, float d2, float vmax, float amax,
                         motion_profile_type_t type, sync_profile_t *out)
{
    build_axis_profile(&out->p1, d1, vmax, amax, type);
    build_axis_profile(&out->p2, d2, vmax, amax, type);
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
    return trapezoid_velocity(&sp->p1, t_local) * (sp->T1 / sp->T);
}

float sync_profile_vel2(const sync_profile_t *sp, float t_global)
{
    if (sp->T <= 0.0f || sp->T2 <= 0.0f) return 0.0f;
    float t_local = t_global * (sp->T2 / sp->T);
    return trapezoid_velocity(&sp->p2, t_local) * (sp->T2 / sp->T);
}
