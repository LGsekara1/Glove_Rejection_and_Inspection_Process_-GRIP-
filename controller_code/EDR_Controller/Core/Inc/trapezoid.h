/* trapezoid.h */
#ifndef TRAPEZOID_H
#define TRAPEZOID_H

typedef enum {
    PROFILE_TRAPEZOID,
    PROFILE_SCURVE
} motion_profile_type_t;

typedef struct {
    float distance;   /* signed */
    float vmax, amax;
    float total_time, t_acc, vpeak;
    motion_profile_type_t type;
} axis_profile_t;

typedef struct {
    axis_profile_t p1, p2;
    float T1, T2, T;   /* individual and synchronized total times */
} sync_profile_t;

void trapezoid_timing(float distance, float vmax, float amax,
                       float *total_out, float *t_acc_out, float *vpeak_out);
void scurve_timing(float distance, float vmax, float amax,
                    float *total_out, float *t_acc_out, float *vpeak_out);

/* Displacement magnitude covered at time t (0..|distance|) - dispatches
   on p->type internally. */
float trapezoid_sample(const axis_profile_t *p, float t);

/* Instantaneous signed velocity at time t (deg/s), for feed-forward -
   dispatches on p->type internally. */
float trapezoid_velocity(const axis_profile_t *p, float t);

/* Builds a synchronized two-axis profile (both axes finish at the same
   time T = max(T1, T2)), matching synchronized_two_axis_profile() in the
   Python dashboard. `type` selects PROFILE_TRAPEZOID or PROFILE_SCURVE. */
void build_sync_profile(float d1, float d2, float vmax, float amax,
                         motion_profile_type_t type, sync_profile_t *out);

/* Signed displacement / velocity of each axis at global time t_global (0..out->T) */
float sync_profile_pos1(const sync_profile_t *sp, float t_global);
float sync_profile_pos2(const sync_profile_t *sp, float t_global);
float sync_profile_vel1(const sync_profile_t *sp, float t_global);
float sync_profile_vel2(const sync_profile_t *sp, float t_global);

#endif
