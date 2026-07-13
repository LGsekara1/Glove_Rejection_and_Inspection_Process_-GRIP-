/*
 * five_bar_types.h
 *
 * Shared structs for the five-bar linkage motion stack, ported from the
 * Python/NiceGUI dashboard. This header has no HAL dependency so it can be
 * unit-tested off-target if you want (e.g. compiled with gcc on a PC).
 */
#ifndef FIVE_BAR_TYPES_H
#define FIVE_BAR_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ELBOW_UP = 0,
    ELBOW_DOWN = 1,
} ElbowConfig;

typedef enum {
    FK_BRANCH_UPPER = 0,
    FK_BRANCH_LOWER = 1,
} FkBranch;

/* Link geometry, mm. Mirrors dashboard's self.params dict. */
typedef struct {
    float L0;   /* base separation between axis0 and axis1 pivots */
    float l1a;  /* axis0 proximal link length */
    float l2a;  /* axis0 distal link length */
    float l1b;  /* axis1 proximal link length */
    float l2b;  /* axis1 distal link length */
    ElbowConfig elbow1;
    ElbowConfig elbow2;
    FkBranch fk_branch;
} FiveBarParams;

/* 2D point, mm */
typedef struct {
    float x;
    float y;
} Point2D;

/* Motor <-> joint-angle conversion, one per axis. Mirrors self.axis_cfg[idx]. */
typedef struct {
    float gear_ratio;    /* motor turns per joint revolution */
    float offset_turns;  /* raw encoder turns considered "home" before gearing */
    float direction;     /* +1.0 or -1.0 */
} AxisConfig;

/* Software trapezoidal motion-planning limits, joint-space. */
typedef struct {
    float max_vel_deg_s;
    float max_accel_deg_s2;
    float control_rate_hz;
} TrajConfig;

/* One Cartesian waypoint for simple point-to-point path planning. */
typedef struct {
    float x;
    float y;
} Waypoint;

#ifdef __cplusplus
}
#endif

#endif /* FIVE_BAR_TYPES_H */
