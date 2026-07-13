/*
 * motion_controller.h
 *
 * Ties kinematics + trapezoid profile + odrive_uart together. This is the
 * equivalent of everything in the Python FiveBarDashboard class EXCEPT the
 * NiceGUI widgets/SVG rendering: joint<->turns conversion, trajectory
 * streaming, waypoint paths, and homing.
 *
 * Execution model: unlike the Python version (which streamed a move from a
 * background thread while a GUI event loop kept running), these are
 * BLOCKING calls meant to run from a bare main loop or a dedicated FreeRTOS
 * task. Each move function returns only once the move (or homing step)
 * completes, times out, or is aborted via the fivebar_motion_request_abort()
 * flag - call that from a button/limit-switch ISR to interrupt a move
 * early, same role as the dashboard's "Stop Trajectory" button.
 */
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <stdbool.h>
#include "five_bar_types.h"
#include "odrive_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTION_OK = 0,
    MOTION_UNREACHABLE = -1,
    MOTION_ODRIVE_ERROR = -2,
    MOTION_ABORTED = -3,
    MOTION_TIMEOUT = -4,
} MotionResult;

typedef struct {
    ODriveUart odrive;

    FiveBarParams params;
    AxisConfig axis_cfg[2];
    TrajConfig traj_cfg;
    float home_angle_deg;

    volatile bool abort_requested;
} FiveBarMotion;

/* Initializes the controller with a bound ODrive UART device and starting
 * config values (copy in whatever you loaded from flash - see
 * dashboard_config.h - or sane defaults). */
void fivebar_motion_init(FiveBarMotion *m, ODriveUart odrive,
                          const FiveBarParams *params, const AxisConfig axis_cfg[2],
                          const TrajConfig *traj_cfg, float home_angle_deg);

/* Call from a button/estop ISR or safety task to interrupt whatever move is
 * currently streaming. The move function will return MOTION_ABORTED on its
 * next control tick; caller should clear the flag before starting the next
 * move (fivebar_motion_clear_abort()). */
void fivebar_motion_request_abort(FiveBarMotion *m);
void fivebar_motion_clear_abort(FiveBarMotion *m);

/* --- Joint <-> turns conversion (mirrors joint_deg_to_turns/turns_to_joint_deg) --- */
float fivebar_joint_deg_to_turns(const FiveBarMotion *m, uint8_t axis, float angle_deg);
float fivebar_turns_to_joint_deg(const FiveBarMotion *m, uint8_t axis, float turns);

/* Reads both axes' encoder position and converts to joint degrees.
 * Returns MOTION_OK or MOTION_ODRIVE_ERROR. */
MotionResult fivebar_motion_get_current_joint_deg(FiveBarMotion *m, float *theta1_deg, float *theta2_deg);

/*
 * Streams a trapezoidal joint-space move from current position to
 * (t1_target, t2_target), blocking until complete. Internally: reads
 * current position, builds a TwoAxisProfile from traj_cfg, then loops at
 * traj_cfg.control_rate_hz calling odrive_set_position() for both axes
 * (converted through the axis_cfg gear/offset/direction) until the profile
 * finishes or abort_requested is set.
 */
MotionResult fivebar_motion_move_joints(FiveBarMotion *m, float t1_target_deg, float t2_target_deg);

/* Computes IK for (x, y) using the elbow config in m->params, then calls
 * fivebar_motion_move_joints(). Returns MOTION_UNREACHABLE if IK fails. */
MotionResult fivebar_motion_move_xy(FiveBarMotion *m, float x, float y);

/* Runs a sequence of Cartesian waypoints back-to-back (each one a full
 * trapezoidal move from wherever the previous one ended), same as the
 * dashboard's "Run Path". Stops early (MOTION_ABORTED) if abort is
 * requested mid-sequence, or MOTION_UNREACHABLE if any waypoint's IK fails
 * (checked up front, before any motion starts, so a bad waypoint later in
 * the list can't strand the arm mid-path). */
MotionResult fivebar_motion_run_path(FiveBarMotion *m, const Waypoint *waypoints, int count);

/* --- Homing (mirrors _home_axis_blocking / home_both) --- */

/* Requests AXIS_STATE_HOMING on one axis and polls current_state until it
 * returns to AXIS_STATE_IDLE or timeout_ms elapses. Checks axis/motor/
 * encoder error afterward. */
MotionResult fivebar_motion_home_axis(FiveBarMotion *m, uint8_t axis, uint32_t timeout_ms);

/* Homes axis0 fully to completion, then axis1 - never both at once, same
 * as the Python dashboard's sequential default. */
MotionResult fivebar_motion_home_both(FiveBarMotion *m, uint32_t timeout_ms);

/* Enables closed-loop position control on both axes (position_control +
 * passthrough input mode, then requested_state = CLOSED_LOOP_CONTROL). */
MotionResult fivebar_motion_enable_closed_loop_both(FiveBarMotion *m);

/* Sets both axes to AXIS_STATE_IDLE immediately - use for an e-stop path.
 * Deliberately does the minimum possible work per call so it's safe to call
 * from tight timing contexts. */
MotionResult fivebar_motion_emergency_stop(FiveBarMotion *m);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROLLER_H */
