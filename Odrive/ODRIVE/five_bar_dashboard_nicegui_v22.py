"""
Five-Bar Linkage (Parallel SCARA) Control Dashboard - NiceGUI version (v22)
==========================================================================

Runs as a local web app (works on Python 3.8, no tkinter needed).

Install (Python 3.8 compatible release of NiceGUI):
    pip install odrive "nicegui==1.4.37"

Run:
    python five_bar_dashboard_nicegui_v22.py

Then open the URL it prints (default http://127.0.0.1:8080) in a browser.

Hardware: ODrive v3.6, firmware 0.5.6, odrive python lib 0.5.4
Two motors (axis0, axis1) at fixed base pivots, each driving a proximal link.
Distal links connect the proximal link ends to a common end-effector point,
forming a five-bar parallel linkage. Encoders are SPI absolute (each on its
own chip-select GPIO: axis0 -> GPIO_4, axis1 -> GPIO_3), so pos_estimate is in
turns and survives ODrive power-cycles. There are no endstops; the zero
reference is set purely in software ("Sync Now (Current Pose = 90deg)").

WHAT CHANGED FROM v20 (this is a deliberate "do it raw" rewrite)
----------------------------------------------------------------
The old PC-side motion planner is gone. v20 discretized every move into a
software S-curve / Hermite trajectory and streamed hundreds of input_pos
setpoints from a background thread. v21 removes all of that (Path Planning,
Custom Trajectory, Draw Path and Teach-By-Hand tabs, and their joint-space
interpolators) and instead exposes the ODrive's own motion primitives:

1. POINT-TO-POINT MOVES use the ODrive firmware trapezoidal trajectory
   planner (INPUT_MODE_TRAP_TRAJ). The PC sets each axis's
   trap_traj.config.vel_limit / accel_limit / decel_limit and writes
   input_pos ONCE; the firmware generates the profile. The two axes are
   time-synchronized (the faster-moving axis's limits are scaled down so both
   finish together) but the interpolation itself is done on the ODrive, not by
   streaming discrete points. Used by the Joint Control and Inverse Kinematics
   tabs.

2. VELOCITY CONTROL is real ODrive velocity control
   (CONTROL_MODE_VELOCITY_CONTROL), NOT position control emulated with tiny
   discrete steps. The new "Velocity Control" tab offers two Jacobian-based
   modes:
     a) Cartesian velocity JOG - you command an end-effector velocity vector
        (mm/s); it is mapped through the inverse Jacobian to joint velocities
        (deg/s -> turns/s) and written to controller.input_vel.
     b) PC-SIDE CARTESIAN POSITION CONTROL - the position loop is closed on
        the PC (not the ODrive): a control loop reads the live end-effector
        pose (encoders -> forward kinematics), computes the Cartesian error to
        a target, applies a proportional law to get a desired Cartesian
        velocity, resolves it through the inverse Jacobian, and streams
        input_vel. This is exactly "position control outside the ODrive, in
        the PC", built on top of ODrive velocity control.

3. LIVE GRAPHS: the right-hand panel now shows rolling plots of position
   (deg), velocity (deg/s) and motor current (A, Iq_measured) for both axes,
   fed from the poll loop when idle and from the motion/velocity threads while
   moving.

SAFETY (velocity control can destroy a mechanism if unguarded - these layers
are always on while a velocity session is active):
  * ODrive hardware WATCHDOG: entering velocity mode enables
    axis.config.enable_watchdog with a short axis.config.watchdog_timeout and
    the PC loop calls axis.watchdog_feed() every tick. If the PC loop stalls
    or dies, the ODrive faults to IDLE within the timeout instead of running
    the last commanded velocity forever. This is the single most important
    guard against a PC-side hiccup turning into a runaway.
  * SINGULARITY handling: joint velocity for a given Cartesian velocity is
    amplified near the five-bar's parallel singularities. The resolver
    measures that amplification (sigma_max of the inverse Jacobian) and
    smoothly de-rates commanded speed to zero as the pose approaches a
    singularity, so the arm eases off instead of whipping. Unreachable /
    degenerate poses are refused outright.
  * JOINT VELOCITY hard clamp (deg/s) and a PC-side ACCELERATION (slew) clamp
    (deg/s^2) on the commanded joint velocity, both direction-preserving.
  * DEADMAN on the jog: if no fresh jog command arrives within a timeout the
    commanded velocity is forced to zero.
  * The ODrive's own controller.config.vel_limit remains the firmware hard
    cap, and the global EMERGENCY STOP still idles both axes immediately.

The ODrive lock, connection/retry logic, SPI absolute-encoder config,
calibration / pre-calibration flags, software home reference, PID tuning +
step-response tooling, and the SVG linkage visualization are carried over from
v20 unchanged.
"""

import math
import time
import queue
import threading
import asyncio
import concurrent.futures
import json
import os
import base64

from nicegui import ui, run

try:
    import odrive
    from odrive.enums import *  # noqa: F401,F403
    ODRIVE_AVAILABLE = True
except ImportError:
    ODRIVE_AVAILABLE = False

# ---------------------------------------------------------------------------
# Fallback state / mode constants (works regardless of odrive.enums import)
# ---------------------------------------------------------------------------
AXIS_STATE_IDLE = 1
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
# NOTE: AXIS_STATE_HOMING (11) is intentionally not used anymore - this build
# has no limit switches / endstops, so ODrive-firmware homing-to-endstop is
# not applicable. The zero reference is set purely in software (see
# home_angle_deg / sync_reference_now()).

CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1

# The joint angle (deg, measured from the +X axis per this script's theta
# convention) that the Calibration tab's "Sync Now" button assigns to
# whatever pose the arm is physically in when you press it.
SYNC_REFERENCE_DEG = 90.0

# SPI absolute encoder modes (odrive.enums exposes these too when available,
# but they're redefined here so the dashboard still works if that import
# fails). Values match ODrive firmware 0.5.x's Encoder.Mode bitfield: the
# 0x100 "SPI" flag OR'd with a per-chip sub-mode index.
ENCODER_MODE_SPI_ABS_CUI = 0x100 + 0
ENCODER_MODE_SPI_ABS_AMS = 0x100 + 1
ENCODER_MODE_SPI_ABS_AEAT = 0x100 + 2
ENCODER_MODE_SPI_ABS_RLS = 0x100 + 3
ENCODER_MODE_SPI_ABS_MA732 = 0x100 + 5

# Maps the friendly names shown in the Config-tab dropdown to the mode
# constants above.
SPI_ENCODER_MODE_OPTIONS = {
    "AMS (AS5047/AS5048)": ENCODER_MODE_SPI_ABS_AMS,
    "CUI AMT22x": ENCODER_MODE_SPI_ABS_CUI,
    "Broadcom AEAT": ENCODER_MODE_SPI_ABS_AEAT,
    "RLS": ENCODER_MODE_SPI_ABS_RLS,
    "MA732": ENCODER_MODE_SPI_ABS_MA732,
}

# Local file (next to this script) used to persist the DASHBOARD's own
# software settings (link geometry, axis gear/offset/direction, home
# reference angle, trajectory limits) across restarts of this script. This
# is separate from - and in addition to - the ODrive's own persistent
# calibration (motor/encoder pre_calibrated flags, saved with
# save_configuration()), which survives ODrive power-cycles instead.
DASHBOARD_CONFIG_FILENAME = "five_bar_dashboard_config.json"
TAUGHT_TRAJECTORY_FILENAME = "five_bar_taught_trajectory.json"

from collections import deque

# ---------------------------------------------------------------------------
# Extra ODrive mode constants for this build (velocity control + firmware
# trapezoidal trajectory). odrive.enums exports these too when the library is
# installed; redefined here (matching ODrive firmware 0.5.6) so the dashboard
# still imports and the viz still works without the hardware/library present.
# ---------------------------------------------------------------------------
CONTROL_MODE_VELOCITY_CONTROL = 2
INPUT_MODE_VEL_RAMP = 2
INPUT_MODE_TRAP_TRAJ = 5


# ---------------------------------------------------------------------------
# Cartesian -> joint velocity resolver, with singularity handling (pure math,
# no hardware access). This is the heart of the Jacobian velocity control:
# given a desired end-effector velocity it returns the joint velocities to
# command, bounded so the mechanism cannot be driven past its safe limits.
# ---------------------------------------------------------------------------
def _svd2_singular_values(a, b, c, d):
    """Largest and smallest singular values of the 2x2 matrix [[a, b], [c, d]]
    (closed form). sigma_max is the worst-case gain of the matrix - here, how
    much a unit Cartesian velocity can be amplified into joint velocity."""
    e = (a + d) / 2.0
    f = (a - d) / 2.0
    g = (c + b) / 2.0
    h = (c - b) / 2.0
    q = math.hypot(e, h)
    r = math.hypot(f, g)
    return q + r, abs(q - r)


def cartesian_to_joint_velocity(x, y, vx, vy, params, joint_vel_cap_deg_s,
                                manip_soft, manip_hard):
    """
    Resolve a desired end-effector velocity (vx, vy) mm/s at pose (x, y) into
    joint velocities (deg/s) for the five-bar, with layered safety:

      1. A = numerical_jacobian(x, y) is d(theta)/d(xy) in deg/mm (the inverse
         Jacobian). The naive joint velocity is w = A . [vx, vy].
      2. SINGULARITY de-rating: sigma_max(A) (deg per mm) measures how strongly
         a Cartesian velocity is amplified into joint velocity. As the pose
         approaches a parallel singularity this grows without bound. Commanded
         speed is scaled down smoothly once sigma_max crosses `manip_soft`,
         reaching zero (full refusal) at `manip_hard` - the arm eases off near
         a singularity instead of whipping. An unreachable / degenerate pose
         (numerical_jacobian raises) is refused outright.
      3. Hard CLAMP: whatever survives is scaled so neither joint exceeds
         `joint_vel_cap_deg_s`, scaling BOTH joints by the same factor so the
         Cartesian direction is preserved (the arm slows; it does not veer).

    Returns (w1_deg_s, w2_deg_s, info). info keys:
      ok         - False if the request was fully blocked (singular/unreachable)
      reason     - short human-readable status
      sigma_max  - joint/Cartesian amplification at this pose (deg/mm)
      derate     - singularity speed factor applied (1.0 none .. 0.0 blocked)
      clamp      - joint-velocity clamp factor applied (1.0 = none)
    """
    info = {"ok": True, "reason": "ok", "sigma_max": float("inf"),
            "derate": 1.0, "clamp": 1.0}
    try:
        A = numerical_jacobian(x, y, params)
    except Exception as e:
        info.update(ok=False, reason="unreachable/singular pose", derate=0.0,
                    detail=str(e))
        return 0.0, 0.0, info

    a, b = A[0][0], A[0][1]
    c, d = A[1][0], A[1][1]
    sigma_max, _sigma_min = _svd2_singular_values(a, b, c, d)
    info["sigma_max"] = sigma_max

    if not math.isfinite(sigma_max) or sigma_max <= 0.0:
        info.update(ok=False, reason="degenerate Jacobian", derate=0.0)
        return 0.0, 0.0, info

    if manip_hard <= manip_soft:
        manip_hard = manip_soft + 1e-6

    if sigma_max >= manip_hard:
        info.update(ok=False, reason="near singularity - motion blocked",
                    derate=0.0)
        return 0.0, 0.0, info
    if sigma_max > manip_soft:
        derate = (manip_hard - sigma_max) / (manip_hard - manip_soft)
        info["reason"] = "near singularity - slowing"
    else:
        derate = 1.0
    info["derate"] = derate

    w1 = (a * vx + b * vy) * derate
    w2 = (c * vx + d * vy) * derate

    peak = max(abs(w1), abs(w2))
    if peak > joint_vel_cap_deg_s and peak > 1e-9:
        clamp = joint_vel_cap_deg_s / peak
        w1 *= clamp
        w2 *= clamp
        info["clamp"] = clamp
        if info["reason"] == "ok":
            info["reason"] = "joint-speed-limited"
    return w1, w2, info

# ---------------------------------------------------------------------------
# Kinematics helpers (pure functions, no hardware access)
# ---------------------------------------------------------------------------
def solve_arm_angle(anchor, target, l1, l2, elbow="up"):
    """
    Solve the base joint angle (radians) for one arm of a 2-link chain.
    anchor: fixed motor pivot (x, y)
    target: end-effector point (x, y)
    l1: proximal link length, l2: distal link length
    """
    dx = target[0] - anchor[0]
    dy = target[1] - anchor[1]
    d = math.hypot(dx, dy)

    if d > (l1 + l2) or d < abs(l1 - l2) or d == 0:
        raise ValueError(
            "Target unreachable for arm at {}: distance={:.2f}, limits=[{:.2f}, {:.2f}]".format(
                anchor, d, abs(l1 - l2), l1 + l2)
        )

    base_angle = math.atan2(dy, dx)
    cos_val = (l1 ** 2 + d ** 2 - l2 ** 2) / (2 * l1 * d)
    cos_val = max(-1.0, min(1.0, cos_val))
    elbow_angle = math.acos(cos_val)

    if elbow == "up":
        return base_angle + elbow_angle
    else:
        return base_angle - elbow_angle


def inverse_kinematics(x, y, params):
    """Given desired end-effector (x, y) mm, return (theta1_deg, theta2_deg)."""
    L0 = params["L0"]
    A = (-L0 / 2.0, 0.0)
    B = (L0 / 2.0, 0.0)

    theta1 = solve_arm_angle(A, (x, y), params["l1a"], params["l2a"], params["elbow1"])
    theta2 = solve_arm_angle(B, (x, y), params["l1b"], params["l2b"], params["elbow2"])

    return math.degrees(theta1), math.degrees(theta2)


def circle_intersection(p1, r1, p2, r2, branch="upper"):
    x1, y1 = p1
    x2, y2 = p2
    d = math.hypot(x2 - x1, y2 - y1)

    if d > (r1 + r2) or d < abs(r1 - r2) or d == 0:
        raise ValueError("Circles do not intersect: d={:.2f}, r1={}, r2={}".format(d, r1, r2))

    a = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
    h_sq = r1 ** 2 - a ** 2
    h = math.sqrt(max(0.0, h_sq))

    xm = x1 + a * (x2 - x1) / d
    ym = y1 + a * (y2 - y1) / d

    rx = -(y2 - y1) * (h / d)
    ry = (x2 - x1) * (h / d)

    sol1 = (xm + rx, ym + ry)
    sol2 = (xm - rx, ym - ry)

    if branch == "upper":
        return sol1 if sol1[1] >= sol2[1] else sol2
    else:
        return sol1 if sol1[1] < sol2[1] else sol2


def forward_kinematics(theta1_deg, theta2_deg, params):
    """Given motor angles (degrees), return end-effector (x, y) and elbow points."""
    L0 = params["L0"]
    A = (-L0 / 2.0, 0.0)
    B = (L0 / 2.0, 0.0)

    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)

    P1 = (A[0] + params["l1a"] * math.cos(t1), A[1] + params["l1a"] * math.sin(t1))
    P2 = (B[0] + params["l1b"] * math.cos(t2), B[1] + params["l1b"] * math.sin(t2))

    branch = "upper" if params.get("fk_branch", "upper") == "upper" else "lower"
    E = circle_intersection(P1, params["l2a"], P2, params["l2b"], branch=branch)

    return E, P1, P2


# ---------------------------------------------------------------------------
# Jacobian helpers (for Cartesian -> joint VELOCITY conversion)
#
# Used by the Custom Trajectory feature so a waypoint can specify an
# end-effector velocity (e.g. "match the conveyor: 80 mm/s along +X") instead
# of only a position. There's no simple closed-form Jacobian for this
# particular five-bar geometry choice, so it's computed numerically by
# finite-differencing the existing inverse_kinematics().
# ---------------------------------------------------------------------------
def numerical_jacobian(x, y, params, eps=0.5):
    """
    Returns [[dtheta1/dx, dtheta1/dy], [dtheta2/dx, dtheta2/dy]] (deg/mm) at
    (x, y), via central differences. Shrinks eps a few times if the probe
    points land outside the reachable workspace (common near its boundary).
    """
    last_err = None
    for e in (eps, eps / 2.0, eps / 5.0, eps / 10.0, eps / 25.0):
        try:
            t1_xp, t2_xp = inverse_kinematics(x + e, y, params)
            t1_xm, t2_xm = inverse_kinematics(x - e, y, params)
            t1_yp, t2_yp = inverse_kinematics(x, y + e, params)
            t1_ym, t2_ym = inverse_kinematics(x, y - e, params)
            dt1dx = (t1_xp - t1_xm) / (2 * e)
            dt2dx = (t2_xp - t2_xm) / (2 * e)
            dt1dy = (t1_yp - t1_ym) / (2 * e)
            dt2dy = (t2_yp - t2_ym) / (2 * e)
            return [[dt1dx, dt1dy], [dt2dx, dt2dy]]
        except ValueError as e_:
            last_err = e_
            continue
    raise ValueError("Could not compute Jacobian near ({:.2f}, {:.2f}): {}".format(x, y, last_err))


def joint_velocity_from_cartesian(J, vx, vy):
    """Given a 2x2 Jacobian (deg/mm) and a desired Cartesian velocity
    (mm/s), returns (theta1_dot, theta2_dot) in deg/s."""
    dt1dx, dt1dy = J[0]
    dt2dx, dt2dy = J[1]
    w1 = dt1dx * vx + dt1dy * vy
    w2 = dt2dx * vx + dt2dy * vy
    return w1, w2

# ---------------------------------------------------------------------------
# Trapezoidal motion-profile helpers (pure math, no hardware access)
# ---------------------------------------------------------------------------
def _trapezoid_timing(distance, vmax, amax):
    """
    Returns (total_time, accel_time, peak_velocity) for a trapezoidal (or,
    if the distance is too short to reach vmax, triangular) velocity profile
    covering |distance| starting and ending at rest.
    """
    distance = abs(distance)
    if distance <= 1e-9 or vmax <= 1e-9 or amax <= 1e-9:
        return 0.0, 0.0, 0.0

    t_acc = vmax / amax
    d_acc = 0.5 * amax * t_acc * t_acc

    if 2 * d_acc >= distance:
        # Triangular profile: never reaches vmax.
        t_acc = math.sqrt(distance / amax)
        vpeak = amax * t_acc
        total = 2 * t_acc
    else:
        d_flat = distance - 2 * d_acc
        t_flat = d_flat / vmax
        vpeak = vmax
        total = 2 * t_acc + t_flat

    return total, t_acc, vpeak

# ---------------------------------------------------------------------------
# Main dashboard
# ---------------------------------------------------------------------------
class FiveBarDashboard:
    def __init__(self):
        self.odrv0 = None
        self.connected = False

        # Messages produced before self.log_box exists get buffered here and
        # flushed once build_ui() has created the log widget.
        self._startup_messages = []

        self.params = {
            "L0": 300.0,
            "l1a": 300.0,
            "l2a": 450.0,
            "l1b": 300.0,
            "l2b": 450.0,
            "elbow1": "up",
            "elbow2": "down",
            "fk_branch": "upper",
        }

        self.axis_cfg = {
            0: {"gear_ratio": 1.0, "offset_turns": 0.0, "direction": -1.0},
            1: {"gear_ratio": 1.0, "offset_turns": 0.0, "direction": -1.0},
        }
        self.home_angle_deg = {0: 90.0, 1: 90.0}

        # SPI absolute encoder interface config (which CS GPIO + which chip).
        self.spi_cfg = {
            0: {"cs_gpio": 4, "mode": ENCODER_MODE_SPI_ABS_AMS},
            1: {"cs_gpio": 3, "mode": ENCODER_MODE_SPI_ABS_AMS},
        }

        # Point-to-point move limits. These are now handed to the ODrive
        # firmware trapezoidal trajectory planner (trap_traj.config.*), NOT
        # used to build a PC-side profile. Kept in joint deg for the UI and
        # converted to motor turns per axis at move time.
        self.traj_cfg = {
            "max_vel_deg_s": 60.0,
            "max_accel_deg_s2": 120.0,
        }

        # Velocity-control (Jacobian) safety + tuning parameters. Every one of
        # these bounds what the velocity loop can command; see the resolver and
        # _velocity_loop_blocking for how each is used.
        self.vel_cfg = {
            "loop_hz": 60.0,                 # PC control-loop rate
            "joint_vel_cap_deg_s": 45.0,     # hard clamp on |joint speed|
            "joint_accel_cap_deg_s2": 180.0, # PC-side slew limit on joint speed
            "max_cart_speed_mm_s": 80.0,     # cap on commanded EE speed
            "pos_kp": 3.0,                   # PC Cartesian position P gain (1/s)
            "pos_tol_mm": 1.0,               # position-loop "arrived" tolerance
            "manip_soft_deg_mm": 3.0,        # start de-rating above this sigma_max
            "manip_hard_deg_mm": 8.0,        # fully block at/above this sigma_max
            "watchdog_s": 0.15,              # ODrive auto-idles if PC stops feeding
            "deadman_s": 0.5,                # jog auto-stops if no fresh command
        }

        # Visualization display toggles.
        self.display_cfg = {
            "show_workspace": True,
        }

        # Motion-execution state (shared by trap moves and velocity control).
        self.motion_task = None
        self._motion_active = False
        self._motion_stop_event = threading.Event()
        self._viz_queue = queue.Queue(maxsize=1)

        # Serializes ALL actual ODrive/USB access across threads (see comment
        # in v20 - libusb is not safe to touch from two threads at once).
        self._odrv_lock = threading.Lock()

        # Visualization overlays (kept so render_svg still works; used now to
        # show the current velocity-control target marker).
        self.planned_path = []
        self.waypoints_viz = []
        self.velocity_viz = []

        # Velocity-control session state.
        self._vel_task = None
        self._vel_mode = False
        self._vel_submode = "jog"            # "jog" or "position"
        self._vel_stop_event = threading.Event()
        self._vel_cmd = {"vx": 0.0, "vy": 0.0}   # jog command (mm/s)
        self._vel_cmd_time = 0.0                 # perf_counter of last jog cmd
        self._vel_target = None                  # (x, y) mm for position mode
        self._vel_status = {"text": "idle", "class": "text-gray-500"}

        # Live telemetry (position / velocity / current) ring buffer. Producers
        # (poll loop when idle; motion + velocity threads while moving) push
        # samples onto _telemetry_queue; the UI-thread _drain_telemetry timer
        # moves them into _telemetry_buffer and refreshes the charts.
        self._telem_window_s = 20.0
        self._telemetry_buffer = deque(maxlen=1500)
        self._telemetry_queue = queue.Queue()
        self._telem_t0 = time.perf_counter()

        # PID tuning history (for overlaid step-response comparisons)
        self.step_test_history = []

        self._load_dashboard_config()
        self.build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def build_ui(self):
        ui.page_title("Five-Bar Linkage Dashboard")

        # Global ESC-key emergency stop. `ignore=[]` overrides NiceGUI's
        # default of not firing key events while an input/select/button/
        # textarea has focus - E-stop must work no matter what's focused
        # (e.g. mid-typing in a jog-velocity number field).
        ui.keyboard(on_key=self._on_key, ignore=[])

        with ui.row().classes("w-full items-center justify-between p-2"):
            self.connect_btn = ui.button("Connect to ODrive", on_click=self.connect_odrive)
            self.status_label = ui.label("Not connected").classes("text-red-600 font-bold")
            with ui.row().classes("items-center gap-2"):
                ui.button("Stop Motion", on_click=self.abort_motion, color="orange")
                ui.button("EMERGENCY STOP", on_click=self.emergency_stop, color="red").classes("font-bold")
                ui.button("Resume After E-Stop", on_click=self.resume_from_estop, color="green")

        with ui.row().classes("w-full no-wrap"):
            # ---------------- Left: control tabs ----------------
            with ui.column().classes("basis-1/3 min-w-[380px]"):
                with ui.tabs().classes("w-full") as tabs:
                    t_joint = ui.tab("Joint Control")
                    t_ik = ui.tab("Inverse Kinematics")
                    t_fk = ui.tab("Forward Kinematics")
                    t_vel = ui.tab("Velocity Control")
                    t_cal = ui.tab("Calibration / Homing")
                    t_pid = ui.tab("PID Tuning")
                    t_cfg = ui.tab("Config")

                with ui.tab_panels(tabs, value=t_joint).classes("w-full"):
                    with ui.tab_panel(t_joint):
                        self.build_joint_tab()
                    with ui.tab_panel(t_ik):
                        self.build_ik_tab()
                    with ui.tab_panel(t_fk):
                        self.build_fk_tab()
                    with ui.tab_panel(t_vel):
                        self.build_velocity_tab()
                    with ui.tab_panel(t_cal):
                        self.build_cal_tab()
                    with ui.tab_panel(t_pid):
                        self.build_pid_tab()
                    with ui.tab_panel(t_cfg):
                        self.build_cfg_tab()

            # ---------------- Right: visualization + live telemetry ----------------
            with ui.column().classes("basis-2/3 items-center"):
                ui.label("Live Linkage Visualization").classes("text-lg font-bold")
                self.show_workspace_checkbox = ui.checkbox(
                    "Show reachable workspace", value=self.display_cfg["show_workspace"],
                    on_change=self._on_toggle_workspace_overlay)
                self.viz = ui.html(self.render_svg(None, None, None)).classes("border")
                self.ee_label = ui.label("End effector: X=--  Y=--").classes("font-bold")

                ui.label("Live Telemetry (last {:.0f}s)".format(self._telem_window_s)).classes(
                    "text-lg font-bold mt-2")
                self.chart_pos = ui.echart(self._telemetry_chart_options(
                    "Position (deg)", "deg")).classes("w-full").style("height:180px")
                self.chart_vel = ui.echart(self._telemetry_chart_options(
                    "Velocity (deg/s)", "deg/s")).classes("w-full").style("height:180px")
                self.chart_cur = ui.echart(self._telemetry_chart_options(
                    "Motor current Iq (A)", "A")).classes("w-full").style("height:180px")

        # ---------------- Bottom: log ----------------
        ui.label("Log").classes("font-bold mt-2")
        self.log_box = ui.log(max_lines=200).classes("w-full h-40 border")

        for msg in self._startup_messages:
            self.log(msg)
        self._startup_messages = []

        # Idle live-pose poll (skipped while a move/velocity session is active,
        # since those threads push their own higher-rate updates).
        ui.timer(0.2, self.poll_live)
        # Drains viz updates pushed by background motion threads.
        ui.timer(0.05, self._drain_viz_queue)
        # Drains telemetry samples into the buffer and refreshes the charts.
        ui.timer(0.1, self._drain_telemetry)
        # Refreshes the velocity-control status line.
        ui.timer(0.1, self._refresh_vel_status)

    async def _on_toggle_workspace_overlay(self):
        self.display_cfg["show_workspace"] = bool(self.show_workspace_checkbox.value)
        self.save_dashboard_config(silent=True)
        if self.connected and self.odrv0 is not None:
            try:
                turns0, turns1 = await run.io_bound(self._locked_call, self._read_encoder_turns)
                self._force_viz_refresh(turns0, turns1)
                return
            except Exception:
                pass
        self.viz.set_content(self.render_svg(None, None, None))

    def build_joint_tab(self):
        ui.label("Move by joint angle (degrees)").classes("font-bold")
        ui.label(
            "Uses the ODrive firmware trapezoidal trajectory planner "
            "(INPUT_MODE_TRAP_TRAJ): the vel/accel limits from the Config tab "
            "are written to trap_traj.config and the target is sent once - the "
            "ODrive generates the profile. The two axes are time-synchronized "
            "so they start and finish together."
        ).classes("text-xs text-gray-500")
        self.theta1_input = ui.number(label="Theta1 (axis0)", value=0.0, format="%.2f")
        self.theta2_input = ui.number(label="Theta2 (axis1)", value=0.0, format="%.2f")
        ui.button("Move Joints (trapezoidal)", on_click=self.move_joints_from_inputs).classes("w-full")

        ui.separator()

        ui.label("Move by raw motor turns (instant, bypasses the trajectory "
                 "planner - use small increments only)").classes("font-bold")
        self.turns0_input = ui.number(label="Axis0 turns", value=0.0, format="%.4f")
        self.turns1_input = ui.number(label="Axis1 turns", value=0.0, format="%.4f")
        ui.button("Move Motors (raw turns)", on_click=self.move_raw_turns_from_inputs).classes("w-full")

        ui.separator()
        self.live_joint_label = ui.label("theta1=--  theta2=--")

    def build_ik_tab(self):
        ui.label("Target End-Effector Position (mm)").classes("font-bold")
        self.ik_x_input = ui.number(label="X", value=0.0, format="%.2f")
        self.ik_y_input = ui.number(label="Y", value=40.0, format="%.2f")
        self.ik_elbow1_select = ui.select(["up", "down"], value=self.params["elbow1"], label="Elbow 1 (axis0)")
        self.ik_elbow2_select = ui.select(["up", "down"], value=self.params["elbow2"], label="Elbow 2 (axis1)")

        ui.button("Compute Only", on_click=self.compute_ik_only).classes("w-full")
        ui.button("Compute & Move (trapezoidal)", on_click=self.compute_and_move_ik).classes("w-full")

        self.ik_result_label = ui.label("theta1=--  theta2=--")

    def build_fk_tab(self):
        ui.label("Joint Angles (degrees)").classes("font-bold")
        self.fk_t1_input = ui.number(label="Theta1 (axis0)", value=0.0, format="%.2f")
        self.fk_t2_input = ui.number(label="Theta2 (axis1)", value=0.0, format="%.2f")

        ui.button("Compute FK", on_click=self.compute_fk_from_inputs).classes("w-full")
        ui.button("Use Current Motor Angles", on_click=self.compute_fk_from_live).classes("w-full")

        self.fk_result_label = ui.label("X=--  Y=--")

    def build_velocity_tab(self):
        ui.label("Velocity Control (real ODrive velocity mode + Jacobian)").classes("font-bold")
        ui.label(
            "This puts the ODrive in CONTROL_MODE_VELOCITY_CONTROL and streams "
            "controller.input_vel - it does NOT emulate position control with "
            "discrete steps. While a session is active the ODrive hardware "
            "watchdog is armed: if this PC stops feeding it (loop stall, USB "
            "drop, app crash) the ODrive faults to IDLE within the watchdog "
            "timeout instead of running the last velocity forever. Joint speed, "
            "joint acceleration and proximity-to-singularity are all clamped "
            "(see the Config tab, Velocity Control Safety). Support the arm the "
            "first time you try this."
        ).classes("text-xs text-orange-700")

        ui.separator()
        self.vel_submode_toggle = ui.toggle(
            {"jog": "Cartesian velocity jog", "position": "PC-side position control"},
            value="jog", on_change=self._on_vel_submode_change)

        with ui.row().classes("w-full"):
            self.vel_start_btn = ui.button("Start Velocity Control", color="primary",
                                           on_click=self.start_velocity_control).classes("flex-1")
            self.vel_stop_btn = ui.button("Stop (hold position)", color="orange",
                                          on_click=self.stop_velocity_control).classes("flex-1")

        self.vel_status_label = ui.label("Velocity control: idle").classes(
            "text-sm font-bold text-gray-500")

        # ---------------- Jog sub-panel ----------------
        self.vel_jog_panel = ui.column().classes("w-full")
        with self.vel_jog_panel:
            ui.separator()
            ui.label("Cartesian velocity jog (end-effector frame, mm/s)").classes("font-bold")
            self.jog_speed_input = ui.number(label="Jog speed (mm/s)", value=30.0, format="%.1f")
            with ui.row().classes("w-full items-center justify-center gap-1"):
                ui.button("+Y", on_click=lambda: self.jog_dir(0.0, 1.0)).props("dense").classes("w-16")
            with ui.row().classes("w-full items-center justify-center gap-1"):
                ui.button("-X", on_click=lambda: self.jog_dir(-1.0, 0.0)).props("dense").classes("w-16")
                ui.button("STOP", color="red",
                          on_click=self.jog_stop).props("dense").classes("w-16")
                ui.button("+X", on_click=lambda: self.jog_dir(1.0, 0.0)).props("dense").classes("w-16")
            with ui.row().classes("w-full items-center justify-center gap-1"):
                ui.button("-Y", on_click=lambda: self.jog_dir(0.0, -1.0)).props("dense").classes("w-16")
            ui.label(
                "Buttons set an end-effector velocity vector. Because of the "
                "deadman, the arm keeps moving only while you re-press within "
                "the deadman timeout; press STOP (or release) to zero it. You "
                "can also type an explicit vector:"
            ).classes("text-xs text-gray-500")
            with ui.row().classes("w-full"):
                self.jog_vx_input = ui.number(label="Vx (mm/s)", value=0.0, format="%.1f").classes("flex-1")
                self.jog_vy_input = ui.number(label="Vy (mm/s)", value=0.0, format="%.1f").classes("flex-1")
            ui.button("Apply velocity vector", on_click=self.apply_jog_from_inputs).classes("w-full")

        # ---------------- Position-control sub-panel ----------------
        self.vel_pos_panel = ui.column().classes("w-full")
        with self.vel_pos_panel:
            ui.separator()
            ui.label("PC-side Cartesian position control").classes("font-bold")
            ui.label(
                "The position loop runs HERE, on the PC: it reads the live "
                "end-effector pose (encoders -> forward kinematics), computes "
                "the Cartesian error to the target, applies a proportional gain "
                "to get a desired Cartesian velocity (capped), resolves it "
                "through the inverse Jacobian and streams input_vel. Gains, "
                "caps and tolerance are on the Config tab."
            ).classes("text-xs text-gray-500")
            with ui.row().classes("w-full"):
                self.vel_target_x = ui.number(label="Target X (mm)", value=0.0, format="%.2f").classes("flex-1")
                self.vel_target_y = ui.number(label="Target Y (mm)", value=40.0, format="%.2f").classes("flex-1")
            with ui.row().classes("w-full"):
                ui.button("Set Target", on_click=self.set_velocity_target_from_inputs).classes("flex-1")
                ui.button("Use Current EE", on_click=self.set_velocity_target_current).classes("flex-1")
            self.vel_target_label = ui.label("No target set.").classes("text-xs")

        self._sync_vel_subpanels()

    def build_cal_tab(self):
        ui.button("Clear Errors", on_click=self.clear_errors).classes("w-full")
        ui.button("Calibrate Axis0 (motor + encoder)", on_click=lambda: self.calibrate_axis(0)).classes("w-full")
        ui.button("Calibrate Axis1 (motor + encoder)", on_click=lambda: self.calibrate_axis(1)).classes("w-full")
        ui.button("Calibrate Both", on_click=self.calibrate_both).classes("w-full")
        ui.separator()
        ui.button("Enable Closed Loop Control (Both)", on_click=self.enable_closed_loop_both).classes("w-full")

        ui.separator()
        ui.button("Idle Both Axes", on_click=self.idle_both).classes("w-full")
        ui.separator()
        ui.button("Show Errors", on_click=self.show_errors).classes("w-full")

        ui.separator()
        ui.label("Sync (Set as 90\u00b0)").classes("font-bold")
        ui.label(
            "Whatever pose the real arm is physically in right now, press "
            "this and that becomes joint angle 90 deg (from the +X axis). "
            "Each axis is set independently from its own current encoder "
            "reading - axis0 and axis1 are NOT averaged together, so this "
            "works correctly even if the two links aren't sitting "
            "symmetrically when you press it. Everything downstream (the "
            "Live Linkage Visualization, IK, FK, path planning) is computed "
            "relative to these references from then on. Because the "
            "encoders are SPI absolute, this stays valid across ODrive "
            "power-cycles - no need to press it again after a power-cycle "
            "unless the arm was physically moved while powered off."
        ).classes("text-xs text-gray-500")
        ui.button(
            "Sync Now (Current Pose = 90\u00b0)", on_click=self.sync_reference_now, color="primary",
        ).classes("w-full")
        self.sync_status_label = ui.label("").classes("text-xs")

        ui.separator()
        ui.label("Persistent Calibration (survive ODrive power-cycle)").classes("font-bold")
        ui.label(
            "Once both axes have calibrated successfully in this session (no "
            "motor/encoder errors), this marks that calibration as "
            "pre-verified on the ODrive (motor.config.pre_calibrated + "
            "encoder.config.pre_calibrated) so you don't need to press "
            "Calibrate again after power-cycling the ODrive - it saves to "
            "flash and reboots."
        ).classes("text-xs text-gray-500")
        ui.label(
            "Important: pre_calibrated skips re-running the CALIBRATION "
            "(the physical constants - phase resistance/inductance, "
            "encoder-to-phase electrical offset). With the SPI absolute "
            "encoders on this build, absolute POSITION is also retained "
            "across power-cycles (no index pulse to lose), so - unlike a "
            "plain incremental encoder - you do NOT need to re-sync after "
            "power-on to recover a valid turn count, as long as the arm "
            "wasn't physically moved while powered off."
        ).classes("text-xs text-orange-600")
        self.startup_closed_loop_checkbox = ui.checkbox(
            "Auto-enter closed loop control on power-up", value=True)
        ui.label(
            "Caution: with this on, the ODrive servos to whatever input_pos "
            "is left at (often 0 turns) as soon as it boots - if that isn't "
            "a safe pose for your linkage, the arm can move on power-up. "
            "Support the arm physically for the first power-cycle after "
            "enabling this."
        ).classes("text-xs text-orange-600")
        with ui.row().classes("w-full"):
            ui.button("Read Startup/Calibration Flags", on_click=self.read_startup_flags).classes("flex-1")
            ui.button("Mark Calibrated & Save (reboots)",
                      on_click=self.confirm_mark_precalibrated, color="primary").classes("flex-1")
        self.startup_flags_label = ui.label("").classes("text-xs whitespace-pre-line")
        ui.button("Clear Pre-Calibrated Flags (forces re-calibration, reboots)",
                  on_click=self.confirm_clear_precalibrated, color="red").classes("w-full")

    def build_pid_tab(self):
        ui.label(
            "Tip: use the nudge buttons to scale a gain by \u00d70.8 / \u00d71.25 "
            "relative to its current field value, then hit Apply + Run Step "
            "Test. Each step test is kept and overlaid on the chart (last 4 "
            "runs) so you can directly compare a change against what came "
            "before it, instead of tuning from memory."
        ).classes("text-xs text-gray-500")

        ui.label("Axis0 Gains").classes("font-bold")
        with ui.row().classes("items-center gap-1 w-full"):
            self.pid0_pos_gain = ui.number(label="pos_gain", value=0.0, format="%.4f").classes("flex-1")
            ui.button("-20%", on_click=lambda: self._nudge_gain(self.pid0_pos_gain, 0.8)).props("dense")
            ui.button("+25%", on_click=lambda: self._nudge_gain(self.pid0_pos_gain, 1.25)).props("dense")
        with ui.row().classes("items-center gap-1 w-full"):
            self.pid0_vel_gain = ui.number(label="vel_gain", value=0.0, format="%.6f").classes("flex-1")
            ui.button("-20%", on_click=lambda: self._nudge_gain(self.pid0_vel_gain, 0.8)).props("dense")
            ui.button("+25%", on_click=lambda: self._nudge_gain(self.pid0_vel_gain, 1.25)).props("dense")
        with ui.row().classes("items-center gap-1 w-full"):
            self.pid0_vel_int_gain = ui.number(label="vel_integrator_gain", value=0.0, format="%.6f").classes("flex-1")
            ui.button("-20%", on_click=lambda: self._nudge_gain(self.pid0_vel_int_gain, 0.8)).props("dense")
            ui.button("+25%", on_click=lambda: self._nudge_gain(self.pid0_vel_int_gain, 1.25)).props("dense")
        self.pid0_vel_limit = ui.number(label="vel_limit (turns/s)", value=0.0, format="%.3f")
        self.pid0_current_lim = ui.number(label="current_lim (A)", value=0.0, format="%.2f")
        with ui.row().classes("w-full"):
            ui.button("Read Axis0", on_click=lambda: self.read_gains(0)).classes("flex-1")
            ui.button("Apply Axis0", on_click=lambda: self.apply_gains(0)).classes("flex-1")

        ui.separator()
        ui.label("Axis1 Gains").classes("font-bold")
        with ui.row().classes("items-center gap-1 w-full"):
            self.pid1_pos_gain = ui.number(label="pos_gain", value=0.0, format="%.4f").classes("flex-1")
            ui.button("-20%", on_click=lambda: self._nudge_gain(self.pid1_pos_gain, 0.8)).props("dense")
            ui.button("+25%", on_click=lambda: self._nudge_gain(self.pid1_pos_gain, 1.25)).props("dense")
        with ui.row().classes("items-center gap-1 w-full"):
            self.pid1_vel_gain = ui.number(label="vel_gain", value=0.0, format="%.6f").classes("flex-1")
            ui.button("-20%", on_click=lambda: self._nudge_gain(self.pid1_vel_gain, 0.8)).props("dense")
            ui.button("+25%", on_click=lambda: self._nudge_gain(self.pid1_vel_gain, 1.25)).props("dense")
        with ui.row().classes("items-center gap-1 w-full"):
            self.pid1_vel_int_gain = ui.number(label="vel_integrator_gain", value=0.0, format="%.6f").classes("flex-1")
            ui.button("-20%", on_click=lambda: self._nudge_gain(self.pid1_vel_int_gain, 0.8)).props("dense")
            ui.button("+25%", on_click=lambda: self._nudge_gain(self.pid1_vel_int_gain, 1.25)).props("dense")
        self.pid1_vel_limit = ui.number(label="vel_limit (turns/s)", value=0.0, format="%.3f")
        self.pid1_current_lim = ui.number(label="current_lim (A)", value=0.0, format="%.2f")
        with ui.row().classes("w-full"):
            ui.button("Read Axis1", on_click=lambda: self.read_gains(1)).classes("flex-1")
            ui.button("Apply Axis1", on_click=lambda: self.apply_gains(1)).classes("flex-1")

        ui.separator()
        ui.button("Save Config to Flash (reboots ODrive)", on_click=self.confirm_save_config).classes("w-full")

        ui.separator()
        ui.label("Step Response Test").classes("font-bold")
        self.step_axis_select = ui.select([0, 1], value=0, label="Axis")
        self.step_size_input = ui.number(label="Step size (turns)", value=0.05, format="%.4f")
        self.step_duration_input = ui.number(label="Sample duration (s)", value=1.5, format="%.2f")
        with ui.row().classes("w-full"):
            ui.button("Run Step Test", on_click=self.run_step_test).classes("flex-1")
            ui.button("Clear History", on_click=self.clear_step_history).classes("flex-1")
        self.step_metrics_label = ui.label("").classes("text-xs")
        self.step_chart = ui.html("").classes("border")

    def build_cfg_tab(self):
        ui.label("Link Geometry (mm)").classes("font-bold")
        self.cfg_L0 = ui.number(label="Base separation L0", value=self.params["L0"])
        self.cfg_l1a = ui.number(label="Proximal link A (l1a)", value=self.params["l1a"])
        self.cfg_l2a = ui.number(label="Distal link A (l2a)", value=self.params["l2a"])
        self.cfg_l1b = ui.number(label="Proximal link B (l1b)", value=self.params["l1b"])
        self.cfg_l2b = ui.number(label="Distal link B (l2b)", value=self.params["l2b"])

        ui.separator()
        ui.label("Motor <-> Joint Conversion").classes("font-bold")
        self.cfg_gear0 = ui.number(label="Axis0 gear ratio (motor turns/rev)", value=self.axis_cfg[0]["gear_ratio"])
        self.cfg_off0 = ui.number(label="Axis0 zero offset (turns)", value=self.axis_cfg[0]["offset_turns"])
        self.cfg_dir0 = ui.number(label="Axis0 direction (+1/-1)", value=self.axis_cfg[0]["direction"])
        self.cfg_gear1 = ui.number(label="Axis1 gear ratio (motor turns/rev)", value=self.axis_cfg[1]["gear_ratio"])
        self.cfg_off1 = ui.number(label="Axis1 zero offset (turns)", value=self.axis_cfg[1]["offset_turns"])
        self.cfg_dir1 = ui.number(label="Axis1 direction (+1/-1)", value=self.axis_cfg[1]["direction"])

        ui.separator()
        ui.label("Encoder Interface (SPI)").classes("font-bold")
        ui.label(
            "This build uses SPI absolute encoders (not ABI/incremental). "
            "Each axis's encoder talks over its own chip-select GPIO line: "
            "M0 (axis0) -> GPIO_4, M1 (axis1) -> GPIO_3 by default. Applying "
            "this writes axis.encoder.config.mode and "
            "axis.encoder.config.abs_spi_cs_gpio_pin, then saves to flash "
            "and reboots the ODrive - an encoder mode/CS change only takes "
            "full effect after a reboot."
        ).classes("text-xs text-gray-500")
        self.cfg_spi_mode0 = ui.select(
            list(SPI_ENCODER_MODE_OPTIONS.keys()), label="Axis0 (M0) encoder chip",
            value=self._spi_mode_label(self.spi_cfg[0]["mode"]))
        self.cfg_spi_cs0 = ui.number(label="Axis0 (M0) chip-select GPIO", value=self.spi_cfg[0]["cs_gpio"])
        self.cfg_spi_mode1 = ui.select(
            list(SPI_ENCODER_MODE_OPTIONS.keys()), label="Axis1 (M1) encoder chip",
            value=self._spi_mode_label(self.spi_cfg[1]["mode"]))
        self.cfg_spi_cs1 = ui.number(label="Axis1 (M1) chip-select GPIO", value=self.spi_cfg[1]["cs_gpio"])
        with ui.row().classes("w-full"):
            ui.button("Read Current SPI Encoder Config", on_click=self.read_spi_encoder_config).classes("flex-1")
            ui.button("Apply SPI Encoder Config (saves & reboots)",
                      on_click=self.confirm_apply_spi_encoder_config, color="primary").classes("flex-1")

        ui.separator()
        ui.label("Point-to-Point Move Limits (ODrive trapezoidal trajectory)").classes("font-bold")
        ui.label(
            "These joint limits are converted per-axis to motor turns and "
            "written to trap_traj.config.vel_limit / accel_limit / decel_limit "
            "before each Joint/IK move. They are clamped below each axis's "
            "controller.config.vel_limit (the firmware hard cap - see PID "
            "Tuning) so a move can't request an overspeed."
        ).classes("text-xs text-gray-500")
        self.cfg_max_vel = ui.number(label="Max joint velocity (deg/s)", value=self.traj_cfg["max_vel_deg_s"])
        self.cfg_max_accel = ui.number(label="Max joint acceleration (deg/s^2)",
                                        value=self.traj_cfg["max_accel_deg_s2"])

        ui.separator()
        ui.label("Velocity Control Safety (Jacobian velocity mode)").classes("font-bold")
        ui.label(
            "All of these bound what the velocity loop can command. The "
            "watchdog is the key runaway guard: the ODrive idles itself if "
            "this PC stops feeding it within the timeout. The joint speed cap "
            "and acceleration (slew) cap bound joint motion directly; the "
            "singularity thresholds (sigma_max of the inverse Jacobian, in "
            "deg/mm) smoothly de-rate speed to zero as the arm nears a "
            "singularity so it can't whip."
        ).classes("text-xs text-gray-500")
        self.cfg_vel_loop_hz = ui.number(label="Control loop rate (Hz)", value=self.vel_cfg["loop_hz"])
        self.cfg_vel_joint_cap = ui.number(label="Joint velocity hard cap (deg/s)",
                                            value=self.vel_cfg["joint_vel_cap_deg_s"])
        self.cfg_vel_accel_cap = ui.number(label="Joint acceleration (slew) cap (deg/s^2)",
                                            value=self.vel_cfg["joint_accel_cap_deg_s2"])
        self.cfg_vel_cart_speed = ui.number(label="Max commanded EE speed (mm/s)",
                                             value=self.vel_cfg["max_cart_speed_mm_s"])
        self.cfg_vel_pos_kp = ui.number(label="Position-control P gain (1/s)", value=self.vel_cfg["pos_kp"])
        self.cfg_vel_pos_tol = ui.number(label="Position-control arrive tolerance (mm)",
                                          value=self.vel_cfg["pos_tol_mm"])
        self.cfg_vel_manip_soft = ui.number(label="Singularity soft threshold (deg/mm, start slowing)",
                                             value=self.vel_cfg["manip_soft_deg_mm"])
        self.cfg_vel_manip_hard = ui.number(label="Singularity hard threshold (deg/mm, block)",
                                             value=self.vel_cfg["manip_hard_deg_mm"])
        self.cfg_vel_watchdog = ui.number(label="ODrive watchdog timeout (s)", value=self.vel_cfg["watchdog_s"])
        self.cfg_vel_deadman = ui.number(label="Jog deadman timeout (s)", value=self.vel_cfg["deadman_s"])

        ui.button("Apply Config", on_click=self.apply_config).classes("w-full")

        ui.separator()
        ui.label("Dashboard Settings Persistence").classes("font-bold")
        ui.label(
            "Everything on this tab (plus the home reference angle on the "
            "Calibration tab) is auto-saved to {} next to this script "
            "whenever you apply it, so restarting THIS SCRIPT doesn't lose "
            "your geometry/gear/offset/home setup. This is separate from the "
            "ODrive's own persistent calibration.".format(DASHBOARD_CONFIG_FILENAME)
        ).classes("text-xs text-gray-500")
        with ui.row().classes("w-full"):
            ui.button("Save Dashboard Settings Now", on_click=lambda: self.save_dashboard_config()).classes("flex-1")
            ui.button("Reload From File", on_click=self.reload_dashboard_config).classes("flex-1")

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------
    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        line = "[{}] {}".format(ts, msg)
        self.log_box.push(line)
        print(line)

    # ------------------------------------------------------------------
    # Dashboard-settings persistence (survives restarting THIS SCRIPT -
    # separate from the ODrive's own persistent calibration below, which
    # survives power-cycling the ODrive itself)
    # ------------------------------------------------------------------
    def _dashboard_config_path(self):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), DASHBOARD_CONFIG_FILENAME)

    def _load_dashboard_config(self):
        path = self._dashboard_config_path()
        if not os.path.exists(path):
            self._startup_messages.append(
                "No saved dashboard settings found at {} - using built-in defaults.".format(path))
            return
        try:
            with open(path, "r") as f:
                data = json.load(f)
            if "params" in data:
                self.params.update(data["params"])
            if "axis_cfg" in data:
                for idx_str, cfg in data["axis_cfg"].items():
                    idx = int(idx_str)
                    if idx in self.axis_cfg:
                        self.axis_cfg[idx].update(cfg)
            if "spi_cfg" in data:
                for idx_str, cfg in data["spi_cfg"].items():
                    idx = int(idx_str)
                    if idx in self.spi_cfg:
                        self.spi_cfg[idx].update(cfg)
            if "home_angle_deg" in data:
                raw = data["home_angle_deg"]
                if isinstance(raw, dict):
                    for idx_str, val in raw.items():
                        idx = int(idx_str)
                        if idx in self.home_angle_deg:
                            self.home_angle_deg[idx] = float(val)
                else:
                    self.home_angle_deg = {0: float(raw), 1: float(raw)}
            # Only merge keys we still use, so an old v20 file (which also had
            # control_rate_hz / motion_profile) loads cleanly without dragging
            # dead settings back in.
            if "traj_cfg" in data:
                for k in self.traj_cfg:
                    if k in data["traj_cfg"]:
                        self.traj_cfg[k] = data["traj_cfg"][k]
            if "vel_cfg" in data:
                for k in self.vel_cfg:
                    if k in data["vel_cfg"]:
                        self.vel_cfg[k] = data["vel_cfg"][k]
            if "display_cfg" in data:
                self.display_cfg.update(data["display_cfg"])
            self._startup_messages.append("Loaded saved dashboard settings from {}.".format(path))
        except Exception as e:
            self._startup_messages.append("Failed to load dashboard settings from {}: {}".format(path, e))

    def save_dashboard_config(self, silent=False):
        """Writes link geometry, axis gear/offset/direction, home reference
        angle, move limits and velocity-control safety settings to a local
        JSON file so they survive restarting this script."""
        path = self._dashboard_config_path()
        data = {
            "params": self.params,
            "axis_cfg": {str(k): v for k, v in self.axis_cfg.items()},
            "spi_cfg": {str(k): v for k, v in self.spi_cfg.items()},
            "home_angle_deg": {str(k): v for k, v in self.home_angle_deg.items()},
            "traj_cfg": self.traj_cfg,
            "vel_cfg": self.vel_cfg,
            "display_cfg": self.display_cfg,
        }
        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            if not silent:
                self.log("Dashboard settings saved to {}.".format(path))
                ui.notify("Dashboard settings saved.", type="positive")
        except Exception as e:
            self.log("Failed to save dashboard settings: {}".format(e))
            if not silent:
                ui.notify("Failed to save dashboard settings: {}".format(e), type="negative")

    def reload_dashboard_config(self):
        """Re-reads the JSON file and pushes the values back into the Config
        tab fields."""
        self._load_dashboard_config()
        self.cfg_L0.value = self.params["L0"]
        self.cfg_l1a.value = self.params["l1a"]
        self.cfg_l2a.value = self.params["l2a"]
        self.cfg_l1b.value = self.params["l1b"]
        self.cfg_l2b.value = self.params["l2b"]
        self.cfg_gear0.value = self.axis_cfg[0]["gear_ratio"]
        self.cfg_off0.value = self.axis_cfg[0]["offset_turns"]
        self.cfg_dir0.value = self.axis_cfg[0]["direction"]
        self.cfg_gear1.value = self.axis_cfg[1]["gear_ratio"]
        self.cfg_off1.value = self.axis_cfg[1]["offset_turns"]
        self.cfg_dir1.value = self.axis_cfg[1]["direction"]
        for label, val in SPI_ENCODER_MODE_OPTIONS.items():
            if val == self.spi_cfg[0]["mode"]:
                self.cfg_spi_mode0.value = label
            if val == self.spi_cfg[1]["mode"]:
                self.cfg_spi_mode1.value = label
        self.cfg_spi_cs0.value = self.spi_cfg[0]["cs_gpio"]
        self.cfg_spi_cs1.value = self.spi_cfg[1]["cs_gpio"]
        self.cfg_max_vel.value = self.traj_cfg["max_vel_deg_s"]
        self.cfg_max_accel.value = self.traj_cfg["max_accel_deg_s2"]
        self.cfg_vel_loop_hz.value = self.vel_cfg["loop_hz"]
        self.cfg_vel_joint_cap.value = self.vel_cfg["joint_vel_cap_deg_s"]
        self.cfg_vel_accel_cap.value = self.vel_cfg["joint_accel_cap_deg_s2"]
        self.cfg_vel_cart_speed.value = self.vel_cfg["max_cart_speed_mm_s"]
        self.cfg_vel_pos_kp.value = self.vel_cfg["pos_kp"]
        self.cfg_vel_pos_tol.value = self.vel_cfg["pos_tol_mm"]
        self.cfg_vel_manip_soft.value = self.vel_cfg["manip_soft_deg_mm"]
        self.cfg_vel_manip_hard.value = self.vel_cfg["manip_hard_deg_mm"]
        self.cfg_vel_watchdog.value = self.vel_cfg["watchdog_s"]
        self.cfg_vel_deadman.value = self.vel_cfg["deadman_s"]
        for msg in self._startup_messages:
            self.log(msg)
        self._startup_messages = []
        self.log("Dashboard settings reloaded from file.")

    # ------------------------------------------------------------------
    # ODrive connection
    # ------------------------------------------------------------------
    def _find_odrive_hard_timeout(self, attempt_timeout):
        """
        odrive.find_any(timeout=...) does not reliably self-cancel a stuck
        libusb call on some platforms/driver setups, so it can hang well past
        the requested timeout. Enforce a real hard timeout by running the
        call in its own thread and giving up on waiting for it.

        IMPORTANT: do NOT use the executor as a context manager here. A
        ThreadPoolExecutor's __exit__ calls shutdown(wait=True), which
        blocks until the background thread actually finishes - completely
        defeating the timeout (a "timed out" attempt would silently sit
        there for however long the real call takes, which is exactly what
        caused multi-minute connect times). shutdown(wait=False) lets this
        function return immediately on timeout; the abandoned thread is
        left to finish on its own without blocking anything else.
        """
        ex = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        try:
            future = ex.submit(odrive.find_any, timeout=attempt_timeout)
            return future.result(timeout=attempt_timeout + 2)
        finally:
            ex.shutdown(wait=False)

    async def connect_odrive(self):
        if not ODRIVE_AVAILABLE:
            ui.notify("The 'odrive' python package is not installed.", type="negative")
            return

        self.connect_btn.props("loading")
        self.status_label.text = "Connecting..."
        self.status_label.classes(remove="text-green-600", add="text-orange-500")

        max_attempts = 6
        attempt_timeout = 5  # seconds per attempt
        found = None

        for attempt in range(1, max_attempts + 1):
            self.log("Connect attempt {}/{} (timeout {}s)...".format(attempt, max_attempts, attempt_timeout))
            try:
                found = await run.io_bound(self._find_odrive_hard_timeout, attempt_timeout)
            except concurrent.futures.TimeoutError:
                self.log("Attempt {} timed out, retrying...".format(attempt))
                continue
            except Exception as e:
                self.log("Attempt {} failed: {}".format(attempt, e))
                continue
            if found is not None:
                break

        self.connect_btn.props(remove="loading")

        if found is None:
            self.status_label.text = "Not connected"
            self.status_label.classes(remove="text-orange-500", add="text-red-600")
            self.log("Could not connect after {} attempts. Check udev rules / drivers, "
                      "and try unplugging and replugging the ODrive.".format(max_attempts))
            ui.notify("Could not connect to ODrive.", type="negative")
            return

        self.odrv0 = found
        self.connected = True
        self.status_label.text = "Connected"
        self.status_label.classes(remove="text-orange-500", add="text-green-600")
        self.log("Connected to ODrive.")

    # ------------------------------------------------------------------
    # Motor <-> joint conversion
    # ------------------------------------------------------------------
    def joint_deg_to_turns(self, axis_idx, angle_deg):
        cfg = self.axis_cfg[axis_idx]
        normalized_angle_deg = angle_deg - self.home_angle_deg[axis_idx]
        return cfg["offset_turns"] + cfg["direction"] * (normalized_angle_deg / 360.0) * cfg["gear_ratio"]

    def turns_to_joint_deg(self, axis_idx, turns):
        cfg = self.axis_cfg[axis_idx]
        normalized_angle_deg = ((turns - cfg["offset_turns"]) / cfg["gear_ratio"]) * 360.0 / cfg["direction"]
        return normalized_angle_deg + self.home_angle_deg[axis_idx]

    def turns_vel_to_joint_deg_vel(self, axis_idx, turns_per_sec):
        """Same scale/direction conversion as turns_to_joint_deg, but for a
        RATE (turns/s -> deg/s) - no offset or home angle involved since
        those only shift position, not its derivative."""
        cfg = self.axis_cfg[axis_idx]
        return (turns_per_sec / cfg["gear_ratio"]) * 360.0 / cfg["direction"]

    @staticmethod
    def _spi_mode_label(mode_value):
        """Looks up the dropdown label for a raw SPI encoder mode int,
        falling back to the first known option if the stored value doesn't
        match anything (e.g. a fresh install before any config was saved)."""
        for label, val in SPI_ENCODER_MODE_OPTIONS.items():
            if val == mode_value:
                return label
        return next(iter(SPI_ENCODER_MODE_OPTIONS.keys()))

    # ------------------------------------------------------------------
    # Low-level motion primitives
    # ------------------------------------------------------------------
    def require_connected(self):
        if not self.connected or self.odrv0 is None:
            ui.notify("Connect to the ODrive first.", type="warning")
            return False
        return True

    def _read_axis_states_blocking(self):
        return self.odrv0.axis0.current_state, self.odrv0.axis1.current_state

    async def require_closed_loop(self):
        """
        Checks both axes are actually in CLOSED_LOOP_CONTROL before starting
        a commanded move. Without this, a write to controller.input_pos
        while an axis is IDLE (most commonly: right after EMERGENCY STOP,
        or before the first 'Enable Closed Loop Control' of a session)
        succeeds silently and does nothing - the motor just doesn't move,
        with no error anywhere. That "nothing happens and I don't know
        why" is exactly the trap this check is meant to catch early, with
        a clear message pointing at the fix instead of a silent no-op.
        """
        if not self.require_connected():
            return False
        try:
            state0, state1 = await run.io_bound(self._locked_call, self._read_axis_states_blocking)
        except Exception as e:
            self.log("Could not read axis state: {}".format(e))
            ui.notify("Could not read axis state: {}".format(e), type="negative")
            return False
        if state0 != AXIS_STATE_CLOSED_LOOP_CONTROL or state1 != AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.log("Move refused: axes are not in CLOSED_LOOP_CONTROL (axis0 state={}, "
                      "axis1 state={}). This is expected right after EMERGENCY STOP or before "
                      "the first move of a session - press 'Resume After E-Stop' (top bar) or "
                      "'Enable Closed Loop Control (Both)' (Calibration tab) first.".format(
                          state0, state1))
            ui.notify("Axes are idle (likely after E-Stop) - press 'Resume After E-Stop' first.",
                      type="warning")
            return False
        return True

    def _locked_call(self, func, *args):
        """Runs func(*args) while holding self._odrv_lock, so it can never
        overlap with another thread's ODrive access (see the lock's comment
        in __init__). This is the ONLY thing that should be passed as the
        first positional callable to run.io_bound() for anything that
        touches self.odrv0 - wrap the real target function/closure as the
        second argument, e.g. run.io_bound(self._locked_call, some_func,
        arg1, arg2)."""
        with self._odrv_lock:
            return func(*args)

    def _read_encoder_turns(self):
        return self.odrv0.axis0.encoder.pos_estimate, self.odrv0.axis1.encoder.pos_estimate

    async def _get_current_joint_deg(self):
        turns0, turns1 = await run.io_bound(self._locked_call, self._read_encoder_turns)
        t1 = self.turns_to_joint_deg(0, turns0)
        t2 = self.turns_to_joint_deg(1, turns1)
        return t1, t2

    def _write_input_pos(self, turns0, turns1):
        self.odrv0.axis0.controller.input_pos = turns0
        self.odrv0.axis1.controller.input_pos = turns1

    async def set_raw_turns(self, turns0, turns1):
        """Instant/direct position write - no trajectory shaping. Used for
        manual raw-turns jogging only."""
        if not await self.require_closed_loop():
            return

        def _do():
            self.odrv0.axis0.controller.input_pos = turns0
            self.odrv0.axis1.controller.input_pos = turns1

        try:
            await run.io_bound(self._locked_call, _do)
            self.log("Sent raw turns -> axis0={:.4f}, axis1={:.4f}".format(turns0, turns1))
        except Exception as e:
            self.log("Move failed: {}".format(e))

    # ------------------------------------------------------------------
    # Visualization update plumbing
    #
    # Background threads (the trap-move monitor and the velocity control loop)
    # can't touch NiceGUI elements directly, so they push the latest linkage
    # pose through a small queue that the UI-thread timer (_drain_viz_queue)
    # drains. Cancellation of those threads is via a threading.Event checked
    # every tick (asyncio task cancellation can't interrupt a thread that's
    # mid blocking-call).
    # ------------------------------------------------------------------
    def _push_viz_update(self, P1, P2, E, t1, t2):
        item = (P1, P2, E, t1, t2)
        try:
            self._viz_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self._viz_queue.put_nowait(item)
        except queue.Full:
            pass

    def _force_viz_refresh(self, turns0, turns1):
        """Immediately recomputes joint angles/FK from raw encoder turns
        (using whatever axis_cfg/home_angle_deg is current right NOW) and
        pushes it straight to the visualization panel, instead of waiting
        for the next poll_live tick. Used by anything that changes the
        physical<->simulation mapping (home capture, SPI config, etc.) so
        the sync is visibly instant."""
        try:
            t1 = self.turns_to_joint_deg(0, turns0)
            t2 = self.turns_to_joint_deg(1, turns1)
            E, P1, P2 = forward_kinematics(t1, t2, self.params)
            self.viz.set_content(self.render_svg(P1, P2, E))
            self.live_joint_label.text = "theta1={:.2f} deg   theta2={:.2f} deg".format(t1, t2)
            self.ee_label.text = "End effector: X={:.2f} mm   Y={:.2f} mm".format(E[0], E[1])
        except Exception:
            pass  # unreachable pose right at this instant - next poll tick will retry

    def _drain_viz_queue(self):
        try:
            P1, P2, E, t1, t2 = self._viz_queue.get_nowait()
        except queue.Empty:
            return
        try:
            self.viz.set_content(self.render_svg(P1, P2, E))
            self.live_joint_label.text = "theta1={:.2f} deg   theta2={:.2f} deg".format(t1, t2)
            self.ee_label.text = "End effector: X={:.2f} mm   Y={:.2f} mm".format(E[0], E[1])
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Live telemetry (position / velocity / current) plumbing
    # ------------------------------------------------------------------
    @staticmethod
    def _telemetry_chart_options(title, unit):
        return {
            "animation": False,
            "grid": {"left": 50, "right": 14, "top": 30, "bottom": 26},
            "title": {"text": title, "textStyle": {"fontSize": 12}},
            "tooltip": {"trigger": "axis"},
            "legend": {"data": ["axis0", "axis1"], "right": 10, "top": 4,
                       "textStyle": {"fontSize": 10}},
            "xAxis": {"type": "value", "name": "s", "min": "dataMin", "max": "dataMax",
                      "axisLabel": {"fontSize": 9}},
            "yAxis": {"type": "value", "name": unit, "scale": True,
                      "axisLabel": {"fontSize": 9}},
            "series": [
                {"name": "axis0", "type": "line", "showSymbol": False, "data": []},
                {"name": "axis1", "type": "line", "showSymbol": False, "data": []},
            ],
        }

    def _read_full_telemetry(self):
        """Unlocked read of pos/vel/current for both axes. Call via
        _locked_call (idle poll) or inside an existing lock block (threads)."""
        a0, a1 = self.odrv0.axis0, self.odrv0.axis1
        return (a0.encoder.pos_estimate, a1.encoder.pos_estimate,
                a0.encoder.vel_estimate, a1.encoder.vel_estimate,
                a0.motor.current_control.Iq_measured,
                a1.motor.current_control.Iq_measured)

    def _enqueue_telemetry_from_turns(self, p0, p1, v0, v1, c0, c1):
        """Convert raw ODrive readings to joint deg / deg-s + current and push
        one timestamped sample onto the telemetry queue (drained by the UI)."""
        t = time.perf_counter() - self._telem_t0
        pos0 = self.turns_to_joint_deg(0, p0)
        pos1 = self.turns_to_joint_deg(1, p1)
        vel0 = self.turns_vel_to_joint_deg_vel(0, v0)
        vel1 = self.turns_vel_to_joint_deg_vel(1, v1)
        try:
            self._telemetry_queue.put_nowait((t, pos0, pos1, vel0, vel1, c0, c1))
        except queue.Full:
            pass

    def _drain_telemetry(self):
        got = False
        while True:
            try:
                item = self._telemetry_queue.get_nowait()
            except queue.Empty:
                break
            self._telemetry_buffer.append(item)
            got = True
        self._telem_refresh_counter = getattr(self, "_telem_refresh_counter", 0) + 1
        # Refresh the charts at ~3 Hz (every 3rd 100 ms drain) when there's
        # new data, to keep the browser update rate sane.
        if got and (self._telem_refresh_counter % 3 == 0):
            self._refresh_telemetry_charts()

    def _refresh_telemetry_charts(self):
        buf = list(self._telemetry_buffer)
        if not buf:
            return
        t_latest = buf[-1][0]
        t_min = t_latest - self._telem_window_s
        buf = [s for s in buf if s[0] >= t_min]
        if not buf:
            return
        step = max(1, len(buf) // 300)
        buf = buf[::step]
        xs = [round(s[0], 3) for s in buf]

        def series(col):
            return [[xs[i], round(buf[i][col], 4)] for i in range(len(buf))]

        try:
            self.chart_pos.options["series"][0]["data"] = series(1)
            self.chart_pos.options["series"][1]["data"] = series(2)
            self.chart_vel.options["series"][0]["data"] = series(3)
            self.chart_vel.options["series"][1]["data"] = series(4)
            self.chart_cur.options["series"][0]["data"] = series(5)
            self.chart_cur.options["series"][1]["data"] = series(6)
            self.chart_pos.update()
            self.chart_vel.update()
            self.chart_cur.update()
        except Exception:
            pass

    def _joint_dps_to_turns_per_s(self, axis_idx, dps):
        """Joint velocity (deg/s) -> motor velocity (turns/s), signed, matching
        the slope of joint_deg_to_turns for this axis."""
        cfg = self.axis_cfg[axis_idx]
        return dps * cfg["direction"] * cfg["gear_ratio"] / 360.0

    # ------------------------------------------------------------------
    # Point-to-point moves via the ODrive firmware trapezoidal planner
    #
    # We set trap_traj.config limits and write input_pos ONCE per axis; the
    # firmware generates the profile. The two axes are time-synchronized by
    # scaling the faster axis's vel/accel down so both finish together. A
    # lightweight monitor thread only reads telemetry + drives the viz; it does
    # NOT compute or stream the trajectory.
    # ------------------------------------------------------------------
    @staticmethod
    def _solve_trap_scale(dist, vlim, alim, T_target):
        """Find k in (0, 1] such that a trapezoid over `dist` with limits
        (k*vlim, k*alim) takes T_target seconds. Duration decreases as k grows,
        so we bisect. Used to stretch the faster axis to match the slower."""
        def dur(k):
            return _trapezoid_timing(dist, k * vlim, k * alim)[0]
        if dur(1.0) >= T_target:
            return 1.0
        lo, hi = 1e-3, 1.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if dur(mid) > T_target:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def _trap_move_blocking(self, t1_target, t2_target, stop_event):
        ax0, ax1 = self.odrv0.axis0, self.odrv0.axis1
        try:
            with self._odrv_lock:
                cur0 = ax0.encoder.pos_estimate
                cur1 = ax1.encoder.pos_estimate
                ctrl_vlim0 = ax0.controller.config.vel_limit
                ctrl_vlim1 = ax1.controller.config.vel_limit
        except Exception as e:
            return "Move: could not read start state: {}".format(e)

        tgt0 = self.joint_deg_to_turns(0, t1_target)
        tgt1 = self.joint_deg_to_turns(1, t2_target)
        d0 = tgt0 - cur0
        d1 = tgt1 - cur1

        slope0 = abs(self.axis_cfg[0]["direction"]) * self.axis_cfg[0]["gear_ratio"] / 360.0
        slope1 = abs(self.axis_cfg[1]["direction"]) * self.axis_cfg[1]["gear_ratio"] / 360.0
        vmax = self.traj_cfg["max_vel_deg_s"]
        amax = self.traj_cfg["max_accel_deg_s2"]
        vlim0 = max(1e-4, vmax * slope0)
        alim0 = max(1e-4, amax * slope0)
        vlim1 = max(1e-4, vmax * slope1)
        alim1 = max(1e-4, amax * slope1)
        # Keep trap vel below the firmware hard cap so the move can't overspeed.
        if ctrl_vlim0 and ctrl_vlim0 > 0:
            vlim0 = min(vlim0, 0.95 * ctrl_vlim0)
        if ctrl_vlim1 and ctrl_vlim1 > 0:
            vlim1 = min(vlim1, 0.95 * ctrl_vlim1)

        T0 = _trapezoid_timing(d0, vlim0, alim0)[0]
        T1 = _trapezoid_timing(d1, vlim1, alim1)[0]
        T = max(T0, T1)
        if T <= 1e-6:
            return "Move: target already reached, nothing to do."

        if T0 < T and abs(d0) > 1e-9:
            k = self._solve_trap_scale(d0, vlim0, alim0, T)
            vlim0 *= k
            alim0 *= k
        if T1 < T and abs(d1) > 1e-9:
            k = self._solve_trap_scale(d1, vlim1, alim1, T)
            vlim1 *= k
            alim1 *= k

        try:
            with self._odrv_lock:
                for ax, vl, al in ((ax0, vlim0, alim0), (ax1, vlim1, alim1)):
                    ax.trap_traj.config.vel_limit = vl
                    ax.trap_traj.config.accel_limit = al
                    ax.trap_traj.config.decel_limit = al
                    ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    ax.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
                ax0.controller.input_pos = tgt0
                ax1.controller.input_pos = tgt1
        except Exception as e:
            return "Move failed to start: {}".format(e)

        rate = 50.0
        dt = 1.0 / rate
        timeout = T * 1.6 + 0.6
        pos_tol = 0.01   # turns
        vel_tol = 0.02   # turns/s
        start = time.perf_counter()
        i = 0
        while True:
            if stop_event.is_set():
                try:
                    with self._odrv_lock:
                        h0 = ax0.encoder.pos_estimate
                        h1 = ax1.encoder.pos_estimate
                        ax0.controller.input_pos = h0
                        ax1.controller.input_pos = h1
                except Exception:
                    pass
                return "Move cancelled (holding current position)."

            try:
                with self._odrv_lock:
                    p0, p1, v0, v1, c0, c1 = self._read_full_telemetry()
            except Exception as e:
                return "Move read failed, aborting: {}".format(e)

            t1d = self.turns_to_joint_deg(0, p0)
            t2d = self.turns_to_joint_deg(1, p1)
            try:
                E, P1, P2 = forward_kinematics(t1d, t2d, self.params)
                self._push_viz_update(P1, P2, E, t1d, t2d)
            except Exception:
                pass
            self._enqueue_telemetry_from_turns(p0, p1, v0, v1, c0, c1)

            elapsed = time.perf_counter() - start
            done = (abs(p0 - tgt0) < pos_tol and abs(p1 - tgt1) < pos_tol
                    and abs(v0) < vel_tol and abs(v1) < vel_tol)
            if (done and elapsed > min(0.15, 0.5 * T)) or elapsed > timeout:
                break

            i += 1
            sleep_for = (start + i * dt) - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)

        return "Move complete (firmware trapezoidal, T~={:.2f}s).".format(T)

    async def _run_trap_move(self, t1_target, t2_target, marker_xy=None):
        self.waypoints_viz = [marker_xy] if marker_xy is not None else []
        self.velocity_viz = []
        self.planned_path = []
        self.log("Move: firmware trapezoidal trajectory (both axes synchronized)...")
        self._motion_active = True
        self._motion_stop_event.clear()
        try:
            msg = await run.io_bound(
                self._trap_move_blocking, t1_target, t2_target, self._motion_stop_event)
            self.log(msg)
        finally:
            self._motion_active = False

    # ------------------------------------------------------------------
    # Velocity control (real ODrive velocity mode + Jacobian), with the safety
    # stack described at the top of the file.
    # ------------------------------------------------------------------
    def _on_vel_submode_change(self):
        if self._vel_mode:
            ui.notify("Stop velocity control before switching mode.", type="warning")
            self.vel_submode_toggle.value = self._vel_submode
            return
        self._sync_vel_subpanels()

    def _sync_vel_subpanels(self):
        sub = self.vel_submode_toggle.value or "jog"
        self.vel_jog_panel.set_visibility(sub == "jog")
        self.vel_pos_panel.set_visibility(sub == "position")

    def set_jog_velocity(self, vx, vy):
        self._vel_cmd = {"vx": float(vx), "vy": float(vy)}
        self._vel_cmd_time = time.perf_counter()

    def jog_dir(self, dx, dy):
        if not self._vel_mode or self._vel_submode != "jog":
            ui.notify("Start velocity control in Jog mode first.", type="warning")
            return
        try:
            speed = max(0.0, float(self.jog_speed_input.value))
        except (TypeError, ValueError):
            speed = 20.0
        self.set_jog_velocity(dx * speed, dy * speed)

    def jog_stop(self):
        self.set_jog_velocity(0.0, 0.0)

    def apply_jog_from_inputs(self):
        if not self._vel_mode or self._vel_submode != "jog":
            ui.notify("Start velocity control in Jog mode first.", type="warning")
            return
        try:
            vx = float(self.jog_vx_input.value)
            vy = float(self.jog_vy_input.value)
        except (TypeError, ValueError):
            ui.notify("Vx/Vy must be numbers.", type="negative")
            return
        self.set_jog_velocity(vx, vy)

    def set_velocity_target_from_inputs(self):
        try:
            tx = float(self.vel_target_x.value)
            ty = float(self.vel_target_y.value)
        except (TypeError, ValueError):
            ui.notify("Target X/Y must be numbers.", type="negative")
            return
        try:
            inverse_kinematics(tx, ty, self._current_ik_params())
        except Exception as e:
            ui.notify("Target not reachable: {}".format(e), type="negative")
            return
        self._vel_target = (tx, ty)
        self.waypoints_viz = [(tx, ty)]
        self.vel_target_label.text = "Target set: X={:.2f}  Y={:.2f} mm".format(tx, ty)
        self.log("Velocity position target set: ({:.2f}, {:.2f}) mm".format(tx, ty))

    async def set_velocity_target_current(self):
        if not self.require_connected():
            return
        try:
            turns0, turns1 = await run.io_bound(self._locked_call, self._read_encoder_turns)
            t1 = self.turns_to_joint_deg(0, turns0)
            t2 = self.turns_to_joint_deg(1, turns1)
            E, _, _ = forward_kinematics(t1, t2, self.params)
            self.vel_target_x.value = round(E[0], 2)
            self.vel_target_y.value = round(E[1], 2)
            self.set_velocity_target_from_inputs()
        except Exception as e:
            self.log("Could not read current EE for target: {}".format(e))

    def _enter_velocity_mode_blocking(self):
        ax0, ax1 = self.odrv0.axis0, self.odrv0.axis1
        wd = max(0.02, float(self.vel_cfg["watchdog_s"]))
        try:
            with self._odrv_lock:
                self.odrv0.clear_errors()
                for ax in (ax0, ax1):
                    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
                    ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                    ax.controller.input_vel = 0.0
                    ax.config.watchdog_timeout = wd
                    ax.watchdog_feed()
                    ax.config.enable_watchdog = True
                    ax.watchdog_feed()
                ax0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                ax1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                ax0.watchdog_feed()
                ax1.watchdog_feed()
        except Exception as e:
            return False, "error arming velocity mode: {}".format(e)

        time.sleep(0.05)
        try:
            with self._odrv_lock:
                ax0.watchdog_feed()
                ax1.watchdog_feed()
                s0, s1 = ax0.current_state, ax1.current_state
                e0, e1 = ax0.error, ax1.error
        except Exception as e:
            return False, "could not verify velocity mode: {}".format(e)

        if s0 != AXIS_STATE_CLOSED_LOOP_CONTROL or s1 != AXIS_STATE_CLOSED_LOOP_CONTROL:
            try:
                with self._odrv_lock:
                    for ax in (ax0, ax1):
                        ax.config.enable_watchdog = False
            except Exception:
                pass
            return False, ("axis did not enter closed loop (axis0 state={} err={}, "
                           "axis1 state={} err={})".format(s0, e0, s1, e1))
        return True, "ok"

    def _exit_velocity_mode_blocking(self):
        ax0, ax1 = self.odrv0.axis0, self.odrv0.axis1
        try:
            with self._odrv_lock:
                ax0.controller.input_vel = 0.0
                ax1.controller.input_vel = 0.0
                ax0.watchdog_feed()
                ax1.watchdog_feed()
                h0 = ax0.encoder.pos_estimate
                h1 = ax1.encoder.pos_estimate
                for ax in (ax0, ax1):
                    ax.config.enable_watchdog = False
                    ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                ax0.controller.input_pos = h0
                ax1.controller.input_pos = h1
        except Exception as e:
            return "exit warning: {}".format(e)
        return "zeroed velocity, disabled watchdog, holding position (position/passthrough)."

    def _velocity_loop_blocking(self, params, submode, stop_event):
        ax0, ax1 = self.odrv0.axis0, self.odrv0.axis1
        rate = max(5.0, float(self.vel_cfg["loop_hz"]))
        dt = 1.0 / rate
        cap = float(self.vel_cfg["joint_vel_cap_deg_s"])
        accel_cap = float(self.vel_cfg["joint_accel_cap_deg_s2"])
        max_cart = float(self.vel_cfg["max_cart_speed_mm_s"])
        kp = float(self.vel_cfg["pos_kp"])
        pos_tol = float(self.vel_cfg["pos_tol_mm"])
        msoft = float(self.vel_cfg["manip_soft_deg_mm"])
        mhard = float(self.vel_cfg["manip_hard_deg_mm"])
        deadman = float(self.vel_cfg["deadman_s"])

        w1_prev = w2_prev = 0.0
        start = time.perf_counter()
        i = 0
        err_check = 0
        err_period = max(1, int(rate / 2))

        while not stop_event.is_set():
            try:
                with self._odrv_lock:
                    p0, p1, v0, v1, c0, c1 = self._read_full_telemetry()
            except Exception:
                self._vel_status = {"text": "velocity: read error - stopping",
                                    "class": "text-red-600"}
                break

            t1d = self.turns_to_joint_deg(0, p0)
            t2d = self.turns_to_joint_deg(1, p1)
            try:
                E, P1, P2 = forward_kinematics(t1d, t2d, self.params)
                x, y = E
                fk_ok = True
            except Exception:
                x = y = None
                fk_ok = False

            vx = vy = 0.0
            status_txt = ""
            status_cls = "text-green-700"

            if fk_ok:
                if submode == "jog":
                    if (time.perf_counter() - self._vel_cmd_time) <= deadman:
                        vx = self._vel_cmd.get("vx", 0.0)
                        vy = self._vel_cmd.get("vy", 0.0)
                    else:
                        vx = vy = 0.0
                        status_txt = "idle (deadman) - press a direction"
                else:
                    tgt = self._vel_target
                    if tgt is not None:
                        ex = tgt[0] - x
                        ey = tgt[1] - y
                        dist = math.hypot(ex, ey)
                        if dist > pos_tol:
                            speed = min(max_cart, kp * dist)
                            vx = speed * ex / dist
                            vy = speed * ey / dist
                        else:
                            vx = vy = 0.0
                            status_txt = "at target (err {:.2f} mm)".format(dist)
                    else:
                        status_txt = "no target set"

            cs = math.hypot(vx, vy)
            if cs > max_cart and cs > 1e-9:
                vx *= max_cart / cs
                vy *= max_cart / cs

            sigma = float("nan")
            if fk_ok and (abs(vx) > 1e-9 or abs(vy) > 1e-9):
                w1, w2, info = cartesian_to_joint_velocity(
                    x, y, vx, vy, params, cap, msoft, mhard)
                sigma = info.get("sigma_max", float("nan"))
                if not info["ok"]:
                    w1 = w2 = 0.0
                    status_txt = ("blocked: " + info["reason"]) if submode == "position" \
                        else info["reason"]
                    status_cls = "text-red-600"
                else:
                    if info["derate"] < 1.0 or info["clamp"] < 1.0:
                        status_cls = "text-orange-700"
                    if not status_txt:
                        status_txt = "moving" if info["reason"] == "ok" else info["reason"]
            else:
                w1 = w2 = 0.0
                if not fk_ok:
                    status_txt = "pose read (FK) unreachable - holding"
                    status_cls = "text-orange-700"
                elif not status_txt:
                    status_txt = "holding"
                    status_cls = "text-gray-500"

            # Acceleration (slew) clamp on the commanded joint velocity.
            max_dv = accel_cap * dt
            w1 = max(w1_prev - max_dv, min(w1_prev + max_dv, w1))
            w2 = max(w2_prev - max_dv, min(w2_prev + max_dv, w2))
            # Final magnitude backstop.
            w1 = max(-cap, min(cap, w1))
            w2 = max(-cap, min(cap, w2))

            tv0 = self._joint_dps_to_turns_per_s(0, w1)
            tv1 = self._joint_dps_to_turns_per_s(1, w2)

            try:
                with self._odrv_lock:
                    ax0.controller.input_vel = tv0
                    ax1.controller.input_vel = tv1
                    ax0.watchdog_feed()
                    ax1.watchdog_feed()
            except Exception:
                self._vel_status = {"text": "velocity: write error - stopping",
                                    "class": "text-red-600"}
                break

            w1_prev, w2_prev = w1, w2

            if fk_ok:
                try:
                    self._push_viz_update(P1, P2, E, t1d, t2d)
                except Exception:
                    pass
            self._enqueue_telemetry_from_turns(p0, p1, v0, v1, c0, c1)

            sig_txt = "sigma={:.2f}".format(sigma) if math.isfinite(sigma) else "sigma=--"
            self._vel_status = {
                "text": "velocity [{}]: {}  |  w=({:.1f}, {:.1f}) deg/s  {} deg/mm".format(
                    submode, status_txt, w1, w2, sig_txt),
                "class": status_cls,
            }

            err_check += 1
            if err_check >= err_period:
                err_check = 0
                try:
                    with self._odrv_lock:
                        e0, e1 = ax0.error, ax1.error
                        s0, s1 = ax0.current_state, ax1.current_state
                    if (e0 or e1 or s0 != AXIS_STATE_CLOSED_LOOP_CONTROL
                            or s1 != AXIS_STATE_CLOSED_LOOP_CONTROL):
                        self._vel_status = {
                            "text": ("velocity: ODrive fault / left closed loop "
                                     "(axis0 err={} state={}, axis1 err={} state={}) - "
                                     "stopping".format(e0, s0, e1, s1)),
                            "class": "text-red-600"}
                        break
                except Exception:
                    pass

            i += 1
            sleep_for = (start + i * dt) - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)

        try:
            with self._odrv_lock:
                ax0.controller.input_vel = 0.0
                ax1.controller.input_vel = 0.0
                ax0.watchdog_feed()
                ax1.watchdog_feed()
        except Exception:
            pass
        return "Velocity loop ended."

    async def _run_velocity_control(self, submode):
        self._vel_submode = submode
        self._sync_vel_subpanels()
        self._motion_active = True
        self._vel_mode = True
        self._vel_stop_event.clear()
        self._vel_cmd = {"vx": 0.0, "vy": 0.0}
        self._vel_cmd_time = 0.0
        params = self._current_ik_params()

        ok, msg = await run.io_bound(self._enter_velocity_mode_blocking)
        if not ok:
            self._vel_mode = False
            self._motion_active = False
            self.log("Velocity control could not start - " + msg)
            ui.notify("Velocity control failed to start - " + msg, type="negative")
            self._vel_status = {"text": "velocity: failed to start - " + msg,
                                "class": "text-red-600"}
            return

        self.log("Velocity control started ({} mode). ODrive watchdog armed at "
                 "{:.0f} ms.".format(submode, 1000.0 * self.vel_cfg["watchdog_s"]))
        try:
            result = await run.io_bound(
                self._velocity_loop_blocking, params, submode, self._vel_stop_event)
            self.log(result)
        finally:
            exit_msg = await run.io_bound(self._exit_velocity_mode_blocking)
            self.log("Velocity control stopped - " + exit_msg)
            self._vel_mode = False
            self._motion_active = False
            if self._vel_status.get("class") != "text-red-600":
                self._vel_status = {"text": "velocity control: idle",
                                    "class": "text-gray-500"}

    async def start_velocity_control(self):
        if not self.require_connected():
            return
        if self._vel_mode or (self._vel_task is not None and not self._vel_task.done()):
            ui.notify("Velocity control is already running.", type="warning")
            return
        if self._motion_active:
            ui.notify("A move is in progress - stop it first.", type="warning")
            return
        submode = self.vel_submode_toggle.value or "jog"
        self._vel_task = asyncio.create_task(self._run_velocity_control(submode))

    async def stop_velocity_control(self):
        if not self._vel_mode and (self._vel_task is None or self._vel_task.done()):
            ui.notify("Velocity control is not running.", type="info")
            return
        self._vel_stop_event.set()
        if self._vel_task is not None:
            try:
                await self._vel_task
            except asyncio.CancelledError:
                pass

    def _refresh_vel_status(self):
        st = self._vel_status
        try:
            self.vel_status_label.text = st.get("text", "")
            self.vel_status_label.classes(
                replace="text-sm font-bold " + st.get("class", "text-gray-500"))
        except Exception:
            pass

    async def _launch_motion_task(self, coro):
        if self.motion_task is not None and not self.motion_task.done():
            self._motion_stop_event.set()
            self.motion_task.cancel()
            try:
                await self.motion_task
            except asyncio.CancelledError:
                pass
        self.motion_task = asyncio.create_task(coro)
        return self.motion_task

    async def abort_motion(self):
        acted = False
        if self._vel_mode or (self._vel_task is not None and not self._vel_task.done()):
            self._vel_stop_event.set()
            self.log("Stopping velocity control (will hold position).")
            acted = True
        if self._motion_active and not self._vel_mode:
            self._motion_stop_event.set()
            self.log("Aborting move.")
            acted = True
        if self.motion_task is not None and not self.motion_task.done():
            self.motion_task.cancel()
            acted = True
        if not acted:
            self.log("No motion in progress.")

    # ------------------------------------------------------------------
    # Joint Control tab actions
    # ------------------------------------------------------------------
    async def move_joints_from_inputs(self):
        if self._vel_mode:
            ui.notify("Stop velocity control before commanding a point move.", type="warning")
            return
        if not await self.require_closed_loop():
            return
        try:
            t1_target = float(self.theta1_input.value)
            t2_target = float(self.theta2_input.value)
        except (TypeError, ValueError):
            ui.notify("Theta1/Theta2 must be numbers.", type="negative")
            return

        try:
            E_target, _, _ = forward_kinematics(t1_target, t2_target, self.params)
            marker = E_target
        except Exception:
            marker = None

        await self._launch_motion_task(self._run_trap_move(t1_target, t2_target, marker))

    async def move_raw_turns_from_inputs(self):
        try:
            turns0 = float(self.turns0_input.value)
            turns1 = float(self.turns1_input.value)
        except (TypeError, ValueError):
            ui.notify("Turns must be numbers.", type="negative")
            return
        await self.set_raw_turns(turns0, turns1)

    # ------------------------------------------------------------------
    # Inverse kinematics actions
    # ------------------------------------------------------------------
    def _current_ik_params(self):
        p = dict(self.params)
        p["elbow1"] = self.ik_elbow1_select.value
        p["elbow2"] = self.ik_elbow2_select.value
        return p

    def compute_ik_only(self):
        try:
            x = float(self.ik_x_input.value)
            y = float(self.ik_y_input.value)
            t1, t2 = inverse_kinematics(x, y, self._current_ik_params())
        except Exception as e:
            self.ik_result_label.text = "Error: {}".format(e)
            self.log("IK error: {}".format(e))
            return
        self.ik_result_label.text = "theta1={:.2f} deg   theta2={:.2f} deg".format(t1, t2)
        self.log("IK computed for ({}, {}) -> theta1={:.2f}, theta2={:.2f}".format(x, y, t1, t2))

    async def compute_and_move_ik(self):
        if self._vel_mode:
            ui.notify("Stop velocity control before commanding a point move.", type="warning")
            return
        if not await self.require_closed_loop():
            return
        try:
            x = float(self.ik_x_input.value)
            y = float(self.ik_y_input.value)
            t1_target, t2_target = inverse_kinematics(x, y, self._current_ik_params())
        except Exception as e:
            self.ik_result_label.text = "Error: {}".format(e)
            self.log("IK error: {}".format(e))
            return
        self.ik_result_label.text = "theta1={:.2f} deg   theta2={:.2f} deg".format(t1_target, t2_target)
        await self._launch_motion_task(self._run_trap_move(t1_target, t2_target, (x, y)))

    # ------------------------------------------------------------------
    # Forward kinematics actions
    # ------------------------------------------------------------------
    def compute_fk_from_inputs(self):
        try:
            t1 = float(self.fk_t1_input.value)
            t2 = float(self.fk_t2_input.value)
            E, P1, P2 = forward_kinematics(t1, t2, self.params)
        except Exception as e:
            self.fk_result_label.text = "Error: {}".format(e)
            self.log("FK error: {}".format(e))
            return
        self.fk_result_label.text = "X={:.2f} mm   Y={:.2f} mm".format(E[0], E[1])
        self.viz.set_content(self.render_svg(P1, P2, E))

    async def compute_fk_from_live(self):
        if not self.require_connected():
            return
        try:
            turns0, turns1 = await run.io_bound(self._locked_call, self._read_encoder_turns)
            t1 = self.turns_to_joint_deg(0, turns0)
            t2 = self.turns_to_joint_deg(1, turns1)
            self.fk_t1_input.value = round(t1, 2)
            self.fk_t2_input.value = round(t2, 2)
            E, P1, P2 = forward_kinematics(t1, t2, self.params)
            self.fk_result_label.text = "X={:.2f} mm   Y={:.2f} mm".format(E[0], E[1])
            self.viz.set_content(self.render_svg(P1, P2, E))
        except Exception as e:
            self.log("Read live angles failed: {}".format(e))

    # ------------------------------------------------------------------
    # Calibration / homing / safety
    # ------------------------------------------------------------------
    async def clear_errors(self):
        if not self.require_connected():
            return
        await run.io_bound(self._locked_call, self.odrv0.clear_errors)
        self.log("Errors cleared.")

    def _run_state_blocking(self, axis, state, name, timeout=30):
        with self._odrv_lock:
            axis.requested_state = state
        start = time.time()
        while True:
            with self._odrv_lock:
                current = axis.current_state
            if current == AXIS_STATE_IDLE:
                break
            if time.time() - start > timeout:
                self.log("{}: TIMEOUT waiting for IDLE".format(name))
                return False
            time.sleep(0.1)
        with self._odrv_lock:
            axis_err, motor_err, encoder_err = axis.error, axis.motor.error, axis.encoder.error
        ok = (axis_err == 0 and motor_err == 0 and encoder_err == 0)
        if not ok:
            self.log("{}: error after state -> axis.error={}, motor.error={}, encoder.error={}".format(
                name, axis_err, motor_err, encoder_err))
        else:
            self.log("{}: OK".format(name))
        return ok

    def _calibrate_axis_blocking(self, idx):
        axis = self.odrv0.axis0 if idx == 0 else self.odrv0.axis1
        name = "axis{}".format(idx)
        self.log("{}: starting motor calibration...".format(name))
        if not self._run_state_blocking(axis, AXIS_STATE_MOTOR_CALIBRATION, name):
            return
        self.log("{}: starting encoder offset calibration...".format(name))
        self._run_state_blocking(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, name)

    async def calibrate_axis(self, idx):
        if not self.require_connected():
            return
        await run.io_bound(self._calibrate_axis_blocking, idx)

    async def calibrate_both(self):
        if not self.require_connected():
            return
        await run.io_bound(self._calibrate_axis_blocking, 0)
        await run.io_bound(self._calibrate_axis_blocking, 1)

    async def enable_closed_loop_both(self):
        if not self.require_connected():
            return

        def _do():
            try:
                self.odrv0.axis0.config.enable_watchdog = False
                self.odrv0.axis1.config.enable_watchdog = False
            except Exception:
                pass
            self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            self.odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.odrv0.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        try:
            await run.io_bound(self._locked_call, _do)
            self.log("Requested CLOSED_LOOP_CONTROL (position control, passthrough input) on both axes.")
        except Exception as e:
            self.log("Enable closed loop failed: {}".format(e))

    # ------------------------------------------------------------------
    # SPI absolute encoder interface config
    #
    # Sets which GPIO each axis's encoder chip-select line is on and which
    # SPI absolute encoder protocol to decode. Unlike the gain/geometry
    # settings on this tab, a mode/CS change only takes effect after
    # save_configuration() + reboot, so this is a separate confirm-then-save
    # flow rather than a plain "Apply Config" field.
    # ------------------------------------------------------------------
    async def read_spi_encoder_config(self):
        if not self.require_connected():
            return

        def _do():
            return (
                self.odrv0.axis0.encoder.config.mode,
                getattr(self.odrv0.axis0.encoder.config, "abs_spi_cs_gpio_pin", None),
                self.odrv0.axis1.encoder.config.mode,
                getattr(self.odrv0.axis1.encoder.config, "abs_spi_cs_gpio_pin", None),
            )

        try:
            mode0, cs0, mode1, cs1 = await run.io_bound(self._locked_call, _do)
        except Exception as e:
            self.log("Read SPI encoder config failed: {}".format(e))
            return

        label0 = self._spi_mode_label(mode0) if mode0 in SPI_ENCODER_MODE_OPTIONS.values() else None
        label1 = self._spi_mode_label(mode1) if mode1 in SPI_ENCODER_MODE_OPTIONS.values() else None
        if label0 is not None:
            self.cfg_spi_mode0.value = label0
        if cs0 is not None:
            self.cfg_spi_cs0.value = cs0
        if label1 is not None:
            self.cfg_spi_mode1.value = label1
        if cs1 is not None:
            self.cfg_spi_cs1.value = cs1

        self.log("Read SPI encoder config: axis0 mode=0x{:x} cs_gpio={}, "
                  "axis1 mode=0x{:x} cs_gpio={}".format(mode0, cs0, mode1, cs1))

    def confirm_apply_spi_encoder_config(self):
        if not self.require_connected():
            return
        with ui.dialog() as dialog, ui.card():
            ui.label(
                "This writes the SPI absolute encoder mode and chip-select "
                "GPIO pin for both axes, saves to flash, and reboots the "
                "ODrive. Motor/encoder calibration will need to be re-run "
                "after this if the encoder chip or wiring actually changed. "
                "Continue?"
            )
            with ui.row():
                ui.button("Cancel", on_click=dialog.close)
                ui.button("Apply & Save & Reboot", color="red",
                          on_click=lambda: self._do_apply_spi_encoder_config(dialog))
        dialog.open()

    async def _do_apply_spi_encoder_config(self, dialog):
        dialog.close()
        if not self.require_connected():
            return
        try:
            mode0 = SPI_ENCODER_MODE_OPTIONS[self.cfg_spi_mode0.value]
            cs0 = int(self.cfg_spi_cs0.value)
            mode1 = SPI_ENCODER_MODE_OPTIONS[self.cfg_spi_mode1.value]
            cs1 = int(self.cfg_spi_cs1.value)
        except (KeyError, TypeError, ValueError) as e:
            ui.notify("Invalid SPI encoder config: {}".format(e), type="negative")
            return

        def _do():
            self.odrv0.axis0.encoder.config.mode = mode0
            self.odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = cs0
            self.odrv0.axis1.encoder.config.mode = mode1
            self.odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = cs1

        try:
            await run.io_bound(self._locked_call, _do)
            self.log("SPI encoder config written: axis0 mode=0x{:x} cs_gpio={}, "
                      "axis1 mode=0x{:x} cs_gpio={} (not yet saved).".format(mode0, cs0, mode1, cs1))
        except Exception as e:
            self.log("Failed to write SPI encoder config: {}".format(e))
            return

        self.spi_cfg[0] = {"cs_gpio": cs0, "mode": mode0}
        self.spi_cfg[1] = {"cs_gpio": cs1, "mode": mode1}
        self.save_dashboard_config(silent=True)

        # Stop poll_live from attempting new reads right before the reboot.
        self.connected = False
        self.status_label.text = "Saving & rebooting..."
        try:
            await run.io_bound(self._locked_call, self.odrv0.save_configuration)
            self.log("Configuration saved with new SPI encoder config. ODrive is rebooting; "
                      "reconnect once it comes back, then re-run calibration if the encoder "
                      "chip/wiring actually changed.")
        except Exception as e:
            # save_configuration commonly raises because the device reboots
            # immediately after replying - treat as expected.
            self.log("Save configuration sent (connection drop on reboot is expected): {}".format(e))
        self.odrv0 = None
        self.status_label.text = "Not connected (rebooting)"
        self.status_label.classes(remove="text-green-600", add="text-red-600")

    async def idle_both(self):
        if not self.require_connected():
            return

        def _do():
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            self.odrv0.axis1.requested_state = AXIS_STATE_IDLE

        await run.io_bound(self._locked_call, _do)
        self.log("Both axes set to IDLE.")

    async def sync_reference_now(self):
        """
        Reads the current encoder position on EACH axis independently and
        sets that axis's own home_angle_deg so its CURRENT physical pose
        reads as joint angle SYNC_REFERENCE_DEG (90 deg from the +X axis,
        per this script's theta convention). Axis0 and axis1 each get their
        own reference value - there is no averaging or cross-axis
        comparison, so it doesn't matter if the two links aren't physically
        symmetric when you press this; each motor is simply told "wherever
        you are right now IS 90 deg" independently of the other.

        This is the only zero-reference mechanism in the dashboard - there's
        no separate "apply an arbitrary home angle" step and no
        endstop-based homing state, since this build has no limit switches.
        """
        if not self.require_connected():
            return

        try:
            turns0, turns1 = await run.io_bound(self._locked_call, self._read_encoder_turns)
        except Exception as e:
            self.log("Could not read encoder positions: {}".format(e))
            return

        cfg0 = self.axis_cfg[0]
        cfg1 = self.axis_cfg[1]
        norm0 = ((turns0 - cfg0["offset_turns"]) / cfg0["gear_ratio"]) * 360.0 / cfg0["direction"]
        norm1 = ((turns1 - cfg1["offset_turns"]) / cfg1["gear_ratio"]) * 360.0 / cfg1["direction"]

        self.home_angle_deg[0] = SYNC_REFERENCE_DEG - norm0
        self.home_angle_deg[1] = SYNC_REFERENCE_DEG - norm1

        # Immediately snap the simulation to the physical pose using the
        # just-captured references, instead of waiting up to one poll_live
        # tick (0.2s) for it to catch up - this is the whole point of the
        # "sync" action, so it should be instant and visible right away.
        self._force_viz_refresh(turns0, turns1)

        msg = ("Synced! Current pose is now joint angle {:.0f} deg on both axes, "
               "set independently (axis0 home_angle_deg={:.3f}, axis1 "
               "home_angle_deg={:.3f}).").format(
            SYNC_REFERENCE_DEG, self.home_angle_deg[0], self.home_angle_deg[1])
        self.log(msg)
        self.sync_status_label.text = msg
        self.save_dashboard_config(silent=True)
        ui.notify("Synced - current pose is now {:.0f} deg on both axes.".format(SYNC_REFERENCE_DEG),
                  type="positive")

    # ------------------------------------------------------------------
    # Persistent calibration (survives ODrive power-cycle)
    # ------------------------------------------------------------------
    async def read_startup_flags(self):
        if not self.require_connected():
            return

        def _do():
            out = {}
            for idx in (0, 1):
                axis = self.odrv0.axis0 if idx == 0 else self.odrv0.axis1
                out[idx] = {
                    "motor_pre_calibrated": getattr(axis.motor.config, "pre_calibrated", None),
                    "encoder_pre_calibrated": getattr(axis.encoder.config, "pre_calibrated", None),
                    "startup_closed_loop_control": getattr(axis.config, "startup_closed_loop_control", None),
                    "startup_homing": getattr(axis.config, "startup_homing", None),
                    "startup_motor_calibration": getattr(axis.config, "startup_motor_calibration", None),
                    "startup_encoder_offset_calibration": getattr(
                        axis.config, "startup_encoder_offset_calibration", None),
                }
            return out

        try:
            flags = await run.io_bound(self._locked_call, _do)
        except Exception as e:
            self.log("Read startup flags failed: {}".format(e))
            return

        lines = []
        for idx in (0, 1):
            f = flags[idx]
            lines.append(
                "axis{}: motor.pre_calibrated={}  encoder.pre_calibrated={}  "
                "startup_closed_loop_control={}  startup_homing={}  "
                "startup_motor_cal={}  startup_encoder_offset_cal={}".format(
                    idx, f["motor_pre_calibrated"], f["encoder_pre_calibrated"],
                    f["startup_closed_loop_control"], f["startup_homing"],
                    f["startup_motor_calibration"], f["startup_encoder_offset_calibration"]))
        self.startup_flags_label.text = "\n".join(lines)
        for line in lines:
            self.log(line)

    def confirm_mark_precalibrated(self):
        if not self.require_connected():
            return
        with ui.dialog() as dialog, ui.card():
            ui.label(
                "This marks BOTH axes' motor + encoder calibration as "
                "pre-verified, sets the selected startup behavior, saves to "
                "flash, and reboots the ODrive. Only do this right after a "
                "successful 'Calibrate Both' in THIS session (no motor/"
                "encoder errors) - continue?"
            )
            with ui.row():
                ui.button("Cancel", on_click=dialog.close)
                ui.button("Mark & Save & Reboot", color="red",
                          on_click=lambda: self._do_mark_precalibrated(dialog))
        dialog.open()

    async def _do_mark_precalibrated(self, dialog):
        dialog.close()
        if not self.require_connected():
            return

        def _check_ok():
            a0, a1 = self.odrv0.axis0, self.odrv0.axis1
            return (a0.error == 0 and a0.motor.error == 0 and a0.encoder.error == 0 and
                    a1.error == 0 and a1.motor.error == 0 and a1.encoder.error == 0)

        try:
            ok = await run.io_bound(self._locked_call, _check_ok)
        except Exception as e:
            self.log("Could not check axis errors before marking pre-calibrated: {}".format(e))
            return
        if not ok:
            self.log("Refusing to mark pre-calibrated: at least one axis currently reports an "
                      "error. Clear errors and/or recalibrate first, then try again.")
            ui.notify("Axis errors present - not marking pre-calibrated.", type="negative")
            return

        startup_closed_loop = bool(self.startup_closed_loop_checkbox.value)

        def _apply():
            for idx in (0, 1):
                axis = self.odrv0.axis0 if idx == 0 else self.odrv0.axis1
                axis.motor.config.pre_calibrated = True
                axis.encoder.config.pre_calibrated = True
                axis.config.startup_motor_calibration = False
                axis.config.startup_encoder_offset_calibration = False
                axis.config.startup_closed_loop_control = startup_closed_loop
                # No endstops on this build, so startup-homing-to-endstop is
                # never applicable - keep it explicitly disabled.
                if hasattr(axis.config, "startup_homing"):
                    axis.config.startup_homing = False

        try:
            await run.io_bound(self._locked_call, _apply)
            self.log("Pre-calibrated + startup flags set on both axes (not yet saved).")
        except Exception as e:
            self.log("Failed to set pre-calibrated flags: {}".format(e))
            return

        # Stop poll_live from attempting new reads right before the reboot.
        self.connected = False
        self.status_label.text = "Saving & rebooting..."
        try:
            await run.io_bound(self._locked_call, self.odrv0.save_configuration)
            self.log("Configuration saved with pre-calibrated flags. ODrive is rebooting; "
                      "after it comes back it should skip re-calibration. Since these are SPI "
                      "absolute encoders, the turn count/position is retained across the "
                      "reboot - no re-homing needed.")
        except Exception as e:
            # save_configuration commonly raises because the device reboots
            # immediately after replying - treat as expected.
            self.log("Save configuration sent (connection drop on reboot is expected): {}".format(e))

        self.odrv0 = None
        self.status_label.text = "Not connected (rebooting)"
        self.status_label.classes(remove="text-green-600", add="text-red-600")

    def confirm_clear_precalibrated(self):
        if not self.require_connected():
            return
        with ui.dialog() as dialog, ui.card():
            ui.label(
                "This clears the pre-calibrated / startup-closed-loop flags "
                "(forcing motor + encoder calibration again on next "
                "power-up), saves to flash, and reboots. Continue?"
            )
            with ui.row():
                ui.button("Cancel", on_click=dialog.close)
                ui.button("Clear & Save & Reboot", color="red",
                          on_click=lambda: self._do_clear_precalibrated(dialog))
        dialog.open()

    async def _do_clear_precalibrated(self, dialog):
        dialog.close()
        if not self.require_connected():
            return

        def _apply():
            for idx in (0, 1):
                axis = self.odrv0.axis0 if idx == 0 else self.odrv0.axis1
                axis.motor.config.pre_calibrated = False
                axis.encoder.config.pre_calibrated = False
                axis.config.startup_closed_loop_control = False
                if hasattr(axis.config, "startup_homing"):
                    axis.config.startup_homing = False

        try:
            await run.io_bound(self._locked_call, _apply)
            self.log("Pre-calibrated + startup-closed-loop flags cleared on both axes "
                      "(not yet saved).")
        except Exception as e:
            self.log("Failed to clear pre-calibrated flags: {}".format(e))
            return

        # Stop poll_live from attempting new reads right before the reboot.
        self.connected = False
        self.status_label.text = "Saving & rebooting..."
        try:
            await run.io_bound(self._locked_call, self.odrv0.save_configuration)
            self.log("Configuration saved with calibration flags cleared. ODrive is rebooting.")
        except Exception as e:
            self.log("Save configuration sent (connection drop on reboot is expected): {}".format(e))

        self.odrv0 = None
        self.status_label.text = "Not connected (rebooting)"
        self.status_label.classes(remove="text-green-600", add="text-red-600")

    def _on_key(self, e):
        """Global keyboard handler (see ui.keyboard(...) in build_ui).
        Fires on every keydown/keyup anywhere on the page, including while
        an input/select/textarea has focus. Only Escape does anything."""
        try:
            is_escape = (e.key == "Escape") or getattr(e.key, "escape", False)
        except Exception:
            is_escape = False
        if e.action.keydown and is_escape:
            self.emergency_stop()
            ui.notify("EMERGENCY STOP (Esc key)", type="negative", position="top")

    def emergency_stop(self):
        # Deliberately synchronous / not routed through run.io_bound so it
        # fires immediately even if other background operations are busy.
        self._motion_stop_event.set()
        self._vel_stop_event.set()
        if self.motion_task is not None and not self.motion_task.done():
            self.motion_task.cancel()
        if not self.connected or self.odrv0 is None:
            return
        # Every other hardware write only ever holds _odrv_lock very briefly
        # (a single attribute set, not a whole multi-second operation), so
        # this should essentially never actually wait. The short timeout is
        # a safety net, not the primary mechanism: if the lock somehow isn't
        # free in time, write anyway rather than let an E-stop do nothing.
        acquired = self._odrv_lock.acquire(timeout=0.2)
        try:
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
            # Disable any velocity-mode watchdog so a later resume that isn't
            # feeding it can't immediately re-trip the axis.
            try:
                self.odrv0.axis0.config.enable_watchdog = False
                self.odrv0.axis1.config.enable_watchdog = False
            except Exception:
                pass
            self.log("EMERGENCY STOP: both axes set to IDLE. Both axes are now "
                      "un-powered - commanding a new move will NOT make them move "
                      "again until you press 'Resume After E-Stop' (or manually "
                      "Clear Errors + Enable Closed Loop Control on the "
                      "Calibration tab).")
        except Exception as e:
            self.log("E-stop failed: {}".format(e))
        finally:
            if acquired:
                self._odrv_lock.release()

    async def resume_from_estop(self):
        """
        One-click recovery after EMERGENCY STOP (or any other time the axes
        ended up IDLE and won't move): clears any leftover
        error/stop-event/motion-active state left over from the E-stop,
        clears axis errors on the ODrive, and re-requests
        CLOSED_LOOP_CONTROL on both axes. Pressing 'Enable Closed Loop
        Control (Both)' on the Calibration tab does the ODrive part of this
        too - this button just also resets the dashboard's own motion-state
        bookkeeping and clears errors first, since simply re-enabling
        closed loop after an E-stop often still needs errors cleared first
        or the axis will refuse to re-enter closed loop.
        """
        if not self.require_connected():
            return

        # Reset dashboard-side motion bookkeeping left over from the E-stop
        # so a fresh move isn't immediately treated as "already cancelled".
        self._motion_stop_event.clear()
        self._motion_active = False

        def _do():
            self.odrv0.clear_errors()
            try:
                self.odrv0.axis0.config.enable_watchdog = False
                self.odrv0.axis1.config.enable_watchdog = False
            except Exception:
                pass
            self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            self.odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.odrv0.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        try:
            await run.io_bound(self._locked_call, _do)
        except Exception as e:
            self.log("Resume failed: {}".format(e))
            ui.notify("Resume failed: {}".format(e), type="negative")
            return

        # Re-arming after an E-stop is worth double-checking, not just
        # assuming the request succeeded - if the axis still reports an
        # error, closed loop control will have silently NOT engaged.
        def _check():
            return (self.odrv0.axis0.current_state, self.odrv0.axis0.error,
                     self.odrv0.axis1.current_state, self.odrv0.axis1.error)

        try:
            state0, err0, state1, err1 = await run.io_bound(self._locked_call, _check)
        except Exception as e:
            self.log("Resume: could not verify axis state after re-enabling: {}".format(e))
            return

        ok0 = (state0 == AXIS_STATE_CLOSED_LOOP_CONTROL and err0 == 0)
        ok1 = (state1 == AXIS_STATE_CLOSED_LOOP_CONTROL and err1 == 0)
        if ok0 and ok1:
            self.log("Resumed: errors cleared, both axes back in CLOSED_LOOP_CONTROL. "
                      "Normal moves should work again.")
            ui.notify("Resumed - motors are back online.", type="positive")
        else:
            self.log("Resume attempted but axis state still not clean: "
                      "axis0 state={} error={}, axis1 state={} error={}. Check 'Show Errors' - "
                      "a hardware fault (overcurrent, overvoltage, etc.) may need attention "
                      "before the motors will re-engage.".format(state0, err0, state1, err1))
            ui.notify("Axes did not cleanly re-enter closed loop - check the log / Show Errors.",
                      type="warning")

    async def show_errors(self):
        if not self.require_connected():
            return

        def _do():
            try:
                from odrive.utils import dump_errors
                dump_errors(self.odrv0)
                return "Errors dumped to console (see terminal)."
            except Exception:
                a0, a1 = self.odrv0.axis0, self.odrv0.axis1
                return ("axis0: error={}, motor.error={}, encoder.error={} | "
                        "axis1: error={}, motor.error={}, encoder.error={}").format(
                    a0.error, a0.motor.error, a0.encoder.error,
                    a1.error, a1.motor.error, a1.encoder.error)

        msg = await run.io_bound(self._locked_call, _do)
        self.log(msg)

    # ------------------------------------------------------------------
    # PID Tuning actions
    # ------------------------------------------------------------------
    def _nudge_gain(self, field, factor):
        try:
            current = float(field.value or 0.0)
        except (TypeError, ValueError):
            current = 0.0
        field.value = round(current * factor, 6)

    async def read_gains(self, idx):
        if not self.require_connected():
            return
        axis = self.odrv0.axis0 if idx == 0 else self.odrv0.axis1

        def _do():
            c = axis.controller.config
            return c.pos_gain, c.vel_gain, c.vel_integrator_gain, c.vel_limit, axis.motor.config.current_lim

        try:
            pos_gain, vel_gain, vel_int_gain, vel_limit, current_lim = await run.io_bound(self._locked_call, _do)
        except Exception as e:
            self.log("Read gains failed: {}".format(e))
            return

        if idx == 0:
            self.pid0_pos_gain.value = pos_gain
            self.pid0_vel_gain.value = vel_gain
            self.pid0_vel_int_gain.value = vel_int_gain
            self.pid0_vel_limit.value = vel_limit
            self.pid0_current_lim.value = current_lim
        else:
            self.pid1_pos_gain.value = pos_gain
            self.pid1_vel_gain.value = vel_gain
            self.pid1_vel_int_gain.value = vel_int_gain
            self.pid1_vel_limit.value = vel_limit
            self.pid1_current_lim.value = current_lim

        self.log("axis{}: read pos_gain={:.4f}, vel_gain={:.6f}, vel_integrator_gain={:.6f}, "
                  "vel_limit={:.3f}, current_lim={:.2f}".format(
                      idx, pos_gain, vel_gain, vel_int_gain, vel_limit, current_lim))

    async def apply_gains(self, idx):
        if not self.require_connected():
            return
        if idx == 0:
            fields = (self.pid0_pos_gain, self.pid0_vel_gain, self.pid0_vel_int_gain,
                      self.pid0_vel_limit, self.pid0_current_lim)
        else:
            fields = (self.pid1_pos_gain, self.pid1_vel_gain, self.pid1_vel_int_gain,
                      self.pid1_vel_limit, self.pid1_current_lim)
        try:
            pos_gain = float(fields[0].value)
            vel_gain = float(fields[1].value)
            vel_int_gain = float(fields[2].value)
            vel_limit = float(fields[3].value)
            current_lim = float(fields[4].value)
        except (TypeError, ValueError):
            ui.notify("All gain fields must be numbers.", type="negative")
            return

        axis = self.odrv0.axis0 if idx == 0 else self.odrv0.axis1

        def _do():
            axis.controller.config.pos_gain = pos_gain
            axis.controller.config.vel_gain = vel_gain
            axis.controller.config.vel_integrator_gain = vel_int_gain
            axis.controller.config.vel_limit = vel_limit
            axis.motor.config.current_lim = current_lim

        try:
            await run.io_bound(self._locked_call, _do)
            self.log("axis{}: gains applied (not yet saved to flash).".format(idx))
            ui.notify("axis{} gains applied.".format(idx), type="positive")
        except Exception as e:
            self.log("Apply gains failed: {}".format(e))

    def confirm_save_config(self):
        with ui.dialog() as dialog, ui.card():
            ui.label("This saves current config to flash and reboots the ODrive. "
                      "The connection will drop and you'll need to reconnect. Continue?")
            with ui.row():
                ui.button("Cancel", on_click=dialog.close)
                ui.button("Save & Reboot", color="red", on_click=lambda: self._do_save_config(dialog))
        dialog.open()

    async def _do_save_config(self, dialog):
        dialog.close()
        if not self.require_connected():
            return
        # Stop poll_live from even attempting a new read before we ask the
        # ODrive to reboot - the _odrv_lock already serializes actual USB
        # access, but this avoids queuing pointless work during the reboot.
        self.connected = False
        self.status_label.text = "Saving & rebooting..."
        try:
            await run.io_bound(self._locked_call, self.odrv0.save_configuration)
            self.log("Configuration saved. ODrive is rebooting; reconnect once it comes back.")
        except Exception as e:
            # save_configuration commonly raises because the device reboots
            # immediately after replying - treat as expected.
            self.log("Save configuration sent (connection drop on reboot is expected): {}".format(e))
        self.odrv0 = None
        self.status_label.text = "Not connected (rebooting)"
        self.status_label.classes(remove="text-green-600", add="text-red-600")

    def clear_step_history(self):
        self.step_test_history = []
        self.step_metrics_label.text = ""
        self.step_chart.set_content("")
        self.log("Step-response history cleared.")

    def _analyze_step_response(self, samples, start_pos, target):
        """Rough overshoot % and 2%-settling-time estimate, purely to give a
        quick, comparable number when nudging gains - not a substitute for
        looking at the overlaid curves."""
        if not samples:
            return None
        delta = target - start_pos
        if abs(delta) < 1e-9:
            return {"overshoot_pct": 0.0, "settle_time_s": 0.0, "final_error": samples[-1][1] - target}

        if delta > 0:
            peak = max(y for _, y in samples)
            overshoot = max(0.0, (peak - target) / delta * 100.0)
        else:
            peak = min(y for _, y in samples)
            overshoot = max(0.0, (target - peak) / delta * 100.0)

        tol = 0.02 * abs(delta)
        settle_t = 0.0
        for t, y in samples:
            if abs(y - target) > tol:
                settle_t = t

        return {
            "overshoot_pct": overshoot,
            "settle_time_s": settle_t,
            "final_error": samples[-1][1] - target,
        }

    async def run_step_test(self):
        if not self.require_connected():
            return
        idx = self.step_axis_select.value
        axis = self.odrv0.axis0 if idx == 0 else self.odrv0.axis1
        try:
            step = float(self.step_size_input.value)
            duration = float(self.step_duration_input.value)
        except (TypeError, ValueError):
            ui.notify("Step size / duration must be numbers.", type="negative")
            return

        def _get_pos():
            return axis.encoder.pos_estimate

        try:
            start_pos = await run.io_bound(self._locked_call, _get_pos)
        except Exception as e:
            self.log("Step test failed to read start position: {}".format(e))
            return

        target = start_pos + step
        self.log("axis{}: step test, start={:.4f} turns, target={:.4f} turns".format(idx, start_pos, target))

        def _command():
            axis.controller.input_pos = target

        await run.io_bound(self._locked_call, _command)

        samples = []
        t0 = time.time()
        interval = 0.02
        while (time.time() - t0) < duration:
            try:
                pos = await run.io_bound(self._locked_call, _get_pos)
            except Exception:
                break
            samples.append((time.time() - t0, pos))
            await asyncio.sleep(interval)

        self.log("axis{}: step test collected {} samples.".format(idx, len(samples)))

        # Snapshot the gains in effect right now, so the overlay legend shows
        # what actually produced each curve.
        try:
            gains_label = "pos={:.3f} vel={:.5f} vel_i={:.5f}".format(
                axis.controller.config.pos_gain,
                axis.controller.config.vel_gain,
                axis.controller.config.vel_integrator_gain,
            )
        except Exception:
            gains_label = "run {}".format(len(self.step_test_history) + 1)

        analysis = self._analyze_step_response(samples, start_pos, target)
        if analysis is not None:
            self.step_metrics_label.text = (
                "Latest run [{}]: overshoot={:.1f}%   settle(2%)={:.2f}s   "
                "final error={:.5f} turns".format(
                    gains_label, analysis["overshoot_pct"], analysis["settle_time_s"],
                    analysis["final_error"])
            )
            self.log("Step test analysis: overshoot={:.1f}%, settle_time={:.2f}s, "
                      "final_error={:.5f} turns".format(
                          analysis["overshoot_pct"], analysis["settle_time_s"], analysis["final_error"]))

        colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
        self.step_test_history.append({
            "samples": samples, "start_pos": start_pos, "target": target, "label": gains_label,
        })
        self.step_test_history = self.step_test_history[-4:]
        for i, run_data in enumerate(self.step_test_history):
            run_data["color"] = colors[i % len(colors)]

        self.step_chart.set_content(self.render_step_chart_overlay(self.step_test_history))

    def render_step_chart_overlay(self, history):
        w, h = 620, 300
        pad_l, pad_r, pad_t, pad_b = 40, 10, 16, 40
        if not history:
            return '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}"></svg>'.format(w, h)

        all_t, all_y = [], []
        for run_data in history:
            for t, y in run_data["samples"]:
                all_t.append(t)
                all_y.append(y)
            all_y.append(run_data["target"])
            all_y.append(run_data["start_pos"])

        if not all_t:
            return '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}"></svg>'.format(w, h)

        tmin, tmax = 0.0, max(all_t)
        ymin, ymax = min(all_y), max(all_y)
        if ymax - ymin < 1e-6:
            ymax = ymin + 1e-6

        def to_px(t, y):
            px = pad_l + (t - tmin) / (tmax - tmin + 1e-9) * (w - pad_l - pad_r)
            py = h - pad_b - (y - ymin) / (ymax - ymin) * (h - pad_t - pad_b)
            return px, py

        parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" '
                 'style="background:#fff;border:1px solid #ccc">'.format(w, h)]
        parts.append('<text x="{}" y="14" font-size="11">position (turns) vs time (s) '
                      '- last {} run(s), most recent last in legend</text>'.format(pad_l, len(history)))

        latest = history[-1]
        tx0, ty0 = to_px(tmin, latest["target"])
        tx1, ty1 = to_px(tmax, latest["target"])
        parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
                      'stroke="red" stroke-width="1.2" stroke-dasharray="4,3"/>'.format(tx0, ty0, tx1, ty1))
        parts.append('<text x="{:.1f}" y="{:.1f}" font-size="10" fill="red">target (latest)</text>'.format(
            tx1 - 90, ty1 - 4))

        legend_y = h - pad_b + 14
        for i, run_data in enumerate(history):
            pts_px = [to_px(t, y) for t, y in run_data["samples"]]
            if pts_px:
                points_str = " ".join("{:.1f},{:.1f}".format(px, py) for px, py in pts_px)
                parts.append('<polyline points="{}" fill="none" stroke="{}" stroke-width="2"/>'.format(
                    points_str, run_data["color"]))
            lx = pad_l + i * 145
            parts.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="{}" stroke-width="3"/>'.format(
                lx, legend_y, lx + 14, legend_y, run_data["color"]))
            parts.append('<text x="{}" y="{}" font-size="9">{}</text>'.format(
                lx + 18, legend_y + 3, run_data["label"]))

        parts.append('</svg>')
        return "".join(parts)

    # ------------------------------------------------------------------
    # Config apply
    # ------------------------------------------------------------------
    def apply_config(self):
        try:
            self.params["L0"] = float(self.cfg_L0.value)
            self.params["l1a"] = float(self.cfg_l1a.value)
            self.params["l2a"] = float(self.cfg_l2a.value)
            self.params["l1b"] = float(self.cfg_l1b.value)
            self.params["l2b"] = float(self.cfg_l2b.value)

            self.axis_cfg[0]["gear_ratio"] = float(self.cfg_gear0.value)
            self.axis_cfg[0]["offset_turns"] = float(self.cfg_off0.value)
            self.axis_cfg[0]["direction"] = float(self.cfg_dir0.value)
            self.axis_cfg[1]["gear_ratio"] = float(self.cfg_gear1.value)
            self.axis_cfg[1]["offset_turns"] = float(self.cfg_off1.value)
            self.axis_cfg[1]["direction"] = float(self.cfg_dir1.value)

            self.traj_cfg["max_vel_deg_s"] = float(self.cfg_max_vel.value)
            self.traj_cfg["max_accel_deg_s2"] = float(self.cfg_max_accel.value)

            self.vel_cfg["loop_hz"] = max(5.0, float(self.cfg_vel_loop_hz.value))
            self.vel_cfg["joint_vel_cap_deg_s"] = float(self.cfg_vel_joint_cap.value)
            self.vel_cfg["joint_accel_cap_deg_s2"] = float(self.cfg_vel_accel_cap.value)
            self.vel_cfg["max_cart_speed_mm_s"] = float(self.cfg_vel_cart_speed.value)
            self.vel_cfg["pos_kp"] = float(self.cfg_vel_pos_kp.value)
            self.vel_cfg["pos_tol_mm"] = float(self.cfg_vel_pos_tol.value)
            self.vel_cfg["manip_soft_deg_mm"] = float(self.cfg_vel_manip_soft.value)
            self.vel_cfg["manip_hard_deg_mm"] = float(self.cfg_vel_manip_hard.value)
            self.vel_cfg["watchdog_s"] = max(0.02, float(self.cfg_vel_watchdog.value))
            self.vel_cfg["deadman_s"] = max(0.05, float(self.cfg_vel_deadman.value))

            self.log("Config applied.")
            ui.notify("Config applied.", type="positive")
            self.save_dashboard_config(silent=True)
        except (TypeError, ValueError) as e:
            ui.notify("Invalid config: {}".format(e), type="negative")

    # ------------------------------------------------------------------
    # Live polling loop (idle only - motion/velocity threads feed viz +
    # telemetry directly while active)
    # ------------------------------------------------------------------
    async def poll_live(self):
        if not (self.connected and self.odrv0 is not None):
            return
        if self._motion_active:
            return
        try:
            p0, p1, v0, v1, c0, c1 = await run.io_bound(self._locked_call, self._read_full_telemetry)
            t1 = self.turns_to_joint_deg(0, p0)
            t2 = self.turns_to_joint_deg(1, p1)
            self.live_joint_label.text = "theta1={:.2f} deg   theta2={:.2f} deg".format(t1, t2)

            E, P1, P2 = forward_kinematics(t1, t2, self.params)
            self.ee_label.text = "End effector: X={:.2f} mm   Y={:.2f} mm".format(E[0], E[1])
            self.viz.set_content(self.render_svg(P1, P2, E))
            self._enqueue_telemetry_from_turns(p0, p1, v0, v1, c0, c1)
        except Exception:
            pass  # unreachable pose / transient read error, skip this frame

    # ------------------------------------------------------------------
    # Visualization (renders an SVG string)
    # ------------------------------------------------------------------
    @staticmethod
    def _circle_path_d(cx, cy, r):
        """SVG path data tracing a full circle (as two semicircular arcs),
        centered at (cx, cy) with radius r, in whatever coordinate space
        the caller is working in (here: pixels)."""
        return ("M {:.2f},{:.2f} A {:.2f},{:.2f} 0 1,0 {:.2f},{:.2f} "
                "A {:.2f},{:.2f} 0 1,0 {:.2f},{:.2f} Z").format(
            cx + r, cy, r, r, cx - r, cy, r, r, cx + r, cy)

    @classmethod
    def _annulus_path_d(cls, cx, cy, r_outer, r_inner):
        """SVG path data for a ring (annulus): outer circle with an inner
        circle subtracted via fill-rule="evenodd". If r_inner is ~0 (i.e.
        the two link lengths are equal, so the chain can fold all the way
        to the center), this degenerates to a plain filled disk - which is
        the correct reachable region in that case too."""
        if r_outer <= 0:
            return ""
        d = cls._circle_path_d(cx, cy, r_outer)
        if r_inner > 1e-6:
            d += " " + cls._circle_path_d(cx, cy, r_inner)
        return d

    def render_svg(self, P1, P2, E):
        w, h = 640, 520
        cx, cy = w / 2, h * 0.8
        scale = 0.5  # px per mm

        def to_px(pt):
            return cx + pt[0] * scale, cy - pt[1] * scale

        L0 = self.params["L0"]
        A = (-L0 / 2.0, 0.0)
        B = (L0 / 2.0, 0.0)
        Apx = to_px(A)
        Bpx = to_px(B)

        parts = []
        parts.append('<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" '
                      'style="background:#ffffff;border:1px solid #ccc">'.format(w, h))
        parts.append('<defs><marker id="arrowhead" markerWidth="8" markerHeight="8" refX="6" refY="3" '
                      'orient="auto"><path d="M0,0 L6,3 L0,6 Z" fill="purple"/></marker></defs>')

        # Reachable workspace overlay: the end effector is reachable at a
        # point only if BOTH arms individually can reach it, i.e. the point
        # lies within an annulus (ring) around each base pivot - inner
        # radius |l1-l2| (chain folded back on itself), outer radius l1+l2
        # (chain fully extended). The true reachable region is the
        # INTERSECTION of the two annuli, which we get for free by filling
        # arm A's annulus and clipping it with arm B's annulus (clip-path
        # intersects with the filled shape).
        if self.display_cfg.get("show_workspace", True):
            r_outer_a = (self.params["l1a"] + self.params["l2a"]) * scale
            r_inner_a = abs(self.params["l1a"] - self.params["l2a"]) * scale
            r_outer_b = (self.params["l1b"] + self.params["l2b"]) * scale
            r_inner_b = abs(self.params["l1b"] - self.params["l2b"]) * scale
            annulus_a_d = self._annulus_path_d(Apx[0], Apx[1], r_outer_a, r_inner_a)
            annulus_b_d = self._annulus_path_d(Bpx[0], Bpx[1], r_outer_b, r_inner_b)
            if annulus_a_d and annulus_b_d:
                parts.append('<clipPath id="workspaceClipB" clipPathUnits="userSpaceOnUse">'
                              '<path d="{}" fill-rule="evenodd"/></clipPath>'.format(annulus_b_d))
                parts.append('<path d="{}" fill-rule="evenodd" fill="#bfe3ff" fill-opacity="0.55" '
                              'stroke="#8fc4e8" stroke-width="1" stroke-opacity="0.6" '
                              'clip-path="url(#workspaceClipB)"/>'.format(annulus_a_d))
                parts.append('<text x="12" y="16" font-size="11" fill="#4a90c2">Pale blue = reachable '
                              'workspace</text>')

        # base line
        parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
                      'stroke="gray" stroke-width="2" stroke-dasharray="4,2"/>'.format(
                          Apx[0], Apx[1], Bpx[0], Bpx[1]))

        # planned / preview path (drawn behind the live linkage)
        if self.planned_path and len(self.planned_path) >= 2:
            pts_px = [to_px(p) for p in self.planned_path]
            points_str = " ".join("{:.1f},{:.1f}".format(px, py) for px, py in pts_px)
            parts.append('<polyline points="{}" fill="none" stroke="orange" '
                          'stroke-width="2" stroke-dasharray="5,4"/>'.format(points_str))

        if self.waypoints_viz:
            for idx, wp in enumerate(self.waypoints_viz):
                wpx = to_px(wp)
                parts.append('<circle cx="{:.1f}" cy="{:.1f}" r="6" fill="none" '
                              'stroke="orange" stroke-width="2"/>'.format(wpx[0], wpx[1]))
                parts.append('<text x="{:.1f}" y="{:.1f}" font-size="10" fill="orange">{}</text>'.format(
                    wpx[0] + 8, wpx[1] + 4, idx + 1))

        # velocity vectors (Custom Trajectory tab): shows direction/speed
        # commanded AT each waypoint, e.g. to visually confirm it matches
        # the conveyor before trusting a run.
        if self.velocity_viz:
            arrow_scale = 0.3  # px-per-(mm/s), purely a display scale
            for (vx0, vy0, vvx, vvy) in self.velocity_viz:
                if abs(vvx) < 1e-6 and abs(vvy) < 1e-6:
                    continue
                p0 = to_px((vx0, vy0))
                p1 = to_px((vx0 + vvx * arrow_scale, vy0 + vvy * arrow_scale))
                parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
                              'stroke="purple" stroke-width="2" marker-end="url(#arrowhead)"/>'.format(
                                  p0[0], p0[1], p1[0], p1[1]))

        if P1 is not None and P2 is not None and E is not None:
            P1px = to_px(P1)
            P2px = to_px(P2)
            Epx = to_px(E)

            # proximal links
            parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
                          'stroke="blue" stroke-width="4"/>'.format(Apx[0], Apx[1], P1px[0], P1px[1]))
            parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
                          'stroke="green" stroke-width="4"/>'.format(Bpx[0], Bpx[1], P2px[0], P2px[1]))

            # distal links
            parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
                          'stroke="blue" stroke-width="3" stroke-dasharray="6,3"/>'.format(
                              P1px[0], P1px[1], Epx[0], Epx[1]))
            parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
                          'stroke="green" stroke-width="3" stroke-dasharray="6,3"/>'.format(
                              P2px[0], P2px[1], Epx[0], Epx[1]))

            # elbow points
            for pt, color, label in [(P1px, "blue", "P1"), (P2px, "green", "P2")]:
                parts.append('<circle cx="{:.1f}" cy="{:.1f}" r="5" fill="{}"/>'.format(pt[0], pt[1], color))
                parts.append('<text x="{:.1f}" y="{:.1f}" font-size="11">{}</text>'.format(
                    pt[0] + 8, pt[1] - 8, label))

            # end effector
            parts.append('<circle cx="{:.1f}" cy="{:.1f}" r="8" fill="red"/>'.format(Epx[0], Epx[1]))
            parts.append('<text x="{:.1f}" y="{:.1f}" font-size="12" font-weight="bold">'
                          'E ({:.1f}, {:.1f})</text>'.format(Epx[0] + 10, Epx[1] - 10, E[0], E[1]))

        # base pivots
        for pt, label in [(Apx, "A (axis0)"), (Bpx, "B (axis1)")]:
            parts.append('<circle cx="{:.1f}" cy="{:.1f}" r="5" fill="black"/>'.format(pt[0], pt[1]))
            parts.append('<text x="{:.1f}" y="{:.1f}" font-size="11">{}</text>'.format(
                pt[0] + 8, pt[1] - 8, label))

        parts.append('</svg>')
        return "".join(parts)

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ in {"__main__", "__mp_main__"}:
    FiveBarDashboard()
    ui.run(title="Five-Bar Linkage Dashboard", reload=False, port=8080)
