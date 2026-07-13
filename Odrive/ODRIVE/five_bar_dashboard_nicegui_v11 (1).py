"""
Five-Bar Linkage (Parallel SCARA) Control Dashboard - NiceGUI version
========================================================================

Runs as a local web app (works on Python 3.8, no tkinter needed).

Install (Python 3.8 compatible release of NiceGUI):
    pip install odrive "nicegui==1.4.37"

Run:
    python five_bar_dashboard_nicegui.py

Then open the URL it prints (default http://127.0.0.1:8080) in a browser.

Hardware: ODrive v3.6, firmware 0.5.6, odrive python lib 0.5.4
Two motors (axis0, axis1) mounted at fixed base pivots, each driving a
proximal link. Distal links connect the proximal link ends to a common
end-effector point, forming a five-bar (2 base + 2 proximal + 2 distal
meeting at the end effector) parallel linkage.

Encoders: SPI absolute encoders (NOT the older ABI/incremental quadrature
setup). Each axis's encoder is read over its own SPI chip-select line:
  axis0 (M0) -> GPIO_4
  axis1 (M1) -> GPIO_3
These are configured via axis.encoder.config.mode (one of the
ENCODER_MODE_SPI_ABS_* constants below) and
axis.encoder.config.abs_spi_cs_gpio_pin, from the "Encoder Interface (SPI)"
section of the Config tab. Because the encoders are absolute,
encoder.pos_estimate is still reported in rotations (turns) exactly like the
old incremental setup, but the absolute position is retained across ODrive
power-cycles - there is no index-search / re-homing needed after power-on
just to recover a valid turn count. turns_to_joint_deg()/joint_deg_to_turns()
below still do the rotations-to-degrees conversion using the dashboard's own
software home_angle_deg reference (a mechanical zero-pose reference for this
linkage, not an encoder index).

No limit switches / endstops are used on this build - min-endstop-based
homing has been removed. The only way to set the zero reference is the
manual "Capture Current Pose as Home" / "Apply Home Angle" workflow on the
Calibration tab.

Geometry convention:
  Motor A (axis0) is at (-L0/2, 0), Motor B (axis1) is at (+L0/2, 0).
  theta1 / theta2 are measured from +X axis, counter-clockwise, in degrees,
  and represent the angle of each PROXIMAL link relative to its motor base.

Motion planning:
  All commanded moves (single-point IK moves, joint moves, and multi-point
  paths) are streamed through a software trapezoidal velocity profile in
  JOINT space (degrees/s, degrees/s^2), synchronized so both axes start and
  stop together. This keeps commanded joint velocity bounded by design,
  which is what fixes the "exceeds velocity limit" errors that happen when
  a big position step is written directly to controller.input_pos. The
  "Move Motors (raw turns)" control on the Joint Control tab is the one
  exception - it writes input_pos directly/instantly, by design, for fine
  manual jogging with small increments.

  NOTE: the software trapezoid limits *commanded* joint velocity. The
  ODrive's own axis.controller.config.vel_limit is still the firmware's
  hard safety limit. Set the Config tab's "Max joint velocity" comfortably
  below whatever your vel_limit (turns/s) implies in deg/s for your gear
  ratio, or you can still trip an overspeed error. Use the PID Tuning tab's
  "Read AxisX" button to see the ODrive's current vel_limit.

  Trajectory streaming architecture:
  The whole trapezoidal move (however many control-loop steps it takes) now
  runs inside a SINGLE background thread (one run.io_bound call for the
  entire move), with absolute-time-scheduled writes directly to
  controller.input_pos. Earlier versions awaited run.io_bound() separately
  for every single step, which re-entered the asyncio event loop and the
  NiceGUI thread pool on every tick; that per-step scheduling/handoff
  overhead was frequently larger than the control-loop period itself, so
  the arm would visibly pause-and-jump instead of gliding smoothly - most
  noticeable on short, single-waypoint moves where there weren't enough
  steps to average the jitter out. Streaming from one dedicated thread with
  deadline-based (not sleep-after-work) timing removes that overhead and
  keeps the motor moving continuously until the actual final waypoint.
"""

import math
import time
import queue
import threading
import asyncio
import concurrent.futures
import json
import os

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
# home_angle_deg / "Capture Current Pose as Home").

CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1

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
# Cubic Hermite spline helpers (pure math, no hardware access)
#
# Unlike the rest-to-rest trapezoidal profile below, these segments carry a
# specified velocity at BOTH ends, so a chain of them can pass through a
# waypoint without stopping - the building block for "match the conveyor
# speed, then arrive at the pick point already moving with it."
# ---------------------------------------------------------------------------
def _hermite_basis(u):
    u2 = u * u
    u3 = u2 * u
    h00 = 2 * u3 - 3 * u2 + 1
    h10 = u3 - 2 * u2 + u
    h01 = -2 * u3 + 3 * u2
    h11 = u3 - u2
    return h00, h10, h01, h11


def hermite_pos(p0, v0, p1, v1, T, t):
    """Position at time t (0..T) of a cubic Hermite segment from (p0, v0)
    to (p1, v1) over duration T. Units of p and v must be consistent
    (e.g. deg and deg/s)."""
    if T <= 1e-9:
        return p1
    u = max(0.0, min(1.0, t / T))
    h00, h10, h01, h11 = _hermite_basis(u)
    return h00 * p0 + h10 * T * v0 + h01 * p1 + h11 * T * v1


def hermite_vel(p0, v0, p1, v1, T, t):
    """Velocity (d/dt) at time t of the same segment as hermite_pos()."""
    if T <= 1e-9:
        return v1
    u = max(0.0, min(1.0, t / T))
    dh00 = 6 * u * u - 6 * u
    dh10 = 3 * u * u - 4 * u + 1
    dh01 = -6 * u * u + 6 * u
    dh11 = 3 * u * u - 2 * u
    return (dh00 * p0 + dh10 * T * v0 + dh01 * p1 + dh11 * T * v1) / T


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


def _trapezoid_sample(t, distance, vmax, amax):
    """Displacement magnitude (0..|distance|) covered at time t (seconds)."""
    distance_abs = abs(distance)
    total, t_acc, vpeak = _trapezoid_timing(distance_abs, vmax, amax)
    if total <= 0:
        return 0.0

    t = max(0.0, min(t, total))
    t_dec_start = total - t_acc

    if t <= t_acc:
        s = 0.5 * amax * t * t
    elif t <= t_dec_start:
        s = 0.5 * amax * t_acc * t_acc + vpeak * (t - t_acc)
    else:
        td = total - t
        s = distance_abs - 0.5 * amax * td * td

    return s


def synchronized_two_axis_profile(d1, d2, vmax, amax):
    """
    Builds two time-parameterized displacement functions pos1(t), pos2(t)
    (each returning signed displacement from the start position) for two
    axes moving distances d1, d2, sharing the same vmax/amax limits, but
    synchronized to finish at the same time T = max(T1, T2). The shorter
    move is stretched in time (never sped up), so neither axis ever exceeds
    its own vmax/amax.

    Returns (T, pos1_fn, pos2_fn).
    """
    T1, _, _ = _trapezoid_timing(d1, vmax, amax)
    T2, _, _ = _trapezoid_timing(d2, vmax, amax)
    T = max(T1, T2)

    if T <= 0:
        return 0.0, (lambda t: 0.0), (lambda t: 0.0)

    def pos1(t_global):
        t_local = t_global * (T1 / T) if T1 > 0 else 0.0
        mag = _trapezoid_sample(t_local, d1, vmax, amax)
        return math.copysign(mag, d1) if d1 != 0 else 0.0

    def pos2(t_global):
        t_local = t_global * (T2 / T) if T2 > 0 else 0.0
        mag = _trapezoid_sample(t_local, d2, vmax, amax)
        return math.copysign(mag, d2) if d2 != 0 else 0.0

    return T, pos1, pos2


# ---------------------------------------------------------------------------
# Dashboard application
# ---------------------------------------------------------------------------
class FiveBarDashboard:
    def __init__(self):
        self.odrv0 = None
        self.connected = False

        # Messages produced before self.log_box exists (e.g. while loading
        # saved settings) get buffered here and flushed once build_ui() has
        # created the log widget.
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
        self.home_angle_deg = 90.0

        # SPI absolute encoder interface config: which GPIO each axis's
        # encoder chip-select line is wired to, and which SPI absolute
        # encoder chip/protocol it is. M0 (axis0) -> GPIO_4, M1 (axis1) ->
        # GPIO_3 per the wiring for this build.
        self.spi_cfg = {
            0: {"cs_gpio": 4, "mode": ENCODER_MODE_SPI_ABS_AMS},
            1: {"cs_gpio": 3, "mode": ENCODER_MODE_SPI_ABS_AMS},
        }

        # Software trapezoidal motion-planning limits (joint space).
        self.traj_cfg = {
            "max_vel_deg_s": 60.0,
            "max_accel_deg_s2": 120.0,
            "control_rate_hz": 100.0,
        }

        # Motion-execution state
        self.motion_task = None
        self._motion_active = False
        self._motion_stop_event = threading.Event()
        self._viz_queue = queue.Queue(maxsize=1)

        # Serializes ALL actual ODrive/USB access across threads. NiceGUI's
        # run.io_bound() uses a shared thread pool, so without this, a
        # periodic poll_live() read can end up running on a different OS
        # thread at the exact same moment as, say, save_configuration()
        # triggering a reboot - two threads hitting the same USB handle at
        # once. On Windows this has been observed to corrupt an in-flight
        # libusb transfer ("Transfer on EP 0x03 still in progress... This is
        # gonna be messy") and crash the process with an access violation in
        # a native callback. Every function that touches self.odrv0 should
        # go through _locked_call() so only one thread ever talks to the
        # device at a time.
        self._odrv_lock = threading.Lock()

        # Visualization overlays for planned/preview paths
        self.planned_path = []     # list of (x, y) mm points sampled along the plan
        self.waypoints_viz = []    # list of (x, y) mm waypoint markers to draw

        # Path Planning waypoint list: [{"x": .., "y": ..}, ...]
        self.waypoints = []

        # Custom Trajectory waypoint list: [{"x","y","vx","vy","duration"}, ...]
        # vx/vy are desired end-effector Cartesian velocity (mm/s) AT that
        # waypoint; duration is seconds from the previous waypoint (or from
        # the current pose, for the first one). Velocity is continuous
        # through waypoints via a cubic Hermite blend - the arm only stops
        # where a waypoint's velocity is (0, 0).
        self.custom_waypoints = []
        # velocity-arrow overlay for the visualization: [(x, y, vx, vy), ...]
        self.velocity_viz = []

        # PID tuning history (for overlaid step-response comparisons)
        self.step_test_history = []  # list of dicts: samples, start_pos, target, label, color

        # Overwrite the hardcoded defaults above with anything saved from a
        # previous run of this script, if present. Must happen BEFORE
        # build_ui() so the Config-tab fields are created with the loaded
        # values already in self.params / self.axis_cfg / etc.
        self._load_dashboard_config()

        self.build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def build_ui(self):
        ui.page_title("Five-Bar Linkage Dashboard")

        with ui.row().classes("w-full items-center justify-between p-2"):
            self.connect_btn = ui.button("Connect to ODrive", on_click=self.connect_odrive)
            self.status_label = ui.label("Not connected").classes("text-red-600 font-bold")
            with ui.row().classes("items-center gap-2"):
                ui.button("Stop Trajectory", on_click=self.abort_motion, color="orange")
                ui.button("EMERGENCY STOP", on_click=self.emergency_stop, color="red").classes("font-bold")

        with ui.row().classes("w-full no-wrap"):
            # ---------------- Left: control tabs ----------------
            with ui.column().classes("basis-1/3 min-w-[380px]"):
                with ui.tabs().classes("w-full") as tabs:
                    t_joint = ui.tab("Joint Control")
                    t_ik = ui.tab("Inverse Kinematics")
                    t_fk = ui.tab("Forward Kinematics")
                    t_path = ui.tab("Path Planning")
                    t_custom = ui.tab("Custom Trajectory")
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
                    with ui.tab_panel(t_path):
                        self.build_path_tab()
                    with ui.tab_panel(t_custom):
                        self.build_custom_traj_tab()
                    with ui.tab_panel(t_cal):
                        self.build_cal_tab()
                    with ui.tab_panel(t_pid):
                        self.build_pid_tab()
                    with ui.tab_panel(t_cfg):
                        self.build_cfg_tab()

            # ---------------- Right: visualization ----------------
            with ui.column().classes("basis-2/3 items-center"):
                ui.label("Live Linkage Visualization").classes("text-lg font-bold")
                self.viz = ui.html(self.render_svg(None, None, None)).classes("border")
                self.ee_label = ui.label("End effector: X=--  Y=--").classes("font-bold")

        # ---------------- Bottom: log ----------------
        ui.label("Log").classes("font-bold mt-2")
        self.log_box = ui.log(max_lines=200).classes("w-full h-40 border")

        for msg in self._startup_messages:
            self.log(msg)
        self._startup_messages = []

        # live polling timer (every 200 ms) - skipped while a trajectory is
        # actively streaming, since the trajectory thread pushes its own
        # higher-rate viz updates via _viz_queue instead.
        ui.timer(0.2, self.poll_live)
        # drains viz updates pushed by the background trajectory-streaming
        # thread; runs on the main (UI) event loop so it's safe to touch
        # NiceGUI elements here.
        ui.timer(0.05, self._drain_viz_queue)

    def build_joint_tab(self):
        ui.label("Move by joint angle (degrees) - trapezoidal-limited").classes("font-bold")
        self.theta1_input = ui.number(label="Theta1 (axis0)", value=0.0, format="%.2f")
        self.theta2_input = ui.number(label="Theta2 (axis1)", value=0.0, format="%.2f")
        ui.button("Move Joints", on_click=self.move_joints_from_inputs).classes("w-full")

        ui.separator()

        ui.label("Move by raw motor turns (instant, bypasses trajectory planning - "
                 "use small increments only)").classes("font-bold")
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

    def build_path_tab(self):
        ui.label("Add Waypoints (end-effector X/Y, mm)").classes("font-bold")
        self.path_x_input = ui.number(label="X", value=0.0, format="%.2f")
        self.path_y_input = ui.number(label="Y", value=40.0, format="%.2f")
        with ui.row().classes("w-full"):
            ui.button("Add Waypoint", on_click=self.add_waypoint).classes("flex-1")
            ui.button("Clear", on_click=self.clear_waypoints).classes("flex-1")

        ui.separator()
        ui.label("Waypoints (uses elbow settings from Inverse Kinematics tab)").classes("font-bold")
        self.waypoints_container = ui.column().classes("w-full gap-1")
        self._refresh_waypoints_list()

        ui.separator()
        ui.label("Uses Max Velocity / Max Acceleration from the Config tab "
                 "(Trajectory Limits section). Each leg streams from a single "
                 "background thread and does not pause until its waypoint is "
                 "reached.").classes("text-xs text-gray-500")
        with ui.row().classes("w-full"):
            ui.button("Preview Path", on_click=self.preview_path).classes("flex-1")
            ui.button("Run Path", on_click=self.run_path).classes("flex-1")
        ui.button("Abort Motion", on_click=self.abort_motion, color="red").classes("w-full")

    def build_custom_traj_tab(self):
        ui.label("Custom Trajectory (position + velocity + timing)").classes("font-bold")
        ui.label(
            "For conveyor pick-ups: each waypoint sets an end-effector "
            "velocity (mm/s), not just a stopping point, plus a duration "
            "(seconds) from the previous waypoint. Segments are joined with "
            "a velocity-continuous curve, so the arm does not stop at an "
            "intermediate waypoint unless its velocity is (0, 0) - give a "
            "waypoint just before the pick point with Vx/Vy already matching "
            "the belt, then the pick-point waypoint with the same velocity, "
            "and the arm arrives already moving with the glove instead of "
            "accelerating into it."
        ).classes("text-xs text-gray-500")

        ui.separator()
        ui.label("Conveyor Velocity Helper").classes("font-bold")
        self.conv_speed_input = ui.number(label="Conveyor speed (mm/s)", value=50.0, format="%.2f")
        self.conv_angle_input = ui.number(label="Conveyor direction (deg, 0 = +X)", value=0.0, format="%.2f")
        ui.button("Fill Vx/Vy Below From Conveyor", on_click=self.fill_conveyor_velocity).classes("w-full")

        ui.separator()
        ui.label("Add Waypoint").classes("font-bold")
        self.ct_x_input = ui.number(label="X (mm)", value=0.0, format="%.2f")
        self.ct_y_input = ui.number(label="Y (mm)", value=40.0, format="%.2f")
        self.ct_vx_input = ui.number(label="Vx (mm/s)", value=0.0, format="%.2f")
        self.ct_vy_input = ui.number(label="Vy (mm/s)", value=0.0, format="%.2f")
        self.ct_duration_input = ui.number(label="Duration from previous point (s)", value=1.0, format="%.3f")
        with ui.row().classes("w-full"):
            ui.button("Add Waypoint", on_click=self.add_custom_waypoint).classes("flex-1")
            ui.button("Clear", on_click=self.clear_custom_waypoints).classes("flex-1")

        ui.separator()
        ui.label("Waypoints (uses elbow settings from Inverse Kinematics tab)").classes("font-bold")
        self.custom_waypoints_container = ui.column().classes("w-full gap-1")
        self._refresh_custom_waypoints_list()

        ui.separator()
        ui.label(
            "Preview checks the resulting joint velocity/acceleration against "
            "the Config tab's Trajectory Limits and logs a warning (not a "
            "hard block, since matching a real conveyor may legitimately need "
            "more speed than a generic default) if a segment would exceed "
            "them."
        ).classes("text-xs text-gray-500")
        with ui.row().classes("w-full"):
            ui.button("Preview Custom Path", on_click=self.preview_custom_path).classes("flex-1")
            ui.button("Run Custom Trajectory", on_click=self.run_custom_trajectory).classes("flex-1")
        ui.button("Abort Motion", on_click=self.abort_motion, color="red").classes("w-full")

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
        ui.label("Homing Setup (Zero Offset - manual only, no endstops)").classes("font-bold")
        ui.label(
            "This build has no limit switches, so there is no endstop-based "
            "homing state - the zero reference is purely a software value. "
            "'Home reference angle' is the joint angle (theta1=theta2 "
            "convention) that turns_to_joint_deg()/joint_deg_to_turns() treat "
            "as the zero reference. If you're not sure of the correct value, "
            "jog the arm with raw turns to a known physical reference pose, "
            "enter what joint angle that pose SHOULD read as below, then "
            "capture it. Because the encoders are SPI absolute encoders, "
            "this reference (once captured) stays valid across ODrive "
            "power-cycles without needing to re-home."
        ).classes("text-xs text-gray-500")

        self.home_angle_input = ui.number(
            label="Home reference angle (deg)", value=self.home_angle_deg, format="%.3f"
        )
        ui.button("Apply Home Angle", on_click=self.apply_home_angle).classes("w-full")

        ui.separator()
        self.home_capture_target_input = ui.number(
            label="Target joint angle for CURRENT pose (deg)", value=90.0, format="%.2f"
        )
        ui.button(
            "Capture Current Pose as Home", on_click=self.capture_home_from_current_pose
        ).classes("w-full")
        self.home_capture_label = ui.label("").classes("text-xs")

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
            "plain incremental encoder - you do NOT need to re-home after "
            "power-on to recover a valid turn count. The software zero "
            "reference (home_angle_deg, set on the Homing Setup section "
            "above) is unaffected by any of this and does not need to be "
            "recaptured either."
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
        ui.label("Trajectory Limits (joint-space, software-limited)").classes("font-bold")
        ui.label(
            "Control rate is how often the background streaming thread writes "
            "a new setpoint. Higher = smoother interpolation, but there's no "
            "benefit past what your USB link / ODrive can keep up with; "
            "100 Hz is a reasonable default."
        ).classes("text-xs text-gray-500")
        self.cfg_max_vel = ui.number(label="Max joint velocity (deg/s)", value=self.traj_cfg["max_vel_deg_s"])
        self.cfg_max_accel = ui.number(label="Max joint acceleration (deg/s^2)",
                                        value=self.traj_cfg["max_accel_deg_s2"])
        self.cfg_control_rate = ui.number(label="Control rate (Hz)", value=self.traj_cfg["control_rate_hz"])

        ui.button("Apply Config", on_click=self.apply_config).classes("w-full")

        ui.separator()
        ui.label("Dashboard Settings Persistence").classes("font-bold")
        ui.label(
            "Everything on this tab (plus the home reference angle on the "
            "Calibration tab) is auto-saved to {} next to this script "
            "whenever you apply it, so restarting THIS SCRIPT doesn't lose "
            "your geometry/gear/offset/home setup. This is separate from the "
            "ODrive's own persistent calibration (Calibration tab), which "
            "survives power-cycling the ODrive itself.".format(DASHBOARD_CONFIG_FILENAME)
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
                self.home_angle_deg = float(data["home_angle_deg"])
            if "traj_cfg" in data:
                self.traj_cfg.update(data["traj_cfg"])
            self._startup_messages.append("Loaded saved dashboard settings from {}.".format(path))
        except Exception as e:
            self._startup_messages.append("Failed to load dashboard settings from {}: {}".format(path, e))

    def save_dashboard_config(self, silent=False):
        """Writes link geometry, axis gear/offset/direction, home reference
        angle, and trajectory limits to a local JSON file so they survive
        restarting this script (not just the ODrive). Called automatically
        by Apply Config / Apply Home Angle / Capture Home, and available as
        an explicit button too."""
        path = self._dashboard_config_path()
        data = {
            "params": self.params,
            "axis_cfg": {str(k): v for k, v in self.axis_cfg.items()},
            "spi_cfg": {str(k): v for k, v in self.spi_cfg.items()},
            "home_angle_deg": self.home_angle_deg,
            "traj_cfg": self.traj_cfg,
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
        """Re-reads the JSON file and pushes the values back into the
        Config-tab fields (and home-angle field), in case you want to
        discard in-memory edits and go back to what was last saved."""
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
        self.cfg_control_rate.value = self.traj_cfg["control_rate_hz"]
        self.home_angle_input.value = self.home_angle_deg
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
        normalized_angle_deg = angle_deg - self.home_angle_deg
        return cfg["offset_turns"] + cfg["direction"] * (normalized_angle_deg / 360.0) * cfg["gear_ratio"]

    def turns_to_joint_deg(self, axis_idx, turns):
        cfg = self.axis_cfg[axis_idx]
        normalized_angle_deg = ((turns - cfg["offset_turns"]) / cfg["gear_ratio"]) * 360.0 / cfg["direction"]
        return normalized_angle_deg + self.home_angle_deg

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
        if not self.require_connected():
            return

        def _do():
            if self.odrv0.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                self.log("WARNING: axis0 not in CLOSED_LOOP_CONTROL.")
            if self.odrv0.axis1.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                self.log("WARNING: axis1 not in CLOSED_LOOP_CONTROL.")
            self.odrv0.axis0.controller.input_pos = turns0
            self.odrv0.axis1.controller.input_pos = turns1

        try:
            await run.io_bound(self._locked_call, _do)
            self.log("Sent raw turns -> axis0={:.4f}, axis1={:.4f}".format(turns0, turns1))
        except Exception as e:
            self.log("Move failed: {}".format(e))

    # ------------------------------------------------------------------
    # Trajectory streaming (trapezoidal, joint-space, synchronized)
    #
    # The entire move streams from ONE background thread (one run.io_bound
    # call covers the whole trajectory, not one per control-loop tick).
    # Cancellation is via a threading.Event checked every tick (asyncio task
    # cancellation can't interrupt a thread that's mid blocking-call), and
    # viz updates are pushed through a small queue that the UI-thread timer
    # drains, so the control loop itself never waits on NiceGUI/asyncio.
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

    def _stream_joint_trajectory_blocking(self, t1_start, t2_start, t1_target, t2_target, stop_event):
        """
        Runs entirely in a worker thread. Writes input_pos on an absolute
        wall-clock schedule (start_perf + i*dt) rather than sleeping dt
        after each write, so per-iteration compute/write jitter doesn't
        accumulate into drift or visible stalling. Returns a status string
        for the caller to log back on the UI thread.
        """
        d1 = t1_target - t1_start
        d2 = t2_target - t2_start
        vmax = self.traj_cfg["max_vel_deg_s"]
        amax = self.traj_cfg["max_accel_deg_s2"]
        rate = max(1.0, self.traj_cfg["control_rate_hz"])
        dt = 1.0 / rate

        T, pos1, pos2 = synchronized_two_axis_profile(d1, d2, vmax, amax)
        if T <= 0:
            return "Trajectory: target already reached, nothing to do."

        steps = max(1, int(math.ceil(T / dt)))
        start_perf = time.perf_counter()

        for i in range(steps + 1):
            if stop_event.is_set():
                return "Trajectory cancelled."

            t_elapsed = min(i * dt, T)
            t1 = t1_start + pos1(t_elapsed)
            t2 = t2_start + pos2(t_elapsed)
            turns0 = self.joint_deg_to_turns(0, t1)
            turns1 = self.joint_deg_to_turns(1, t2)

            try:
                with self._odrv_lock:
                    self.odrv0.axis0.controller.input_pos = turns0
                    self.odrv0.axis1.controller.input_pos = turns1
            except Exception as e:
                return "Trajectory write failed, aborting: {}".format(e)

            try:
                E, P1, P2 = forward_kinematics(t1, t2, self.params)
                self._push_viz_update(P1, P2, E, t1, t2)
            except Exception:
                pass

            if t_elapsed >= T:
                break

            next_deadline = start_perf + (i + 1) * dt
            sleep_for = next_deadline - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)

        return "Trajectory segment complete."

    async def _stream_joint_trajectory(self, t1_start, t2_start, t1_target, t2_target):
        self.log("Streaming trajectory (single background thread, deadline-scheduled)...")
        self._motion_active = True
        self._motion_stop_event.clear()
        try:
            msg = await run.io_bound(
                self._stream_joint_trajectory_blocking,
                t1_start, t2_start, t1_target, t2_target, self._motion_stop_event
            )
            self.log(msg)
        finally:
            self._motion_active = False

    def _stream_custom_trajectory_blocking(self, segments, stop_event):
        """
        Same architecture as _stream_joint_trajectory_blocking (one thread,
        absolute-deadline scheduling) but plays back a chain of cubic
        Hermite segments (each with its own start/end joint velocity)
        instead of a single rest-to-rest trapezoid. The whole chain streams
        as one continuous move - it only comes to rest where a segment's
        boundary velocity is actually zero.
        """
        rate = max(1.0, self.traj_cfg["control_rate_hz"])
        dt = 1.0 / rate
        total_T = sum(seg["T"] for seg in segments)
        if total_T <= 0:
            return "Custom trajectory: zero total duration, nothing to do."

        boundaries = []
        acc = 0.0
        for seg in segments:
            boundaries.append((acc, acc + seg["T"], seg))
            acc += seg["T"]

        steps = max(1, int(math.ceil(total_T / dt)))
        start_perf = time.perf_counter()

        for i in range(steps + 1):
            if stop_event.is_set():
                return "Custom trajectory cancelled."

            t_elapsed = min(i * dt, total_T)

            seg = boundaries[-1][2]
            seg_t = t_elapsed - boundaries[-1][0]
            for seg_start, seg_end, s in boundaries:
                if t_elapsed <= seg_end:
                    seg = s
                    seg_t = t_elapsed - seg_start
                    break

            t1 = hermite_pos(seg["t1_0"], seg["w1_0"], seg["t1_1"], seg["w1_1"], seg["T"], seg_t)
            t2 = hermite_pos(seg["t2_0"], seg["w2_0"], seg["t2_1"], seg["w2_1"], seg["T"], seg_t)
            turns0 = self.joint_deg_to_turns(0, t1)
            turns1 = self.joint_deg_to_turns(1, t2)

            try:
                with self._odrv_lock:
                    self.odrv0.axis0.controller.input_pos = turns0
                    self.odrv0.axis1.controller.input_pos = turns1
            except Exception as e:
                return "Custom trajectory write failed, aborting: {}".format(e)

            try:
                E, P1, P2 = forward_kinematics(t1, t2, self.params)
                self._push_viz_update(P1, P2, E, t1, t2)
            except Exception:
                pass

            if t_elapsed >= total_T:
                break

            next_deadline = start_perf + (i + 1) * dt
            sleep_for = next_deadline - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)

        return "Custom trajectory complete."

    async def _stream_custom_trajectory(self, segments):
        self.log("Streaming custom trajectory ({} segment(s), velocity-matched Hermite blend)...".format(
            len(segments)))
        self._motion_active = True
        self._motion_stop_event.clear()
        try:
            msg = await run.io_bound(
                self._stream_custom_trajectory_blocking, segments, self._motion_stop_event
            )
            self.log(msg)
        finally:
            self._motion_active = False

    async def _prepare_custom_segments(self):
        """
        Converts self.custom_waypoints (Cartesian x/y/vx/vy/duration) into a
        list of joint-space Hermite segments. Returns None (after notifying
        the user) if anything is unreachable/invalid.
        """
        if len(self.custom_waypoints) < 1:
            ui.notify("Add at least one custom waypoint first.", type="warning")
            return None

        params = self._current_ik_params()
        joint_wps = []
        for wp in self.custom_waypoints:
            try:
                t1, t2 = inverse_kinematics(wp["x"], wp["y"], params)
                J = numerical_jacobian(wp["x"], wp["y"], params)
                w1, w2 = joint_velocity_from_cartesian(J, wp["vx"], wp["vy"])
            except Exception as e:
                ui.notify("Custom waypoint error at ({:.2f}, {:.2f}): {}".format(wp["x"], wp["y"], e),
                          type="negative")
                return None
            joint_wps.append({"t1": t1, "t2": t2, "w1": w1, "w2": w2, "duration": wp["duration"]})

        if self.connected and self.odrv0 is not None:
            try:
                t1_cur, t2_cur = await self._get_current_joint_deg()
            except Exception:
                t1_cur, t2_cur = joint_wps[0]["t1"], joint_wps[0]["t2"]
        else:
            t1_cur, t2_cur = joint_wps[0]["t1"], joint_wps[0]["t2"]

        chain = [{"t1": t1_cur, "t2": t2_cur, "w1": 0.0, "w2": 0.0}] + joint_wps

        segments = []
        for i in range(len(chain) - 1):
            a, b = chain[i], chain[i + 1]
            T = max(0.01, float(b["duration"]))
            segments.append({
                "t1_0": a["t1"], "t2_0": a["t2"], "w1_0": a["w1"], "w2_0": a["w2"],
                "t1_1": b["t1"], "t2_1": b["t2"], "w1_1": b["w1"], "w2_1": b["w2"],
                "T": T,
            })
        return segments

    def _check_custom_segment_limits(self, segments, samples=40):
        """Soft check only - logs warnings but never blocks, since matching
        a real conveyor may genuinely require more speed than a generic
        default limit."""
        warnings_out = []
        vmax = self.traj_cfg["max_vel_deg_s"]
        amax = self.traj_cfg["max_accel_deg_s2"]
        for idx, seg in enumerate(segments):
            T = seg["T"]
            peak_v = 0.0
            peak_a = 0.0
            prev_v1 = prev_v2 = prev_t = None
            for s in range(samples + 1):
                t = T * s / samples
                v1 = hermite_vel(seg["t1_0"], seg["w1_0"], seg["t1_1"], seg["w1_1"], T, t)
                v2 = hermite_vel(seg["t2_0"], seg["w2_0"], seg["t2_1"], seg["w2_1"], T, t)
                peak_v = max(peak_v, abs(v1), abs(v2))
                if prev_t is not None:
                    dt_s = t - prev_t
                    if dt_s > 1e-9:
                        peak_a = max(peak_a, abs(v1 - prev_v1) / dt_s, abs(v2 - prev_v2) / dt_s)
                prev_v1, prev_v2, prev_t = v1, v2, t
            if peak_v > vmax:
                warnings_out.append("Segment {}: peak joint velocity {:.1f} deg/s exceeds configured "
                                     "max {:.1f} deg/s.".format(idx + 1, peak_v, vmax))
            if peak_a > amax:
                warnings_out.append("Segment {}: peak joint acceleration {:.1f} deg/s^2 exceeds "
                                     "configured max {:.1f} deg/s^2.".format(idx + 1, peak_a, amax))
        return warnings_out

    def _sample_custom_segments_for_viz(self, segments, samples_per_segment=20):
        pts = []
        for seg in segments:
            T = seg["T"]
            for s in range(samples_per_segment + 1):
                t = T * s / samples_per_segment
                t1 = hermite_pos(seg["t1_0"], seg["w1_0"], seg["t1_1"], seg["w1_1"], T, t)
                t2 = hermite_pos(seg["t2_0"], seg["w2_0"], seg["t2_1"], seg["w2_1"], T, t)
                try:
                    E, _, _ = forward_kinematics(t1, t2, self.params)
                    pts.append(E)
                except Exception:
                    pass
        return pts

    def _sample_path_for_viz(self, joint_waypoints, samples_per_segment=20):
        """
        Given a list of (theta1_deg, theta2_deg) waypoints, replays the same
        trapezoidal profile used for actual execution and returns the
        resulting end-effector (x, y) path for drawing - this reflects any
        curvature caused by the linkage kinematics, not just a straight
        Cartesian guess between waypoints.
        """
        pts = []
        vmax = self.traj_cfg["max_vel_deg_s"]
        amax = self.traj_cfg["max_accel_deg_s2"]
        for i in range(len(joint_waypoints) - 1):
            t1a, t2a = joint_waypoints[i]
            t1b, t2b = joint_waypoints[i + 1]
            d1 = t1b - t1a
            d2 = t2b - t2a
            T, pos1, pos2 = synchronized_two_axis_profile(d1, d2, vmax, amax)
            if T <= 0:
                continue
            for s in range(samples_per_segment + 1):
                t = T * s / samples_per_segment
                t1 = t1a + pos1(t)
                t2 = t2a + pos2(t)
                try:
                    E, _, _ = forward_kinematics(t1, t2, self.params)
                    pts.append(E)
                except Exception:
                    pass
        return pts

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
        if self._motion_active:
            self._motion_stop_event.set()
            self.log("Abort requested.")
        elif self.motion_task is not None and not self.motion_task.done():
            self.motion_task.cancel()
            self.log("Abort requested.")
        else:
            self.log("No motion in progress.")

    # ------------------------------------------------------------------
    # Joint Control tab actions
    # ------------------------------------------------------------------
    async def move_joints_from_inputs(self):
        if not self.require_connected():
            return
        try:
            t1_target = float(self.theta1_input.value)
            t2_target = float(self.theta2_input.value)
        except (TypeError, ValueError):
            ui.notify("Theta1/Theta2 must be numbers.", type="negative")
            return

        try:
            t1_start, t2_start = await self._get_current_joint_deg()
        except Exception as e:
            self.log("Could not read current position, aborting move: {}".format(e))
            return

        self.planned_path = self._sample_path_for_viz([(t1_start, t2_start), (t1_target, t2_target)])
        try:
            E_target, _, _ = forward_kinematics(t1_target, t2_target, self.params)
            self.waypoints_viz = [E_target]
        except Exception:
            self.waypoints_viz = []
        self.velocity_viz = []

        await self._launch_motion_task(self._stream_joint_trajectory(t1_start, t2_start, t1_target, t2_target))

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
        if not self.require_connected():
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

        try:
            t1_start, t2_start = await self._get_current_joint_deg()
        except Exception as e:
            self.log("Could not read current position, aborting move: {}".format(e))
            return

        self.planned_path = self._sample_path_for_viz([(t1_start, t2_start), (t1_target, t2_target)])
        self.waypoints_viz = [(x, y)]
        self.velocity_viz = []

        await self._launch_motion_task(self._stream_joint_trajectory(t1_start, t2_start, t1_target, t2_target))

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
    # Path Planning tab actions
    # ------------------------------------------------------------------
    def add_waypoint(self):
        try:
            x = float(self.path_x_input.value)
            y = float(self.path_y_input.value)
        except (TypeError, ValueError):
            ui.notify("X/Y must be numbers.", type="negative")
            return
        try:
            inverse_kinematics(x, y, self._current_ik_params())
        except Exception as e:
            ui.notify("Waypoint unreachable: {}".format(e), type="negative")
            return
        self.waypoints.append({"x": x, "y": y})
        self._refresh_waypoints_list()
        self.log("Added waypoint {}: ({:.2f}, {:.2f})".format(len(self.waypoints), x, y))

    def remove_waypoint(self, idx):
        if 0 <= idx < len(self.waypoints):
            removed = self.waypoints.pop(idx)
            self.log("Removed waypoint: ({:.2f}, {:.2f})".format(removed["x"], removed["y"]))
            self._refresh_waypoints_list()

    def clear_waypoints(self):
        self.waypoints = []
        self.planned_path = []
        self.waypoints_viz = []
        self.velocity_viz = []
        self._refresh_waypoints_list()
        self.log("Waypoints cleared.")

    def _refresh_waypoints_list(self):
        self.waypoints_container.clear()
        with self.waypoints_container:
            if not self.waypoints:
                ui.label("(no waypoints yet)").classes("text-gray-500 text-sm")
            for idx, wp in enumerate(self.waypoints):
                with ui.row().classes("items-center gap-2"):
                    ui.label("{}: ({:.2f}, {:.2f})".format(idx + 1, wp["x"], wp["y"]))
                    ui.button(icon="delete", on_click=lambda i=idx: self.remove_waypoint(i)).props("flat dense")

    async def preview_path(self):
        if len(self.waypoints) < 1:
            ui.notify("Add at least one waypoint first.", type="warning")
            return
        try:
            joint_targets = [inverse_kinematics(wp["x"], wp["y"], self._current_ik_params())
                              for wp in self.waypoints]
        except Exception as e:
            ui.notify("Path error: {}".format(e), type="negative")
            return

        if self.connected and self.odrv0 is not None:
            try:
                t1_cur, t2_cur = await self._get_current_joint_deg()
            except Exception:
                t1_cur, t2_cur = joint_targets[0]
        else:
            t1_cur, t2_cur = joint_targets[0]

        chain = [(t1_cur, t2_cur)] + joint_targets
        self.planned_path = self._sample_path_for_viz(chain)
        self.waypoints_viz = [(wp["x"], wp["y"]) for wp in self.waypoints]
        self.velocity_viz = []
        self.viz.set_content(self.render_svg(None, None, None))
        self.log("Path preview updated ({} waypoint(s)).".format(len(self.waypoints)))

    async def run_path(self):
        if not self.require_connected():
            return
        if len(self.waypoints) < 1:
            ui.notify("Add at least one waypoint first.", type="warning")
            return

        try:
            joint_targets = [inverse_kinematics(wp["x"], wp["y"], self._current_ik_params())
                              for wp in self.waypoints]
        except Exception as e:
            self.log("Path planning error: {}".format(e))
            ui.notify("Path planning error: {}".format(e), type="negative")
            return

        try:
            t1_cur, t2_cur = await self._get_current_joint_deg()
        except Exception as e:
            self.log("Could not read current position, aborting path: {}".format(e))
            return

        chain = [(t1_cur, t2_cur)] + joint_targets
        self.planned_path = self._sample_path_for_viz(chain)
        self.waypoints_viz = [(wp["x"], wp["y"]) for wp in self.waypoints]
        self.velocity_viz = []

        async def _run():
            cur1, cur2 = t1_cur, t2_cur
            for idx, (t1, t2) in enumerate(joint_targets):
                if self._motion_stop_event.is_set():
                    self.log("Path aborted before waypoint {}.".format(idx + 1))
                    break
                self.log("Path: moving to waypoint {}/{}...".format(idx + 1, len(joint_targets)))
                await self._stream_joint_trajectory(cur1, cur2, t1, t2)
                if self._motion_stop_event.is_set():
                    self.log("Path aborted after waypoint {}.".format(idx + 1))
                    break
                cur1, cur2 = t1, t2
            else:
                self.log("Path complete.")

        await self._launch_motion_task(_run())

    # ------------------------------------------------------------------
    # Custom Trajectory tab actions
    # ------------------------------------------------------------------
    def fill_conveyor_velocity(self):
        try:
            speed = float(self.conv_speed_input.value)
            angle = float(self.conv_angle_input.value)
        except (TypeError, ValueError):
            ui.notify("Conveyor speed/direction must be numbers.", type="negative")
            return
        rad = math.radians(angle)
        self.ct_vx_input.value = round(speed * math.cos(rad), 3)
        self.ct_vy_input.value = round(speed * math.sin(rad), 3)
        self.log("Filled Vx/Vy from conveyor: speed={:.2f} mm/s, direction={:.1f} deg "
                  "-> Vx={:.2f}, Vy={:.2f}".format(speed, angle, self.ct_vx_input.value, self.ct_vy_input.value))

    def add_custom_waypoint(self):
        try:
            x = float(self.ct_x_input.value)
            y = float(self.ct_y_input.value)
            vx = float(self.ct_vx_input.value)
            vy = float(self.ct_vy_input.value)
            duration = float(self.ct_duration_input.value)
        except (TypeError, ValueError):
            ui.notify("All waypoint fields must be numbers.", type="negative")
            return
        if duration <= 0:
            ui.notify("Duration must be greater than 0.", type="negative")
            return
        try:
            inverse_kinematics(x, y, self._current_ik_params())
        except Exception as e:
            ui.notify("Waypoint unreachable: {}".format(e), type="negative")
            return
        self.custom_waypoints.append({"x": x, "y": y, "vx": vx, "vy": vy, "duration": duration})
        self._refresh_custom_waypoints_list()
        self.log("Added custom waypoint {}: pos=({:.2f}, {:.2f}) vel=({:.2f}, {:.2f}) mm/s, "
                  "{:.3f}s from previous".format(len(self.custom_waypoints), x, y, vx, vy, duration))

    def remove_custom_waypoint(self, idx):
        if 0 <= idx < len(self.custom_waypoints):
            removed = self.custom_waypoints.pop(idx)
            self.log("Removed custom waypoint: ({:.2f}, {:.2f})".format(removed["x"], removed["y"]))
            self._refresh_custom_waypoints_list()

    def clear_custom_waypoints(self):
        self.custom_waypoints = []
        self.planned_path = []
        self.waypoints_viz = []
        self.velocity_viz = []
        self._refresh_custom_waypoints_list()
        self.log("Custom waypoints cleared.")

    def _refresh_custom_waypoints_list(self):
        self.custom_waypoints_container.clear()
        with self.custom_waypoints_container:
            if not self.custom_waypoints:
                ui.label("(no custom waypoints yet)").classes("text-gray-500 text-sm")
            for idx, wp in enumerate(self.custom_waypoints):
                with ui.row().classes("items-center gap-2"):
                    ui.label("{}: pos=({:.2f}, {:.2f})  vel=({:.2f}, {:.2f}) mm/s  {:.3f}s".format(
                        idx + 1, wp["x"], wp["y"], wp["vx"], wp["vy"], wp["duration"]))
                    ui.button(icon="delete", on_click=lambda i=idx: self.remove_custom_waypoint(i)).props(
                        "flat dense")

    async def preview_custom_path(self):
        segments = await self._prepare_custom_segments()
        if segments is None:
            return
        warnings_out = self._check_custom_segment_limits(segments)
        for w in warnings_out:
            self.log("WARNING: " + w)
        if warnings_out:
            ui.notify("{} trajectory-limit warning(s) - see log.".format(len(warnings_out)), type="warning")

        self.planned_path = self._sample_custom_segments_for_viz(segments)
        self.waypoints_viz = [(wp["x"], wp["y"]) for wp in self.custom_waypoints]
        self.velocity_viz = [(wp["x"], wp["y"], wp["vx"], wp["vy"]) for wp in self.custom_waypoints]
        self.viz.set_content(self.render_svg(None, None, None))
        self.log("Custom path preview updated ({} waypoint(s), total duration {:.2f}s).".format(
            len(self.custom_waypoints), sum(s["T"] for s in segments)))

    async def run_custom_trajectory(self):
        if not self.require_connected():
            return
        segments = await self._prepare_custom_segments()
        if segments is None:
            return
        warnings_out = self._check_custom_segment_limits(segments)
        for w in warnings_out:
            self.log("WARNING: " + w)
        if warnings_out:
            ui.notify("{} trajectory-limit warning(s) - check log before trusting this move.".format(
                len(warnings_out)), type="warning")

        self.planned_path = self._sample_custom_segments_for_viz(segments)
        self.waypoints_viz = [(wp["x"], wp["y"]) for wp in self.custom_waypoints]
        self.velocity_viz = [(wp["x"], wp["y"], wp["vx"], wp["vy"]) for wp in self.custom_waypoints]

        await self._launch_motion_task(self._stream_custom_trajectory(segments))

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

    def apply_home_angle(self):
        """Manually set the home reference angle (deg) used by
        joint_deg_to_turns()/turns_to_joint_deg() as the zero point for both
        axes. Use this if you already know the correct value; otherwise use
        'Capture Current Pose as Home' below."""
        try:
            value = float(self.home_angle_input.value)
        except (TypeError, ValueError):
            ui.notify("Home angle must be a number.", type="negative")
            return
        self.home_angle_deg = value
        self.log("Home reference angle set to {:.3f} deg.".format(value))
        self.save_dashboard_config(silent=True)
        ui.notify("Home angle applied.", type="positive")

    async def capture_home_from_current_pose(self):
        """
        Reads the current encoder position on both axes and back-solves what
        home_angle_deg would need to be for the CURRENT physical pose to read
        as the target angle entered above. This is meant for the common case
        where you don't know the true homing offset: jog the arm (raw turns)
        to a known/repeatable physical reference position, tell it what
        joint angle that position should correspond to, and capture it.

        Each axis is solved independently and the two results are averaged;
        a large discrepancy between them usually means the axis offset_turns
        / gear_ratio / direction settings on the Config tab don't agree with
        each other and should be checked.
        """
        if not self.require_connected():
            return
        try:
            target_deg = float(self.home_capture_target_input.value)
        except (TypeError, ValueError):
            ui.notify("Target angle must be a number.", type="negative")
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

        home0 = target_deg - norm0
        home1 = target_deg - norm1
        avg_home = (home0 + home1) / 2.0
        discrepancy = abs(home0 - home1)

        self.home_angle_deg = avg_home
        self.home_angle_input.value = round(avg_home, 3)

        msg = ("Captured home from current pose: axis0 implies home={:.3f} deg, "
               "axis1 implies home={:.3f} deg, using average={:.3f} deg "
               "(discrepancy={:.3f} deg).").format(home0, home1, avg_home, discrepancy)
        self.log(msg)
        self.home_capture_label.text = msg
        self.save_dashboard_config(silent=True)

        if discrepancy > 1.0:
            ui.notify(
                "Axes disagree on home by {:.2f} deg - check Config tab offsets/gear/direction.".format(
                    discrepancy),
                type="warning",
            )
        else:
            ui.notify("Home angle captured: {:.3f} deg".format(avg_home), type="positive")

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

    def emergency_stop(self):
        # Deliberately synchronous / not routed through run.io_bound so it
        # fires immediately even if other background operations are busy.
        self._motion_stop_event.set()
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
            self.log("EMERGENCY STOP: both axes set to IDLE.")
        except Exception as e:
            self.log("E-stop failed: {}".format(e))
        finally:
            if acquired:
                self._odrv_lock.release()

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
            self.traj_cfg["control_rate_hz"] = max(1.0, float(self.cfg_control_rate.value))

            self.log("Config applied.")
            ui.notify("Config applied.", type="positive")
            self.save_dashboard_config(silent=True)
        except (TypeError, ValueError) as e:
            ui.notify("Invalid config: {}".format(e), type="negative")

    # ------------------------------------------------------------------
    # Live polling loop
    # ------------------------------------------------------------------
    async def poll_live(self):
        if not (self.connected and self.odrv0 is not None):
            return
        if self._motion_active:
            # the trajectory thread is driving viz updates via _viz_queue at
            # a higher rate; don't fight it with a slower/racier update here.
            return
        try:
            turns0, turns1 = await run.io_bound(self._locked_call, self._read_encoder_turns)
            t1 = self.turns_to_joint_deg(0, turns0)
            t2 = self.turns_to_joint_deg(1, turns1)
            self.live_joint_label.text = "theta1={:.2f} deg   theta2={:.2f} deg".format(t1, t2)

            E, P1, P2 = forward_kinematics(t1, t2, self.params)
            self.ee_label.text = "End effector: X={:.2f} mm   Y={:.2f} mm".format(E[0], E[1])
            self.viz.set_content(self.render_svg(P1, P2, E))
        except Exception:
            pass  # unreachable pose / transient read error, skip this frame

    # ------------------------------------------------------------------
    # Visualization (renders an SVG string)
    # ------------------------------------------------------------------
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
