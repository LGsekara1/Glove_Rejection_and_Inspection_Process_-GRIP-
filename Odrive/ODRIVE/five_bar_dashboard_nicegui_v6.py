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
"""

import math
import time
import asyncio
import concurrent.futures

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
AXIS_STATE_HOMING = 11

CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1


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

        self.params = {
            "L0": 30.0,
            "l1a": 30.0,
            "l2a": 45.0,
            "l1b": 30.0,
            "l2b": 45.0,
            "elbow1": "up",
            "elbow2": "down",
            "fk_branch": "upper",
        }

        self.axis_cfg = {
            0: {"gear_ratio": 1.0, "offset_turns": 0.0, "direction": -1.0},
            1: {"gear_ratio": 1.0, "offset_turns": 0.0, "direction": -1.0},
        }
        self.home_angle_deg = 90.0

        # Software trapezoidal motion-planning limits (joint space).
        self.traj_cfg = {
            "max_vel_deg_s": 60.0,
            "max_accel_deg_s2": 120.0,
            "control_rate_hz": 50.0,
        }

        # Motion-execution state
        self.motion_task = None

        # Visualization overlays for planned/preview paths
        self.planned_path = []     # list of (x, y) mm points sampled along the plan
        self.waypoints_viz = []    # list of (x, y) mm waypoint markers to draw

        # Path Planning waypoint list: [{"x": .., "y": ..}, ...]
        self.waypoints = []

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

        # live polling timer (every 200 ms)
        ui.timer(0.2, self.poll_live)

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
                 "(Trajectory Limits section).").classes("text-xs text-gray-500")
        with ui.row().classes("w-full"):
            ui.button("Preview Path", on_click=self.preview_path).classes("flex-1")
            ui.button("Run Path", on_click=self.run_path).classes("flex-1")
        ui.button("Abort Motion", on_click=self.abort_motion, color="red").classes("w-full")

    def build_cal_tab(self):
        ui.button("Clear Errors", on_click=self.clear_errors).classes("w-full")
        ui.button("Calibrate Axis0 (motor + encoder)", on_click=lambda: self.calibrate_axis(0)).classes("w-full")
        ui.button("Calibrate Axis1 (motor + encoder)", on_click=lambda: self.calibrate_axis(1)).classes("w-full")
        ui.button("Calibrate Both", on_click=self.calibrate_both).classes("w-full")
        ui.separator()
        ui.button("Enable Closed Loop Control (Both)", on_click=self.enable_closed_loop_both).classes("w-full")
        ui.button("Home Both Axes", on_click=self.home_both).classes("w-full")
        ui.button("Idle Both Axes", on_click=self.idle_both).classes("w-full")
        ui.separator()
        ui.button("Show Errors", on_click=self.show_errors).classes("w-full")

        ui.separator()
        ui.label("Homing Setup (Zero Offset)").classes("font-bold")
        ui.label(
            "'Home reference angle' is the joint angle (theta1=theta2 convention) "
            "that turns_to_joint_deg()/joint_deg_to_turns() treat as the zero "
            "reference. If you're not sure of the correct value, jog the arm "
            "with raw turns to a known physical reference pose, enter what "
            "joint angle that pose SHOULD read as below, then capture it."
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

    def build_pid_tab(self):
        ui.label("Axis0 Gains").classes("font-bold")
        self.pid0_pos_gain = ui.number(label="pos_gain", value=0.0, format="%.4f")
        self.pid0_vel_gain = ui.number(label="vel_gain", value=0.0, format="%.6f")
        self.pid0_vel_int_gain = ui.number(label="vel_integrator_gain", value=0.0, format="%.6f")
        self.pid0_vel_limit = ui.number(label="vel_limit (turns/s)", value=0.0, format="%.3f")
        self.pid0_current_lim = ui.number(label="current_lim (A)", value=0.0, format="%.2f")
        with ui.row().classes("w-full"):
            ui.button("Read Axis0", on_click=lambda: self.read_gains(0)).classes("flex-1")
            ui.button("Apply Axis0", on_click=lambda: self.apply_gains(0)).classes("flex-1")

        ui.separator()
        ui.label("Axis1 Gains").classes("font-bold")
        self.pid1_pos_gain = ui.number(label="pos_gain", value=0.0, format="%.4f")
        self.pid1_vel_gain = ui.number(label="vel_gain", value=0.0, format="%.6f")
        self.pid1_vel_int_gain = ui.number(label="vel_integrator_gain", value=0.0, format="%.6f")
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
        ui.button("Run Step Test", on_click=self.run_step_test).classes("w-full")
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
        ui.label("Trajectory Limits (joint-space, software-limited)").classes("font-bold")
        self.cfg_max_vel = ui.number(label="Max joint velocity (deg/s)", value=self.traj_cfg["max_vel_deg_s"])
        self.cfg_max_accel = ui.number(label="Max joint acceleration (deg/s^2)",
                                        value=self.traj_cfg["max_accel_deg_s2"])
        self.cfg_control_rate = ui.number(label="Control rate (Hz)", value=self.traj_cfg["control_rate_hz"])

        ui.button("Apply Config", on_click=self.apply_config).classes("w-full")

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------
    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        line = "[{}] {}".format(ts, msg)
        self.log_box.push(line)
        print(line)

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

    # ------------------------------------------------------------------
    # Low-level motion primitives
    # ------------------------------------------------------------------
    def require_connected(self):
        if not self.connected or self.odrv0 is None:
            ui.notify("Connect to the ODrive first.", type="warning")
            return False
        return True

    def _read_encoder_turns(self):
        return self.odrv0.axis0.encoder.pos_estimate, self.odrv0.axis1.encoder.pos_estimate

    async def _get_current_joint_deg(self):
        turns0, turns1 = await run.io_bound(self._read_encoder_turns)
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
            await run.io_bound(_do)
            self.log("Sent raw turns -> axis0={:.4f}, axis1={:.4f}".format(turns0, turns1))
        except Exception as e:
            self.log("Move failed: {}".format(e))

    # ------------------------------------------------------------------
    # Trajectory streaming (trapezoidal, joint-space, synchronized)
    # ------------------------------------------------------------------
    async def _stream_joint_trajectory(self, t1_start, t2_start, t1_target, t2_target):
        d1 = t1_target - t1_start
        d2 = t2_target - t2_start
        vmax = self.traj_cfg["max_vel_deg_s"]
        amax = self.traj_cfg["max_accel_deg_s2"]
        rate = max(1.0, self.traj_cfg["control_rate_hz"])
        dt = 1.0 / rate

        T, pos1, pos2 = synchronized_two_axis_profile(d1, d2, vmax, amax)
        if T <= 0:
            self.log("Trajectory: target already reached, nothing to do.")
            return

        self.log("Streaming trajectory: duration={:.2f}s, dt={:.3f}s".format(T, dt))
        steps = max(1, int(math.ceil(T / dt)))

        try:
            for i in range(steps + 1):
                t = min(i * dt, T)
                t1 = t1_start + pos1(t)
                t2 = t2_start + pos2(t)
                turns0 = self.joint_deg_to_turns(0, t1)
                turns1 = self.joint_deg_to_turns(1, t2)
                try:
                    await run.io_bound(self._write_input_pos, turns0, turns1)
                except Exception as e:
                    self.log("Trajectory write failed, aborting: {}".format(e))
                    return
                try:
                    E, P1, P2 = forward_kinematics(t1, t2, self.params)
                    self.viz.set_content(self.render_svg(P1, P2, E))
                except Exception:
                    pass
                if t >= T:
                    break
                await asyncio.sleep(dt)
        except asyncio.CancelledError:
            self.log("Trajectory cancelled.")
            raise

        self.log("Trajectory segment complete.")

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
            self.motion_task.cancel()
            try:
                await self.motion_task
            except asyncio.CancelledError:
                pass
        self.motion_task = asyncio.create_task(coro)
        return self.motion_task

    async def abort_motion(self):
        if self.motion_task is not None and not self.motion_task.done():
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
            turns0, turns1 = await run.io_bound(self._read_encoder_turns)
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

        async def _run():
            cur1, cur2 = t1_cur, t2_cur
            for idx, (t1, t2) in enumerate(joint_targets):
                self.log("Path: moving to waypoint {}/{}...".format(idx + 1, len(joint_targets)))
                await self._stream_joint_trajectory(cur1, cur2, t1, t2)
                cur1, cur2 = t1, t2
            self.log("Path complete.")

        await self._launch_motion_task(_run())

    # ------------------------------------------------------------------
    # Calibration / homing / safety
    # ------------------------------------------------------------------
    async def clear_errors(self):
        if not self.require_connected():
            return
        await run.io_bound(self.odrv0.clear_errors)
        self.log("Errors cleared.")

    def _run_state_blocking(self, axis, state, name, timeout=30):
        axis.requested_state = state
        start = time.time()
        while axis.current_state != AXIS_STATE_IDLE:
            if time.time() - start > timeout:
                self.log("{}: TIMEOUT waiting for IDLE".format(name))
                return False
            time.sleep(0.1)
        ok = (axis.error == 0 and axis.motor.error == 0 and axis.encoder.error == 0)
        if not ok:
            self.log("{}: error after state -> axis.error={}, motor.error={}, encoder.error={}".format(
                name, axis.error, axis.motor.error, axis.encoder.error))
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
            await run.io_bound(_do)
            self.log("Requested CLOSED_LOOP_CONTROL (position control, passthrough input) on both axes.")
        except Exception as e:
            self.log("Enable closed loop failed: {}".format(e))

    def _home_both_blocking(self):
        self.log("Starting homing on both axes...")
        self.odrv0.axis0.requested_state = AXIS_STATE_HOMING
        self.odrv0.axis1.requested_state = AXIS_STATE_HOMING
        time.sleep(0.2)
        start = time.time()
        while (self.odrv0.axis0.current_state != AXIS_STATE_IDLE or
               self.odrv0.axis1.current_state != AXIS_STATE_IDLE):
            if time.time() - start > 60:
                self.log("Homing TIMEOUT")
                return
            time.sleep(0.1)
        e0 = (self.odrv0.axis0.error == 0 and self.odrv0.axis0.motor.error == 0
              and self.odrv0.axis0.encoder.error == 0)
        e1 = (self.odrv0.axis1.error == 0 and self.odrv0.axis1.motor.error == 0
              and self.odrv0.axis1.encoder.error == 0)
        if e0 and e1:
            self.log("Homing complete on both axes.")
        else:
            self.log("Homing finished with errors. Check 'Show Errors'.")

    async def home_both(self):
        if not self.require_connected():
            return
        await run.io_bound(self._home_both_blocking)

    async def idle_both(self):
        if not self.require_connected():
            return

        def _do():
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            self.odrv0.axis1.requested_state = AXIS_STATE_IDLE

        await run.io_bound(_do)
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
            turns0, turns1 = await run.io_bound(self._read_encoder_turns)
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

        if discrepancy > 1.0:
            ui.notify(
                "Axes disagree on home by {:.2f} deg - check Config tab offsets/gear/direction.".format(
                    discrepancy),
                type="warning",
            )
        else:
            ui.notify("Home angle captured: {:.3f} deg".format(avg_home), type="positive")

    def emergency_stop(self):
        # Deliberately synchronous / not routed through run.io_bound so it
        # fires immediately even if other background operations are busy.
        if self.motion_task is not None and not self.motion_task.done():
            self.motion_task.cancel()
        if not self.connected or self.odrv0 is None:
            return
        try:
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
            self.log("EMERGENCY STOP: both axes set to IDLE.")
        except Exception as e:
            self.log("E-stop failed: {}".format(e))

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

        msg = await run.io_bound(_do)
        self.log(msg)

    # ------------------------------------------------------------------
    # PID Tuning actions
    # ------------------------------------------------------------------
    async def read_gains(self, idx):
        if not self.require_connected():
            return
        axis = self.odrv0.axis0 if idx == 0 else self.odrv0.axis1

        def _do():
            c = axis.controller.config
            return c.pos_gain, c.vel_gain, c.vel_integrator_gain, c.vel_limit, axis.motor.config.current_lim

        try:
            pos_gain, vel_gain, vel_int_gain, vel_limit, current_lim = await run.io_bound(_do)
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
            await run.io_bound(_do)
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
        try:
            await run.io_bound(self.odrv0.save_configuration)
            self.log("Configuration saved. ODrive is rebooting; reconnect once it comes back.")
        except Exception as e:
            # save_configuration commonly raises because the device reboots
            # immediately after replying - treat as expected.
            self.log("Save configuration sent (connection drop on reboot is expected): {}".format(e))
        self.connected = False
        self.odrv0 = None
        self.status_label.text = "Not connected (rebooting)"
        self.status_label.classes(remove="text-green-600", add="text-red-600")

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
            start_pos = await run.io_bound(_get_pos)
        except Exception as e:
            self.log("Step test failed to read start position: {}".format(e))
            return

        target = start_pos + step
        self.log("axis{}: step test, start={:.4f} turns, target={:.4f} turns".format(idx, start_pos, target))

        def _command():
            axis.controller.input_pos = target

        await run.io_bound(_command)

        samples = []
        t0 = time.time()
        interval = 0.02
        while (time.time() - t0) < duration:
            try:
                pos = await run.io_bound(_get_pos)
            except Exception:
                break
            samples.append((time.time() - t0, pos))
            await asyncio.sleep(interval)

        self.log("axis{}: step test collected {} samples.".format(idx, len(samples)))
        self.step_chart.set_content(self.render_step_chart(samples, start_pos, target))

    def render_step_chart(self, samples, start_pos, target):
        w, h = 600, 260
        pad = 30
        if not samples:
            return '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}"></svg>'.format(w, h)

        ts = [s[0] for s in samples]
        ys = [s[1] for s in samples]
        y_all = ys + [start_pos, target]
        tmin, tmax = 0.0, max(ts) if ts else 1.0
        ymin, ymax = min(y_all), max(y_all)
        if ymax - ymin < 1e-6:
            ymax = ymin + 1e-6

        def to_px(t, y):
            px = pad + (t - tmin) / (tmax - tmin + 1e-9) * (w - 2 * pad)
            py = h - pad - (y - ymin) / (ymax - ymin) * (h - 2 * pad)
            return px, py

        parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" '
                 'style="background:#fff;border:1px solid #ccc">'.format(w, h)]

        tx0, ty0 = to_px(tmin, target)
        tx1, ty1 = to_px(tmax, target)
        parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
                      'stroke="red" stroke-width="1.5" stroke-dasharray="4,3"/>'.format(tx0, ty0, tx1, ty1))
        parts.append('<text x="{:.1f}" y="{:.1f}" font-size="10" fill="red">target</text>'.format(tx1 - 40, ty1 - 4))

        pts_px = [to_px(t, y) for t, y in samples]
        points_str = " ".join("{:.1f},{:.1f}".format(px, py) for px, py in pts_px)
        parts.append('<polyline points="{}" fill="none" stroke="blue" stroke-width="2"/>'.format(points_str))

        parts.append('<text x="{}" y="14" font-size="11">position (turns) vs time (s)</text>'.format(pad))
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
        except (TypeError, ValueError) as e:
            ui.notify("Invalid config: {}".format(e), type="negative")

    # ------------------------------------------------------------------
    # Live polling loop
    # ------------------------------------------------------------------
    async def poll_live(self):
        if not (self.connected and self.odrv0 is not None):
            return
        try:
            turns0, turns1 = await run.io_bound(self._read_encoder_turns)
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
        scale = 5  # px per mm

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