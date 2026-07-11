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
"""

import math
import time

from nicegui import ui, run

try:
    import odrive
    from odrive.enums import *  # noqa: F401,F403
    ODRIVE_AVAILABLE = True
except ImportError:
    ODRIVE_AVAILABLE = False

# ---------------------------------------------------------------------------
# Fallback state constants
# ---------------------------------------------------------------------------
AXIS_STATE_IDLE = 1
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_HOMING = 11


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

        self.build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def build_ui(self):
        ui.page_title("Five-Bar Linkage Dashboard")

        with ui.row().classes("w-full items-center justify-between p-2"):
            self.connect_btn = ui.button("Connect to ODrive", on_click=self.connect_odrive)
            self.status_label = ui.label("Not connected").classes("text-red-600 font-bold")
            ui.button("EMERGENCY STOP", on_click=self.emergency_stop,
                       color="red").classes("font-bold")

        with ui.row().classes("w-full no-wrap"):
            # ---------------- Left: control tabs ----------------
            with ui.column().classes("basis-1/3 min-w-[380px]"):
                with ui.tabs().classes("w-full") as tabs:
                    t_joint = ui.tab("Joint Control")
                    t_ik = ui.tab("Inverse Kinematics")
                    t_fk = ui.tab("Forward Kinematics")
                    t_cal = ui.tab("Calibration / Homing")
                    t_cfg = ui.tab("Config")

                with ui.tab_panels(tabs, value=t_joint).classes("w-full"):
                    with ui.tab_panel(t_joint):
                        self.build_joint_tab()
                    with ui.tab_panel(t_ik):
                        self.build_ik_tab()
                    with ui.tab_panel(t_fk):
                        self.build_fk_tab()
                    with ui.tab_panel(t_cal):
                        self.build_cal_tab()
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
        ui.label("Move by joint angle (degrees)").classes("font-bold")
        self.theta1_input = ui.number(label="Theta1 (axis0)", value=0.0, format="%.2f")
        self.theta2_input = ui.number(label="Theta2 (axis1)", value=0.0, format="%.2f")
        ui.button("Move Joints", on_click=self.move_joints_from_inputs).classes("w-full")

        ui.separator()

        ui.label("Move by raw motor turns").classes("font-bold")
        self.turns0_input = ui.number(label="Axis0 turns", value=0.0, format="%.4f")
        self.turns1_input = ui.number(label="Axis1 turns", value=0.0, format="%.4f")
        ui.button("Move Motors (raw turns)", on_click=self.move_raw_turns_from_inputs).classes("w-full")

        ui.separator()
        self.live_joint_label = ui.label("theta1=--  theta2=--")

    def build_ik_tab(self):
        ui.label("Target End-Effector Position (mm)").classes("font-bold")
        self.ik_x_input = ui.number(label="X", value=0.0, format="%.2f")
        self.ik_y_input = ui.number(label="Y", value=200.0, format="%.2f")
        self.ik_elbow1_select = ui.select(["up", "down"], value=self.params["elbow1"], label="Elbow 1 (axis0)")
        self.ik_elbow2_select = ui.select(["up", "down"], value=self.params["elbow2"], label="Elbow 2 (axis1)")

        ui.button("Compute Only", on_click=self.compute_ik_only).classes("w-full")
        ui.button("Compute & Move", on_click=self.compute_and_move_ik).classes("w-full")

        self.ik_result_label = ui.label("theta1=--  theta2=--")

    def build_fk_tab(self):
        ui.label("Joint Angles (degrees)").classes("font-bold")
        self.fk_t1_input = ui.number(label="Theta1 (axis0)", value=0.0, format="%.2f")
        self.fk_t2_input = ui.number(label="Theta2 (axis1)", value=0.0, format="%.2f")

        ui.button("Compute FK", on_click=self.compute_fk_from_inputs).classes("w-full")
        ui.button("Use Current Motor Angles", on_click=self.compute_fk_from_live).classes("w-full")

        self.fk_result_label = ui.label("X=--  Y=--")

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
    async def connect_odrive(self):
        if not ODRIVE_AVAILABLE:
            ui.notify("The 'odrive' python package is not installed.", type="negative")
            return
        self.log("Searching for ODrive...")
        try:
            self.odrv0 = await run.io_bound(odrive.find_any, timeout=15)
        except Exception as e:
            self.log("Connection failed: {}".format(e))
            ui.notify("Connection failed: {}".format(e), type="negative")
            return
        self.connected = True
        self.status_label.text = "Connected"
        self.status_label.classes(remove="text-red-600", add="text-green-600")
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
    # Motion commands
    # ------------------------------------------------------------------
    def require_connected(self):
        if not self.connected or self.odrv0 is None:
            ui.notify("Connect to the ODrive first.", type="warning")
            return False
        return True

    async def set_raw_turns(self, turns0, turns1):
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

    async def move_joints_from_inputs(self):
        try:
            t1 = float(self.theta1_input.value)
            t2 = float(self.theta2_input.value)
        except (TypeError, ValueError):
            ui.notify("Theta1/Theta2 must be numbers.", type="negative")
            return
        turns0 = self.joint_deg_to_turns(0, t1)
        turns1 = self.joint_deg_to_turns(1, t2)
        await self.set_raw_turns(turns0, turns1)

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
        try:
            x = float(self.ik_x_input.value)
            y = float(self.ik_y_input.value)
            t1, t2 = inverse_kinematics(x, y, self._current_ik_params())
        except Exception as e:
            self.ik_result_label.text = "Error: {}".format(e)
            self.log("IK error: {}".format(e))
            return
        self.ik_result_label.text = "theta1={:.2f} deg   theta2={:.2f} deg".format(t1, t2)
        turns0 = self.joint_deg_to_turns(0, t1)
        turns1 = self.joint_deg_to_turns(1, t2)
        await self.set_raw_turns(turns0, turns1)

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

    def _read_encoder_turns(self):
        return self.odrv0.axis0.encoder.pos_estimate, self.odrv0.axis1.encoder.pos_estimate

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
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        try:
            await run.io_bound(_do)
            self.log("Requested CLOSED_LOOP_CONTROL on both axes.")
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

    def emergency_stop(self):
        # Deliberately synchronous / not routed through run.io_bound so it
        # fires immediately even if other background operations are busy.
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
        scale = 5 #1.3  # px per mm

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
