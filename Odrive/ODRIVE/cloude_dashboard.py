"""
Five-Bar Linkage (Parallel SCARA) Control Dashboard
=====================================================

Hardware: ODrive v3.6, firmware 0.5.6, odrivetool/python lib 0.5.4
Two motors (axis0, axis1) mounted at fixed base pivots, each driving a
proximal link. Distal links connect the proximal link ends to a common
end-effector point, forming a five-bar (2 base + 2 proximal + 2 distal
meeting at the end effector) parallel linkage.

Features:
  - Connect / disconnect from ODrive
  - Clear errors, calibrate (motor + encoder offset), homing, idle, e-stop
  - Manual joint control (degrees) and raw motor control (turns)
  - Inverse kinematics: give end-effector (X, Y) -> compute & move joints
  - Forward kinematics: give joint angles -> compute end-effector (X, Y)
  - Live readback of motor position -> live end-effector position
  - Canvas visualization of the linkage geometry, updated live
  - Configurable link lengths, base separation, gear ratio, zero offsets,
    direction sign, and elbow (up/down) configuration per arm

Geometry convention:
  Motor A (axis0) is at (-L0/2, 0), Motor B (axis1) is at (+L0/2, 0).
  theta1 / theta2 are measured from +X axis, counter-clockwise, in degrees,
  and represent the angle of each PROXIMAL link relative to its motor base.

Run:
  python five_bar_dashboard.py
"""

import math
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import odrive
    from odrive.enums import *
    ODRIVE_AVAILABLE = True
except ImportError:
    ODRIVE_AVAILABLE = False

# ---------------------------------------------------------------------------
# Fallback state constants (works regardless of odrive.enums import success)
# ---------------------------------------------------------------------------
AXIS_STATE_IDLE = 1
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_HOMING = 11


# ---------------------------------------------------------------------------
# Kinematics helpers
# ---------------------------------------------------------------------------
def solve_arm_angle(anchor, target, l1, l2, elbow="up"):
    """
    Solve the base joint angle (radians) for one arm of a 2-link chain,
    where 'anchor' is the fixed motor pivot, 'target' is the end-effector
    point, l1 is the proximal link length, l2 is the distal link length.

    Returns the angle (radians) of the proximal link measured from +X axis.
    Raises ValueError if target is unreachable by this arm.
    """
    dx = target[0] - anchor[0]
    dy = target[1] - anchor[1]
    d = math.hypot(dx, dy)

    if d > (l1 + l2) or d < abs(l1 - l2) or d == 0:
        raise ValueError(
            f"Target unreachable for arm at {anchor}: distance={d:.2f}, "
            f"limits=[{abs(l1-l2):.2f}, {l1+l2:.2f}]"
        )

    base_angle = math.atan2(dy, dx)
    cos_val = (l1 ** 2 + d ** 2 - l2 ** 2) / (2 * l1 * d)
    cos_val = max(-1.0, min(1.0, cos_val))  # clamp for float safety
    elbow_angle = math.acos(cos_val)

    if elbow == "up":
        return base_angle + elbow_angle
    else:
        return base_angle - elbow_angle


def inverse_kinematics(x, y, params):
    """
    Given desired end-effector (x, y) in mm, compute (theta1_deg, theta2_deg)
    for the two motors, using the link parameters dict:
        L0, l1a, l2a, l1b, l2b, elbow1, elbow2
    """
    L0 = params["L0"]
    A = (-L0 / 2.0, 0.0)
    B = (L0 / 2.0, 0.0)

    theta1 = solve_arm_angle(A, (x, y), params["l1a"], params["l2a"], params["elbow1"])
    theta2 = solve_arm_angle(B, (x, y), params["l1b"], params["l2b"], params["elbow2"])

    return math.degrees(theta1), math.degrees(theta2)


def circle_intersection(p1, r1, p2, r2, branch="upper"):
    """
    Intersection of two circles: center p1 radius r1, center p2 radius r2.
    Returns the chosen intersection point based on 'branch' ('upper'/'lower'
    relative to the line joining the two centers).
    """
    x1, y1 = p1
    x2, y2 = p2
    d = math.hypot(x2 - x1, y2 - y1)

    if d > (r1 + r2) or d < abs(r1 - r2) or d == 0:
        raise ValueError(f"Circles do not intersect: d={d:.2f}, r1={r1}, r2={r2}")

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
    """
    Given motor angles (degrees), compute end-effector (x, y) in mm.
    Also returns the two elbow points (P1, P2) for visualization.
    """
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
# Main application
# ---------------------------------------------------------------------------
class FiveBarDashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Five-Bar Linkage (Parallel SCARA) Dashboard - ODrive")
        self.geometry("1180x760")

        self.odrv0 = None
        self.connected = False
        self.poll_job = None

        # Linkage / kinematics parameters (edit defaults or change in Config tab)
        self.params = {
            "L0": 100.0,     # base separation (mm)
            "l1a": 120.0,    # proximal link, arm A (axis0)
            "l2a": 160.0,    # distal link, arm A
            "l1b": 120.0,    # proximal link, arm B (axis1)
            "l2b": 160.0,    # distal link, arm B
            "elbow1": "up",
            "elbow2": "up",
            "fk_branch": "upper",
        }

        # Motor <-> joint-angle conversion
        self.axis_cfg = {
            0: {"gear_ratio": 1.0, "offset_turns": 0.0, "direction": 1},
            1: {"gear_ratio": 1.0, "offset_turns": 0.0, "direction": 1},
        }

        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(side="top", fill="x")

        self.connect_btn = ttk.Button(top, text="Connect to ODrive", command=self.connect_odrive)
        self.connect_btn.pack(side="left", padx=4)

        self.status_var = tk.StringVar(value="Not connected")
        ttk.Label(top, textvariable=self.status_var, foreground="red").pack(side="left", padx=10)

        self.estop_btn = ttk.Button(top, text="EMERGENCY STOP", command=self.emergency_stop)
        self.estop_btn.pack(side="right", padx=4)

        main = ttk.Frame(self)
        main.pack(side="top", fill="both", expand=True)

        left = ttk.Frame(main, padding=8)
        left.pack(side="left", fill="y")

        right = ttk.Frame(main, padding=8)
        right.pack(side="right", fill="both", expand=True)

        self._build_left_panel(left)
        self._build_right_panel(right)

        # Log box at the bottom
        log_frame = ttk.LabelFrame(self, text="Log", padding=6)
        log_frame.pack(side="bottom", fill="x")
        self.log_text = tk.Text(log_frame, height=7, state="disabled")
        self.log_text.pack(fill="x")

    def _build_left_panel(self, parent):
        nb = ttk.Notebook(parent)
        nb.pack(fill="y", expand=False)

        self.tab_joint = ttk.Frame(nb, padding=10)
        self.tab_ik = ttk.Frame(nb, padding=10)
        self.tab_fk = ttk.Frame(nb, padding=10)
        self.tab_cal = ttk.Frame(nb, padding=10)
        self.tab_cfg = ttk.Frame(nb, padding=10)

        nb.add(self.tab_joint, text="Joint Control")
        nb.add(self.tab_ik, text="Inverse Kinematics")
        nb.add(self.tab_fk, text="Forward Kinematics")
        nb.add(self.tab_cal, text="Calibration / Homing")
        nb.add(self.tab_cfg, text="Config")

        self._build_joint_tab(self.tab_joint)
        self._build_ik_tab(self.tab_ik)
        self._build_fk_tab(self.tab_fk)
        self._build_cal_tab(self.tab_cal)
        self._build_cfg_tab(self.tab_cfg)

    # -- Joint control tab -----------------------------------------------
    def _build_joint_tab(self, f):
        ttk.Label(f, text="Move by joint angle (degrees)", font=("", 10, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))

        ttk.Label(f, text="Theta1 (axis0):").grid(row=1, column=0, sticky="w")
        self.theta1_entry = ttk.Entry(f, width=12)
        self.theta1_entry.insert(0, "0")
        self.theta1_entry.grid(row=1, column=1, pady=2)

        ttk.Label(f, text="Theta2 (axis1):").grid(row=2, column=0, sticky="w")
        self.theta2_entry = ttk.Entry(f, width=12)
        self.theta2_entry.insert(0, "0")
        self.theta2_entry.grid(row=2, column=1, pady=2)

        ttk.Button(f, text="Move Joints", command=self.move_joints_from_entries).grid(
            row=3, column=0, columnspan=2, pady=8, sticky="ew")

        ttk.Separator(f, orient="horizontal").grid(row=4, column=0, columnspan=2, sticky="ew", pady=8)

        ttk.Label(f, text="Move by raw motor turns", font=("", 10, "bold")).grid(
            row=5, column=0, columnspan=2, sticky="w", pady=(0, 6))

        ttk.Label(f, text="Axis0 turns:").grid(row=6, column=0, sticky="w")
        self.turns0_entry = ttk.Entry(f, width=12)
        self.turns0_entry.insert(0, "0")
        self.turns0_entry.grid(row=6, column=1, pady=2)

        ttk.Label(f, text="Axis1 turns:").grid(row=7, column=0, sticky="w")
        self.turns1_entry = ttk.Entry(f, width=12)
        self.turns1_entry.insert(0, "0")
        self.turns1_entry.grid(row=7, column=1, pady=2)

        ttk.Button(f, text="Move Motors (raw turns)", command=self.move_raw_turns_from_entries).grid(
            row=8, column=0, columnspan=2, pady=8, sticky="ew")

        ttk.Separator(f, orient="horizontal").grid(row=9, column=0, columnspan=2, sticky="ew", pady=8)

        self.live_joint_var = tk.StringVar(value="theta1=--  theta2=--")
        ttk.Label(f, textvariable=self.live_joint_var).grid(row=10, column=0, columnspan=2, sticky="w")

    # -- Inverse kinematics tab -------------------------------------------
    def _build_ik_tab(self, f):
        ttk.Label(f, text="Target End-Effector Position (mm)", font=("", 10, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))

        ttk.Label(f, text="X:").grid(row=1, column=0, sticky="w")
        self.ik_x_entry = ttk.Entry(f, width=12)
        self.ik_x_entry.insert(0, "0")
        self.ik_x_entry.grid(row=1, column=1, pady=2)

        ttk.Label(f, text="Y:").grid(row=2, column=0, sticky="w")
        self.ik_y_entry = ttk.Entry(f, width=12)
        self.ik_y_entry.insert(0, "200")
        self.ik_y_entry.grid(row=2, column=1, pady=2)

        ttk.Label(f, text="Elbow 1 (axis0):").grid(row=3, column=0, sticky="w")
        self.ik_elbow1 = ttk.Combobox(f, values=["up", "down"], width=9, state="readonly")
        self.ik_elbow1.set(self.params["elbow1"])
        self.ik_elbow1.grid(row=3, column=1, pady=2)

        ttk.Label(f, text="Elbow 2 (axis1):").grid(row=4, column=0, sticky="w")
        self.ik_elbow2 = ttk.Combobox(f, values=["up", "down"], width=9, state="readonly")
        self.ik_elbow2.set(self.params["elbow2"])
        self.ik_elbow2.grid(row=4, column=1, pady=2)

        ttk.Button(f, text="Compute Only", command=self.compute_ik_only).grid(
            row=5, column=0, columnspan=2, pady=(10, 2), sticky="ew")
        ttk.Button(f, text="Compute & Move", command=self.compute_and_move_ik).grid(
            row=6, column=0, columnspan=2, pady=2, sticky="ew")

        self.ik_result_var = tk.StringVar(value="theta1=--  theta2=--")
        ttk.Label(f, textvariable=self.ik_result_var, wraplength=220).grid(
            row=7, column=0, columnspan=2, sticky="w", pady=(8, 0))

    # -- Forward kinematics tab -------------------------------------------
    def _build_fk_tab(self, f):
        ttk.Label(f, text="Joint Angles (degrees)", font=("", 10, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))

        ttk.Label(f, text="Theta1 (axis0):").grid(row=1, column=0, sticky="w")
        self.fk_t1_entry = ttk.Entry(f, width=12)
        self.fk_t1_entry.insert(0, "0")
        self.fk_t1_entry.grid(row=1, column=1, pady=2)

        ttk.Label(f, text="Theta2 (axis1):").grid(row=2, column=0, sticky="w")
        self.fk_t2_entry = ttk.Entry(f, width=12)
        self.fk_t2_entry.insert(0, "0")
        self.fk_t2_entry.grid(row=2, column=1, pady=2)

        ttk.Button(f, text="Compute FK", command=self.compute_fk_from_entries).grid(
            row=3, column=0, columnspan=2, pady=8, sticky="ew")
        ttk.Button(f, text="Use Current Motor Angles", command=self.compute_fk_from_live).grid(
            row=4, column=0, columnspan=2, pady=2, sticky="ew")

        self.fk_result_var = tk.StringVar(value="X=--  Y=--")
        ttk.Label(f, textvariable=self.fk_result_var, wraplength=220).grid(
            row=5, column=0, columnspan=2, sticky="w", pady=(8, 0))

    # -- Calibration / homing tab -----------------------------------------
    def _build_cal_tab(self, f):
        ttk.Button(f, text="Clear Errors", command=self.clear_errors).pack(fill="x", pady=3)
        ttk.Button(f, text="Calibrate Axis0 (motor + encoder)", command=lambda: self.calibrate_axis(0)).pack(fill="x", pady=3)
        ttk.Button(f, text="Calibrate Axis1 (motor + encoder)", command=lambda: self.calibrate_axis(1)).pack(fill="x", pady=3)
        ttk.Button(f, text="Calibrate Both", command=self.calibrate_both).pack(fill="x", pady=3)
        ttk.Separator(f, orient="horizontal").pack(fill="x", pady=8)
        ttk.Button(f, text="Enable Closed Loop Control (Both)", command=self.enable_closed_loop_both).pack(fill="x", pady=3)
        ttk.Button(f, text="Home Both Axes", command=self.home_both).pack(fill="x", pady=3)
        ttk.Button(f, text="Idle Both Axes", command=self.idle_both).pack(fill="x", pady=3)
        ttk.Separator(f, orient="horizontal").pack(fill="x", pady=8)
        ttk.Button(f, text="Show Errors", command=self.show_errors).pack(fill="x", pady=3)

    # -- Config tab ---------------------------------------------------------
    def _build_cfg_tab(self, f):
        row = 0
        self.cfg_entries = {}

        def add_row(label, key, default):
            nonlocal row
            ttk.Label(f, text=label).grid(row=row, column=0, sticky="w")
            e = ttk.Entry(f, width=12)
            e.insert(0, str(default))
            e.grid(row=row, column=1, pady=2)
            self.cfg_entries[key] = e
            row += 1

        ttk.Label(f, text="Link Geometry (mm)", font=("", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(0, 6))
        row += 1
        add_row("Base separation L0:", "L0", self.params["L0"])
        add_row("Proximal link A (l1a):", "l1a", self.params["l1a"])
        add_row("Distal link A (l2a):", "l2a", self.params["l2a"])
        add_row("Proximal link B (l1b):", "l1b", self.params["l1b"])
        add_row("Distal link B (l2b):", "l2b", self.params["l2b"])

        ttk.Separator(f, orient="horizontal").grid(row=row, column=0, columnspan=2, sticky="ew", pady=8)
        row += 1

        ttk.Label(f, text="Motor <-> Joint Conversion", font=("", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(0, 6))
        row += 1
        add_row("Axis0 gear ratio (motor turns/rev):", "gear0", self.axis_cfg[0]["gear_ratio"])
        add_row("Axis0 zero offset (turns):", "off0", self.axis_cfg[0]["offset_turns"])
        add_row("Axis0 direction (+1/-1):", "dir0", self.axis_cfg[0]["direction"])
        add_row("Axis1 gear ratio (motor turns/rev):", "gear1", self.axis_cfg[1]["gear_ratio"])
        add_row("Axis1 zero offset (turns):", "off1", self.axis_cfg[1]["offset_turns"])
        add_row("Axis1 direction (+1/-1):", "dir1", self.axis_cfg[1]["direction"])

        ttk.Button(f, text="Apply Config", command=self.apply_config).grid(
            row=row, column=0, columnspan=2, pady=10, sticky="ew")

    # -- Right panel: visualization -----------------------------------------
    def _build_right_panel(self, parent):
        ttk.Label(parent, text="Live Linkage Visualization", font=("", 11, "bold")).pack(anchor="w")
        self.canvas = tk.Canvas(parent, bg="white", width=760, height=600, highlightthickness=1,
                                 highlightbackground="gray")
        self.canvas.pack(fill="both", expand=True, pady=6)
        self.ee_pos_var = tk.StringVar(value="End effector: X=--  Y=--")
        ttk.Label(parent, textvariable=self.ee_pos_var, font=("", 10, "bold")).pack(anchor="w")

    # ------------------------------------------------------------------
    # Logging helper
    # ------------------------------------------------------------------
    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{ts}] {msg}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")
        print(msg)

    # ------------------------------------------------------------------
    # ODrive connection
    # ------------------------------------------------------------------
    def connect_odrive(self):
        if not ODRIVE_AVAILABLE:
            messagebox.showerror("odrive not installed", "The 'odrive' python package is not installed.")
            return

        def worker():
            self.log("Searching for ODrive...")
            try:
                self.odrv0 = odrive.find_any(timeout=15)
            except Exception as e:
                self.log(f"Connection failed: {e}")
                return
            self.connected = True
            self.status_var.set("Connected")
            self.log("Connected to ODrive.")
            self.after(0, lambda: self.status_var.set("Connected"))
            self.start_polling()

        threading.Thread(target=worker, daemon=True).start()

    # ------------------------------------------------------------------
    # Motor <-> joint conversion
    # ------------------------------------------------------------------
    def joint_deg_to_turns(self, axis_idx, angle_deg):
        cfg = self.axis_cfg[axis_idx]
        turns = cfg["offset_turns"] + cfg["direction"] * (angle_deg / 360.0) * cfg["gear_ratio"]
        return turns

    def turns_to_joint_deg(self, axis_idx, turns):
        cfg = self.axis_cfg[axis_idx]
        angle = ((turns - cfg["offset_turns"]) / cfg["gear_ratio"]) * 360.0 / cfg["direction"]
        return angle

    # ------------------------------------------------------------------
    # Motion commands
    # ------------------------------------------------------------------
    def require_connected(self):
        if not self.connected or self.odrv0 is None:
            messagebox.showwarning("Not connected", "Connect to the ODrive first.")
            return False
        return True

    def set_raw_turns(self, turns0, turns1):
        if not self.require_connected():
            return
        try:
            if self.odrv0.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                self.log("WARNING: axis0 not in CLOSED_LOOP_CONTROL. Enable it first (Calibration tab).")
            if self.odrv0.axis1.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                self.log("WARNING: axis1 not in CLOSED_LOOP_CONTROL. Enable it first (Calibration tab).")
            self.odrv0.axis0.controller.input_pos = turns0
            self.odrv0.axis1.controller.input_pos = turns1
            self.log(f"Sent raw turns -> axis0={turns0:.4f}, axis1={turns1:.4f}")
        except Exception as e:
            self.log(f"Move failed: {e}")

    def move_joints_from_entries(self):
        try:
            t1 = float(self.theta1_entry.get())
            t2 = float(self.theta2_entry.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Theta1/Theta2 must be numbers.")
            return
        turns0 = self.joint_deg_to_turns(0, t1)
        turns1 = self.joint_deg_to_turns(1, t2)
        self.set_raw_turns(turns0, turns1)

    def move_raw_turns_from_entries(self):
        try:
            turns0 = float(self.turns0_entry.get())
            turns1 = float(self.turns1_entry.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Turns must be numbers.")
            return
        self.set_raw_turns(turns0, turns1)

    # ------------------------------------------------------------------
    # Inverse kinematics actions
    # ------------------------------------------------------------------
    def _current_ik_params(self):
        p = dict(self.params)
        p["elbow1"] = self.ik_elbow1.get()
        p["elbow2"] = self.ik_elbow2.get()
        return p

    def compute_ik_only(self):
        try:
            x = float(self.ik_x_entry.get())
            y = float(self.ik_y_entry.get())
            t1, t2 = inverse_kinematics(x, y, self._current_ik_params())
        except Exception as e:
            self.ik_result_var.set(f"Error: {e}")
            self.log(f"IK error: {e}")
            return
        self.ik_result_var.set(f"theta1={t1:.2f} deg   theta2={t2:.2f} deg")
        self.log(f"IK computed for ({x}, {y}) -> theta1={t1:.2f}, theta2={t2:.2f}")

    def compute_and_move_ik(self):
        try:
            x = float(self.ik_x_entry.get())
            y = float(self.ik_y_entry.get())
            t1, t2 = inverse_kinematics(x, y, self._current_ik_params())
        except Exception as e:
            self.ik_result_var.set(f"Error: {e}")
            self.log(f"IK error: {e}")
            return
        self.ik_result_var.set(f"theta1={t1:.2f} deg   theta2={t2:.2f} deg")
        turns0 = self.joint_deg_to_turns(0, t1)
        turns1 = self.joint_deg_to_turns(1, t2)
        self.set_raw_turns(turns0, turns1)

    # ------------------------------------------------------------------
    # Forward kinematics actions
    # ------------------------------------------------------------------
    def compute_fk_from_entries(self):
        try:
            t1 = float(self.fk_t1_entry.get())
            t2 = float(self.fk_t2_entry.get())
            E, P1, P2 = forward_kinematics(t1, t2, self.params)
        except Exception as e:
            self.fk_result_var.set(f"Error: {e}")
            self.log(f"FK error: {e}")
            return
        self.fk_result_var.set(f"X={E[0]:.2f} mm   Y={E[1]:.2f} mm")
        self.draw_linkage(P1, P2, E)

    def compute_fk_from_live(self):
        if not self.require_connected():
            return
        try:
            turns0 = self.odrv0.axis0.encoder.pos_estimate
            turns1 = self.odrv0.axis1.encoder.pos_estimate
            t1 = self.turns_to_joint_deg(0, turns0)
            t2 = self.turns_to_joint_deg(1, turns1)
            self.fk_t1_entry.delete(0, "end")
            self.fk_t1_entry.insert(0, f"{t1:.2f}")
            self.fk_t2_entry.delete(0, "end")
            self.fk_t2_entry.insert(0, f"{t2:.2f}")
            E, P1, P2 = forward_kinematics(t1, t2, self.params)
            self.fk_result_var.set(f"X={E[0]:.2f} mm   Y={E[1]:.2f} mm")
            self.draw_linkage(P1, P2, E)
        except Exception as e:
            self.log(f"Read live angles failed: {e}")

    # ------------------------------------------------------------------
    # Calibration / homing / safety
    # ------------------------------------------------------------------
    def clear_errors(self):
        if not self.require_connected():
            return
        self.odrv0.clear_errors()
        self.log("Errors cleared.")

    def _run_state_blocking(self, axis, state, name, timeout=30):
        axis.requested_state = state
        start = time.time()
        while axis.current_state != AXIS_STATE_IDLE:
            if time.time() - start > timeout:
                self.log(f"{name}: TIMEOUT waiting for IDLE")
                return False
            time.sleep(0.1)
        ok = (axis.error == 0 and axis.motor.error == 0 and axis.encoder.error == 0)
        if not ok:
            self.log(f"{name}: error after state -> axis.error={axis.error}, "
                      f"motor.error={axis.motor.error}, encoder.error={axis.encoder.error}")
        else:
            self.log(f"{name}: OK")
        return ok

    def calibrate_axis(self, idx):
        if not self.require_connected():
            return

        def worker():
            axis = self.odrv0.axis0 if idx == 0 else self.odrv0.axis1
            name = f"axis{idx}"
            self.log(f"{name}: starting motor calibration...")
            if not self._run_state_blocking(axis, AXIS_STATE_MOTOR_CALIBRATION, name):
                return
            self.log(f"{name}: starting encoder offset calibration...")
            self._run_state_blocking(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, name)

        threading.Thread(target=worker, daemon=True).start()

    def calibrate_both(self):
        if not self.require_connected():
            return

        def worker():
            for idx, axis in [(0, self.odrv0.axis0), (1, self.odrv0.axis1)]:
                name = f"axis{idx}"
                self.log(f"{name}: starting motor calibration...")
                if not self._run_state_blocking(axis, AXIS_STATE_MOTOR_CALIBRATION, name):
                    continue
                self.log(f"{name}: starting encoder offset calibration...")
                self._run_state_blocking(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, name)

        threading.Thread(target=worker, daemon=True).start()

    def enable_closed_loop_both(self):
        if not self.require_connected():
            return
        try:
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.log("Requested CLOSED_LOOP_CONTROL on both axes.")
        except Exception as e:
            self.log(f"Enable closed loop failed: {e}")

    def home_both(self):
        if not self.require_connected():
            return

        def worker():
            self.log("Starting homing on both axes...")
            self.odrv0.axis0.requested_state = AXIS_STATE_HOMING
            self.odrv0.axis1.requested_state = AXIS_STATE_HOMING
            time.sleep(0.2)
            ok0 = self._run_state_blocking(self.odrv0.axis0, self.odrv0.axis0.current_state, "axis0", timeout=60) \
                if False else True  # state already requested above; just wait below
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

        threading.Thread(target=worker, daemon=True).start()

    def idle_both(self):
        if not self.require_connected():
            return
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
        self.log("Both axes set to IDLE.")

    def emergency_stop(self):
        if not self.connected or self.odrv0 is None:
            return
        try:
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
            self.log("EMERGENCY STOP: both axes set to IDLE.")
        except Exception as e:
            self.log(f"E-stop failed: {e}")

    def show_errors(self):
        if not self.require_connected():
            return
        try:
            from odrive.utils import dump_errors
            dump_errors(self.odrv0)
            self.log("Errors dumped to console (see terminal).")
        except Exception:
            a0, a1 = self.odrv0.axis0, self.odrv0.axis1
            self.log(f"axis0: error={a0.error}, motor.error={a0.motor.error}, encoder.error={a0.encoder.error}")
            self.log(f"axis1: error={a1.error}, motor.error={a1.motor.error}, encoder.error={a1.encoder.error}")

    # ------------------------------------------------------------------
    # Config apply
    # ------------------------------------------------------------------
    def apply_config(self):
        try:
            self.params["L0"] = float(self.cfg_entries["L0"].get())
            self.params["l1a"] = float(self.cfg_entries["l1a"].get())
            self.params["l2a"] = float(self.cfg_entries["l2a"].get())
            self.params["l1b"] = float(self.cfg_entries["l1b"].get())
            self.params["l2b"] = float(self.cfg_entries["l2b"].get())

            self.axis_cfg[0]["gear_ratio"] = float(self.cfg_entries["gear0"].get())
            self.axis_cfg[0]["offset_turns"] = float(self.cfg_entries["off0"].get())
            self.axis_cfg[0]["direction"] = float(self.cfg_entries["dir0"].get())
            self.axis_cfg[1]["gear_ratio"] = float(self.cfg_entries["gear1"].get())
            self.axis_cfg[1]["offset_turns"] = float(self.cfg_entries["off1"].get())
            self.axis_cfg[1]["direction"] = float(self.cfg_entries["dir1"].get())

            self.log("Config applied.")
        except ValueError as e:
            messagebox.showerror("Invalid config", f"Please check numeric fields.\n{e}")

    # ------------------------------------------------------------------
    # Live polling loop
    # ------------------------------------------------------------------
    def start_polling(self):
        self.poll_live()

    def poll_live(self):
        if self.connected and self.odrv0 is not None:
            try:
                turns0 = self.odrv0.axis0.encoder.pos_estimate
                turns1 = self.odrv0.axis1.encoder.pos_estimate
                t1 = self.turns_to_joint_deg(0, turns0)
                t2 = self.turns_to_joint_deg(1, turns1)
                self.live_joint_var.set(f"theta1={t1:.2f} deg   theta2={t2:.2f} deg")

                E, P1, P2 = forward_kinematics(t1, t2, self.params)
                self.ee_pos_var.set(f"End effector: X={E[0]:.2f} mm   Y={E[1]:.2f} mm")
                self.draw_linkage(P1, P2, E)
            except Exception:
                pass  # unreachable pose / transient read error, skip this frame

        self.poll_job = self.after(150, self.poll_live)

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def draw_linkage(self, P1, P2, E):
        c = self.canvas
        c.delete("all")
        w = int(c.winfo_width() or 760)
        h = int(c.winfo_height() or 600)
        cx, cy = w / 2, h * 0.75  # origin near lower-middle so linkage draws upward
        scale = 1.5  # px per mm, adjust if your workspace is larger/smaller

        def to_px(pt):
            return (cx + pt[0] * scale, cy - pt[1] * scale)

        L0 = self.params["L0"]
        A = (-L0 / 2.0, 0.0)
        B = (L0 / 2.0, 0.0)

        Apx = to_px(A)
        Bpx = to_px(B)
        P1px = to_px(P1)
        P2px = to_px(P2)
        Epx = to_px(E)

        # base line
        c.create_line(*Apx, *Bpx, fill="gray", width=2, dash=(4, 2))

        # proximal links
        c.create_line(*Apx, *P1px, fill="blue", width=4)
        c.create_line(*Bpx, *P2px, fill="green", width=4)

        # distal links
        c.create_line(*P1px, *Epx, fill="blue", width=3, dash=(6, 3))
        c.create_line(*P2px, *Epx, fill="green", width=3, dash=(6, 3))

        # pivots
        r = 5
        for pt, color, label in [(Apx, "black", "A (axis0)"), (Bpx, "black", "B (axis1)"),
                                  (P1px, "blue", "P1"), (P2px, "green", "P2")]:
            c.create_oval(pt[0] - r, pt[1] - r, pt[0] + r, pt[1] + r, fill=color)
            c.create_text(pt[0] + 10, pt[1] - 10, text=label, anchor="w", font=("", 8))

        # end effector
        r2 = 8
        c.create_oval(Epx[0] - r2, Epx[1] - r2, Epx[0] + r2, Epx[1] + r2, fill="red")
        c.create_text(Epx[0] + 12, Epx[1] - 12, text=f"E ({E[0]:.1f}, {E[1]:.1f})",
                       anchor="w", font=("", 9, "bold"))


if __name__ == "__main__":
    app = FiveBarDashboard()
    app.mainloop()