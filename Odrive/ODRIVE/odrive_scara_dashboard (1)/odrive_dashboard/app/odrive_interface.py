"""
odrive_interface.py
====================
Hardware abstraction layer for an ODrive v3.6 (56V, dual axis) running the
0.5.x firmware/object model, targeting the `odrive==0.5.4` pip package
specifically (the 0.6.x package/firmware line uses a different object model --
e.g. `MOTOR_TYPE_PMSM_CURRENT_CONTROL` instead of `MOTOR_TYPE_HIGH_CURRENT`,
and reworked config trees -- so the two are not drop-in compatible).

Wiring assumed (as specified by the user):
    - Motors: 2x 5065 270KV BLDC
    - Encoders: 2x AS5047P, ABZ incremental mode (switched from SPI absolute
      due to EMI issues -- A/B/Z lines go straight into each axis's dedicated
      encoder header (ENC0 / ENC1) on the v3.6 board, no GPIO wiring needed).
    - AS5047P ABI resolution left at its factory-default 4096 steps/rev
      (binary mode), which is also ODrive's `encoder.config.cpr` value.
    - Z (index) pulse wired in on both encoders, so `use_index=True` and an
      index search runs as part of calibration / on startup.

This module never blocks the GUI event loop: every call that talks to the
ODrive is either fast (attribute get/set) or is wrapped by the caller in
`run.io_bound(...)` from NiceGUI. A `SimulatedAxis`/`SimulatedODrive` pair is
provided so the whole dashboard (graphs, trajectories, kinematics) can be
exercised with no hardware attached.
"""
from __future__ import annotations

import math
import time
import threading
from dataclasses import dataclass, field
from typing import Callable, Optional

try:
    import odrive
    from odrive.enums import (
        AXIS_STATE_IDLE,
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
        AXIS_STATE_MOTOR_CALIBRATION,
        AXIS_STATE_ENCODER_INDEX_SEARCH,
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
        AXIS_STATE_CLOSED_LOOP_CONTROL,
        CONTROL_MODE_POSITION_CONTROL,
        CONTROL_MODE_VELOCITY_CONTROL,
        CONTROL_MODE_TORQUE_CONTROL,
        CONTROL_MODE_VOLTAGE_CONTROL,
        INPUT_MODE_PASSTHROUGH,
        INPUT_MODE_POS_FILTER,
        INPUT_MODE_VEL_RAMP,
        INPUT_MODE_TORQUE_RAMP,
        INPUT_MODE_TRAP_TRAJ,
        INPUT_MODE_INACTIVE,
        ENCODER_MODE_INCREMENTAL,
        MOTOR_TYPE_HIGH_CURRENT,
    )
    from odrive.utils import dump_errors
    ODRIVE_AVAILABLE = True
except Exception:  # pragma: no cover - odrive lib / libusb not installed
    ODRIVE_AVAILABLE = False
    # Fallback numeric values so the rest of the module still imports and the
    # simulator still works when the real odrive package/driver isn't present.
    AXIS_STATE_IDLE = 1
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
    AXIS_STATE_MOTOR_CALIBRATION = 4
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    CONTROL_MODE_POSITION_CONTROL = 3
    CONTROL_MODE_VELOCITY_CONTROL = 2
    CONTROL_MODE_TORQUE_CONTROL = 1
    CONTROL_MODE_VOLTAGE_CONTROL = 0
    INPUT_MODE_INACTIVE = 0
    INPUT_MODE_PASSTHROUGH = 1
    INPUT_MODE_VEL_RAMP = 2
    INPUT_MODE_POS_FILTER = 3
    INPUT_MODE_MIX_CHANNELS = 4
    INPUT_MODE_TRAP_TRAJ = 5
    INPUT_MODE_TORQUE_RAMP = 6
    ENCODER_MODE_INCREMENTAL = 0
    MOTOR_TYPE_HIGH_CURRENT = 0


CONTROL_MODES = {
    "Position": CONTROL_MODE_POSITION_CONTROL,
    "Velocity": CONTROL_MODE_VELOCITY_CONTROL,
    "Torque": CONTROL_MODE_TORQUE_CONTROL,
    "Voltage": CONTROL_MODE_VOLTAGE_CONTROL,
}

INPUT_MODES = {
    "Passthrough": INPUT_MODE_PASSTHROUGH,
    "Pos Filter (recommended for streamed trajectories)": INPUT_MODE_POS_FILTER,
    "Trap Traj (on-device point-to-point)": INPUT_MODE_TRAP_TRAJ,
    "Vel Ramp": INPUT_MODE_VEL_RAMP,
    "Torque Ramp": INPUT_MODE_TORQUE_RAMP,
    "Inactive": INPUT_MODE_INACTIVE,
}




@dataclass
class MotorParams:
    pole_pairs: int = 7          # typical for a 5065 outrunner (14 poles / 7 pole pairs)
    kv: float = 270.0
    current_lim: float = 20.0
    calibration_current: float = 8.0
    resistance_calib_max_voltage: float = 4.0
    torque_constant: float = field(init=False, default=0.0)

    def __post_init__(self):
        self.torque_constant = 8.27 / self.kv if self.kv else 0.0


@dataclass
class EncoderParams:
    mode: int = ENCODER_MODE_INCREMENTAL
    # AS5047P ABI factory default is 4096 steps/rev (binary mode, 1024 ppr x4
    # quadrature) -- matches ODrive's counts-per-rotation directly. If you've
    # reprogrammed the sensor's ABIRES bits to a different resolution, update
    # this to match (valid binary-mode steps: 4096/2048/1024/512/256/128/... ).
    cpr: int = 4096
    bandwidth: float = 1000.0
    use_index: bool = True  # Z line wired in -> do an index search for a
                             # repeatable absolute reference each power-up.


@dataclass
class AxisTelemetry:
    pos: float = 0.0          # turns
    vel: float = 0.0          # turns/s
    iq_measured: float = 0.0  # A
    iq_setpoint: float = 0.0
    current_state: int = AXIS_STATE_IDLE
    axis_error: int = 0
    motor_error: int = 0
    encoder_error: int = 0
    controller_error: int = 0
    is_calibrated: bool = False
    timestamp: float = 0.0


class AxisHandle:
    """Wraps either a real odrive axis object or a SimulatedAxis with an
    identical interface used by the rest of the app."""

    def __init__(self, axis_num: int, backend, is_sim: bool, parent_odrv=None):
        self.axis_num = axis_num
        self._axis = backend
        self.is_sim = is_sim
        # odrive.utils.dump_errors() (0.5.4) expects the top-level odrv object
        # (it walks odrv.axis0 / odrv.axis1 itself), not an individual axis.
        self._parent_odrv = parent_odrv

    # -- configuration --------------------------------------------------
    def configure_motor(self, p: MotorParams):
        a = self._axis
        a.motor.config.pole_pairs = p.pole_pairs
        a.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        a.motor.config.current_lim = p.current_lim
        a.motor.config.calibration_current = p.calibration_current
        a.motor.config.resistance_calib_max_voltage = p.resistance_calib_max_voltage
        a.motor.config.torque_constant = 8.27 / p.kv if p.kv else 0.0

    def configure_encoder(self, e: EncoderParams):
        a = self._axis
        a.encoder.config.mode = e.mode
        a.encoder.config.cpr = e.cpr
        a.encoder.config.bandwidth = e.bandwidth
        a.encoder.config.use_index = e.use_index
        # With an index (Z) channel wired in, do an index search on every
        # power-up/reboot so the axis re-establishes a repeatable absolute
        # reference before you have to run offset calibration again.
        a.config.startup_encoder_index_search = e.use_index

    def set_gains(self, pos_gain: float, vel_gain: float, vel_integrator_gain: float,
                  vel_limit: Optional[float] = None, input_filter_bandwidth: Optional[float] = None):
        c = self._axis.controller.config
        c.pos_gain = pos_gain
        c.vel_gain = vel_gain
        c.vel_integrator_gain = vel_integrator_gain
        if vel_limit is not None:
            c.vel_limit = vel_limit
        if input_filter_bandwidth is not None:
            c.input_filter_bandwidth = input_filter_bandwidth

    def get_gains(self):
        c = self._axis.controller.config
        return dict(pos_gain=c.pos_gain, vel_gain=c.vel_gain,
                    vel_integrator_gain=c.vel_integrator_gain, vel_limit=c.vel_limit)

    def set_trap_traj_limits(self, vel_limit: float, accel_limit: float, decel_limit: float):
        t = self._axis.trap_traj.config
        t.vel_limit = vel_limit
        t.accel_limit = accel_limit
        t.decel_limit = decel_limit

    # -- state machine ----------------------------------------------------
    def request_state(self, state: int):
        self._axis.requested_state = state

    def current_state(self) -> int:
        return self._axis.current_state

    def is_calibrated(self) -> bool:
        try:
            return bool(self._axis.motor.is_calibrated) and bool(self._axis.encoder.is_ready)
        except Exception:
            return False

    # -- control ------------------------------------------------------
    def set_control_mode(self, control_mode: int, input_mode: int):
        self._axis.controller.config.control_mode = control_mode
        self._axis.controller.config.input_mode = input_mode

    def set_input_pos(self, pos: float, vel_ff: float = 0.0, torque_ff: float = 0.0):
        self._axis.controller.input_pos = pos
        self._axis.controller.input_vel = vel_ff
        self._axis.controller.input_torque = torque_ff

    def set_input_vel(self, vel: float, torque_ff: float = 0.0):
        self._axis.controller.input_vel = vel
        self._axis.controller.input_torque = torque_ff

    def set_input_torque(self, torque: float):
        self._axis.controller.input_torque = torque

    # -- telemetry ------------------------------------------------------
    def read_telemetry(self) -> AxisTelemetry:
        a = self._axis
        try:
            return AxisTelemetry(
                pos=a.encoder.pos_estimate,
                vel=a.encoder.vel_estimate,
                iq_measured=a.motor.current_control.Iq_measured,
                iq_setpoint=a.motor.current_control.Iq_setpoint,
                current_state=a.current_state,
                axis_error=a.error,
                motor_error=a.motor.error,
                encoder_error=a.encoder.error,
                controller_error=a.controller.error,
                is_calibrated=self.is_calibrated(),
                timestamp=time.time(),
            )
        except Exception:
            return AxisTelemetry(timestamp=time.time())

    def errors_text(self) -> str:
        if self.is_sim:
            return "no errors (simulated)"
        try:
            import io, re
            buf = io.StringIO()
            # odrive.utils.dump_errors(odrv, clear=False, printfunc=print) in
            # 0.5.4 walks odrv.axis0/axis1 itself and takes a printfunc, so we
            # pass the top-level odrv (not this axis) and capture via printfunc
            # rather than redirecting stdout.
            dump_errors(self._parent_odrv, clear=False, printfunc=lambda s: buf.write(s + "\n"))
            # Strip ANSI colour codes (dump_errors colourizes for a terminal).
            return re.sub(r"\x1b\[[0-9;]*m", "", buf.getvalue())
        except Exception:
            return f"axis={self._axis.error} motor={self._axis.motor.error} encoder={self._axis.encoder.error}"


class ODriveManager:
    """Top level connection manager. Handles real hardware discovery and
    falls back to a software simulator so the dashboard is always usable."""

    def __init__(self):
        self.odrv = None
        self.axes: dict[int, AxisHandle] = {}
        self.connected = False
        self.is_sim = False
        self.serial_number = None
        self._lock = threading.Lock()

    # -- connection -----------------------------------------------------
    def connect(self, timeout: float = 8.0, simulate: bool = False) -> str:
        with self._lock:
            if simulate or not ODRIVE_AVAILABLE:
                self.odrv = SimulatedODrive()
                self.axes = {
                    0: AxisHandle(0, self.odrv.axis0, is_sim=True, parent_odrv=self.odrv),
                    1: AxisHandle(1, self.odrv.axis1, is_sim=True, parent_odrv=self.odrv),
                }
                self.connected = True
                self.is_sim = True
                self.serial_number = "SIMULATED"
                return "Connected to simulated ODrive"
            try:
                self.odrv = odrive.find_any(timeout=timeout)
                self.axes = {
                    0: AxisHandle(0, self.odrv.axis0, is_sim=False, parent_odrv=self.odrv),
                    1: AxisHandle(1, self.odrv.axis1, is_sim=False, parent_odrv=self.odrv),
                }
                self.connected = True
                self.is_sim = False
                self.serial_number = format(self.odrv.serial_number, "x") if hasattr(self.odrv, "serial_number") else "unknown"
                return f"Connected to ODrive {self.serial_number}"
            except Exception as ex:
                self.connected = False
                raise RuntimeError(f"Could not find an ODrive: {ex}")

    def disconnect(self):
        self.odrv = None
        self.axes = {}
        self.connected = False

    # -- board level config ----------------------------------------------
    def configure_board(self, brake_resistance: float, enable_brake_resistor: bool,
                         dc_bus_overvoltage_trip_level: float, dc_max_negative_current: float):
        if not self.connected:
            return
        c = self.odrv.config
        c.brake_resistance = brake_resistance
        c.enable_brake_resistor = enable_brake_resistor
        c.dc_bus_overvoltage_trip_level = dc_bus_overvoltage_trip_level
        c.dc_max_negative_current = dc_max_negative_current

    def vbus_voltage(self) -> float:
        if not self.connected:
            return 0.0
        try:
            return float(self.odrv.vbus_voltage)
        except Exception:
            return 0.0

    def save_configuration(self):
        if self.connected and not self.is_sim:
            self.odrv.save_configuration()

    def erase_configuration(self):
        if self.connected and not self.is_sim:
            self.odrv.erase_configuration()

    def reboot(self):
        if self.connected and not self.is_sim:
            try:
                self.odrv.reboot()
            except Exception:
                pass  # connection drops as expected on reboot

    def axis(self, n: int) -> AxisHandle:
        return self.axes[n]


# =====================================================================
# Simulator -- lets the whole dashboard be developed/tested with no
# hardware attached. Mimics a 2nd order position-controlled servo.
# =====================================================================
class _SimConfig:
    def __init__(self):
        self.pole_pairs = 7
        self.motor_type = MOTOR_TYPE_HIGH_CURRENT
        self.current_lim = 20.0
        self.calibration_current = 8.0
        self.resistance_calib_max_voltage = 4.0
        self.torque_constant = 0.03
        self.mode = ENCODER_MODE_INCREMENTAL
        self.cpr = 4096
        self.bandwidth = 1000.0
        self.use_index = True
        self.pos_gain = 20.0
        self.vel_gain = 0.16
        self.vel_integrator_gain = 0.32
        self.vel_limit = 10.0
        self.input_filter_bandwidth = 2.0
        self.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.input_mode = INPUT_MODE_PASSTHROUGH
        self.startup_encoder_index_search = True


class _Sub:
    """Generic attribute bag used to mimic the nested odrive object tree."""
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class SimulatedAxis:
    def __init__(self):
        self.config = _SimConfig()
        self.motor = _Sub(config=self.config, is_calibrated=True,
                           error=0, current_control=_Sub(Iq_measured=0.0, Iq_setpoint=0.0))
        self.encoder = _Sub(config=self.config, is_ready=True, error=0,
                             pos_estimate=0.0, vel_estimate=0.0)
        self.controller = _Sub(config=self.config, error=0,
                                input_pos=0.0, input_vel=0.0, input_torque=0.0)
        self.trap_traj = _Sub(config=_Sub(vel_limit=2.0, accel_limit=5.0, decel_limit=5.0))
        self.current_state = AXIS_STATE_IDLE
        self.requested_state = AXIS_STATE_IDLE
        self.error = 0
        self._pos = 0.0
        self._vel = 0.0
        self._last_t = time.time()
        self._calib_until = 0.0

    def __setattr__(self, key, value):
        if key == "requested_state":
            object.__setattr__(self, key, value)
            self._handle_state_request(value)
        else:
            object.__setattr__(self, key, value)

    def _handle_state_request(self, state):
        if state in (AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_MOTOR_CALIBRATION,
                     AXIS_STATE_ENCODER_INDEX_SEARCH, AXIS_STATE_ENCODER_OFFSET_CALIBRATION):
            self.current_state = state
            self._calib_until = time.time() + 2.0  # simulate a couple seconds of calibration
        elif state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.current_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        elif state == AXIS_STATE_IDLE:
            self.current_state = AXIS_STATE_IDLE

    def step(self):
        """Advance the mock 2nd-order servo model. Called by the polling loop."""
        now = time.time()
        dt = max(1e-4, min(0.1, now - self._last_t))
        self._last_t = now

        if self._calib_until and now < self._calib_until:
            pass  # "calibrating"
        elif self._calib_until and now >= self._calib_until:
            self._calib_until = 0.0
            self.current_state = AXIS_STATE_IDLE

        if self.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            c = self.config
            if c.control_mode == CONTROL_MODE_POSITION_CONTROL:
                target = self.controller.input_pos
                vel_ff = self.controller.input_vel
                pos_err = target - self._pos
                # Simplified 2nd-order model tuned so default gains look like a
                # sensibly-damped real servo. vel_gain contributes extra viscous
                # damping on top of a small fixed mechanical friction term.
                accel = c.pos_gain * pos_err + (c.vel_gain * 40.0 + 1.5) * (vel_ff - self._vel)
            elif c.control_mode == CONTROL_MODE_VELOCITY_CONTROL:
                target_vel = self.controller.input_vel
                accel = (target_vel - self._vel) * 10.0
            elif c.control_mode == CONTROL_MODE_TORQUE_CONTROL:
                torque = self.controller.input_torque
                accel = torque / max(c.torque_constant, 1e-6) * 0.05
            else:
                accel = 0.0
            self._vel += accel * dt
            self._vel = max(-c.vel_limit * 1.5, min(c.vel_limit * 1.5, self._vel))
            self._pos += self._vel * dt

        self.encoder.pos_estimate = self._pos
        self.encoder.vel_estimate = self._vel
        iq = self._vel * 0.6 + (self.controller.input_torque / max(self.config.torque_constant, 1e-6) if self.config.control_mode == CONTROL_MODE_TORQUE_CONTROL else 0.0)
        self.motor.current_control.Iq_measured = iq + (math.sin(now * 13) * 0.05)
        self.motor.current_control.Iq_setpoint = iq


class SimulatedODrive:
    def __init__(self):
        self.axis0 = SimulatedAxis()
        self.axis1 = SimulatedAxis()
        self.config = _Sub(brake_resistance=2.0, enable_brake_resistor=True,
                            dc_bus_overvoltage_trip_level=59.0, dc_max_negative_current=-1.0)
        self.vbus_voltage = 55.8
        self.serial_number = 0x51A1

    def save_configuration(self):
        pass

    def erase_configuration(self):
        pass

    def reboot(self):
        pass

    def _tick(self):
        self.axis0.step()
        self.axis1.step()


def start_simulation_clock(manager: ODriveManager, hz: float = 200.0):
    """Background thread that advances the simulated physics regardless of
    whether the GUI is currently polling -- keeps behaviour close to real
    hardware which runs its control loop independently at 8kHz."""
    interval = 1.0 / hz

    def loop():
        while manager.connected and manager.is_sim:
            try:
                manager.odrv._tick()
            except Exception:
                pass
            time.sleep(interval)

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t
