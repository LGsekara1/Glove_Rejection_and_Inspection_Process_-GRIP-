# ODrive v3.6 Dual-Axis Dashboard + 5-Bar SCARA Toolkit

A NiceGUI control dashboard for an **ODrive v3.6 (56V)** driving two **5065
270KV BLDC motors** with **AS5047P encoders in ABZ incremental mode**, plus a
5-bar parallel-linkage SCARA kinematics/trajectory toolkit built on top of it.

## Your hardware, as configured in this app

| | Axis 0 | Axis 1 |
|---|---|---|
| Motor | 5065, 270KV | 5065, 270KV |
| Encoder | AS5047P (ABZ incremental) | AS5047P (ABZ incremental) |
| Wiring | A/B/Z -> ENC0 header | A/B/Z -> ENC1 header |

Switched from SPI absolute to ABZ incremental to avoid EMI issues on the SPI
bus. The A/B/Z lines go straight into each axis's dedicated encoder header
(no GPIO wiring needed, unlike the SPI CS line the SPI-absolute setup used).
CPR and the index-search behavior are editable defaults
(`odrive_interface.EncoderParams`) and also editable in the **Configuration**
tab.

## Install

```bash
pip install -r requirements.txt
```

`odrive` needs `libusb`; on Linux you may also need udev rules
(`sudo odrivetool udev-setup` if you have `odrivetool` installed, or see
https://docs.odriverobotics.com for the current instructions) and to run
without `sudo` you generally need the udev rule installed rather than
running the dashboard as root.

## Run

```bash
cd app
python3 main.py
```

Then open **http://localhost:8080**.

There is a **"Run Simulated"** button on the Connection tab that spins up a
lightweight in-process physics model of both axes (2nd-order servo
response). Use it to explore every tab -- calibration, gains, graphs,
trajectories, SCARA workspace -- with zero hardware attached, and to learn
the workflow before touching the real robot.

## Tabs

1. **Connection** -- connect over USB or run simulated; live vbus, per-axis
   state, calibration flag, and error readout.
2. **Configuration** -- pole pairs, Kv (torque constant is derived
   automatically as `8.27 / Kv`), current limit, calibration current,
   encoder bandwidth, plus board-level brake resistor / trip-voltage
   settings. Includes Save Configuration / Erase Configuration / Reboot.
3. **Calibration** -- full calibration sequence, motor-only, encoder index
   search, encoder-offset only, enter closed-loop control, and idle, per
   axis, with a live activity log. Unlike absolute mode, ABZ incremental
   loses its position reference on every power cycle, so `startup_encoder_index_search`
   is on and the axis re-homes via the Z pulse on each boot (motor
   calibration itself still only needs to be redone if you change the
   motor/wiring).
4. **Control & Gains** -- control mode (Position / Velocity / Torque /
   Voltage) x input mode (Passthrough / Pos Filter / Trap Traj / Vel Ramp /
   Torque Ramp), pos/vel/vel-integrator gains, velocity limit, input filter
   bandwidth, a manual setpoint field, and a one-click step-test (watch the
   response on the Graphs tab).
5. **Graphs** -- live position / velocity / current(Iq), each independently
   toggleable, per-axis toggleable, adjustable rolling time window,
   pause/resume, clear.
6. **Trajectory** -- build multi-waypoint trajectories in **joint space**
   (axis0/axis1 turns) or **Cartesian space** (x/y, auto-converted through
   the 5-bar inverse kinematics), as a **trapezoidal** (stop-to-stop,
   axis-synchronized) or **cubic spline** (smooth, velocity-continuous)
   profile. Generate + preview, then stream it to the ODrive as
   position + velocity-feed-forward setpoints at a fixed sample rate.
7. **5-Bar SCARA** -- link-length geometry, elbow-sign flip (fixes a
   mirrored solution if it doesn't match your physical assembly),
   optional joint-travel limits, a shaded reachable-workspace plot, a
   coordinate -> angle calculator with a "Move Here" button, and a live
   schematic of the linkage driven by real encoder feedback.

## 5-bar linkage convention

```
        A -------- P -------- B
       /                        \
     L1a  (crank, axis0)   L1b   (crank, axis1)
     /                            \
   O1 -------------- d ------------- O2
```

`O1 = (-d/2, 0)`, `O2 = (d/2, 0)`. Only `theta1` (axis0) and `theta2`
(axis1) are driven; the elbow points `A`/`B` are passive. Inverse kinematics
decouples into two independent 2-link (law-of-cosines) problems, one per
side. If your computed pose comes out mirrored vs. the real arm, flip
`elbow_sign_a` / `elbow_sign_b` in the SCARA tab.

**Units:** linkage lengths are meters, angles are radians internally /
degrees in the UI, and `pos` sent to the ODrive is always in **turns**
(1 motor turn = 1 joint revolution is assumed -- if you add a belt or gear
reduction between motor and joint, scale the angle by your gear ratio
before calling `set_input_pos`, e.g. in `ui_scara.py`'s `move_here()` and in
`trajectory._xy_traj_to_joint`).

## Notes / things worth knowing before you spin motors

- **Firmware**: v3.6 boards top out at ODrive firmware **0.5.6** (the last
  release supporting the v3.x hardware line). The `odrive` pip package
  auto-detects the firmware's object schema at connect time, so the same
  package works against both the old and new firmware generations --
  just make sure you flash 0.5.6 (or whatever the latest v3.x-compatible
  release is) rather than assuming the newest pip package version implies
  newest-compatible firmware.
- **Pole pairs**: defaulted to 7 (typical for a 5065 outrunner / 14 poles).
  Confirm against your specific motor's datasheet before calibrating.
- **Calibration safety**: the rotor must spin freely (no belt/linkage
  connected, or a linkage that can't hit a hard stop) the first time you run
  Full Calibration Sequence.
- **Streaming rate**: the Trajectory tab streams `input_pos` (+ velocity
  feed-forward) over USB at whatever `sample dt` you choose. Real USB round
  trips are usually reliable up to a few hundred Hz; 10 ms (100 Hz) is a
  safe starting point. `Pos Filter` input mode is recommended for streamed
  trajectories since it smooths the zero-order-hold steps between samples;
  `Passthrough` is fine too at faster sample rates.
- **Workspace plot** is a dense-grid reachability sample (annulus
  intersection per side, further clipped by any joint-travel limits you
  set) -- it does not model link-link physical collision, only reach.

## File layout

```
app/
  odrive_interface.py   hardware layer: connection, config, calibration,
                         gains, control modes, telemetry, + a full
                         simulated backend for hardware-free testing
  kinematics.py          5-bar forward/inverse kinematics + workspace grid
  trajectory.py          trapezoidal + cubic-spline generators, joint-space
                          and Cartesian(SCARA), sampling for streaming
  state.py               single shared AppState (config, buffers, geometry,
                          trajectory state) used by every panel
  ui_connection.py       Connection tab
  ui_config.py           Configuration tab
  ui_calibration.py      Calibration tab
  ui_control.py          Control & Gains tab
  ui_graphs.py           Graphs tab
  ui_trajectory.py       Trajectory tab
  ui_scara.py            5-Bar SCARA tab
  main.py                page layout / entry point
requirements.txt
```
