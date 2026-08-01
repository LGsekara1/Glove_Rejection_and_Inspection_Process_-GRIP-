# Five-Bar Parallel SCARA Control Dashboard

Native **PySide6 + pyqtgraph** rewrite for an ODrive v3.6 / firmware 0.5.6 controlled five-bar linkage.

## Safety first

This software can command powered machinery. Test with motors mechanically disconnected or current-limited first. Keep a physical power-disconnect/E-stop in the real machine. The on-screen E-stop is an additional software control, not a replacement for a safety-rated circuit.

## Implemented features

- ODrive connection with 6 retries and a hard timeout wrapper
- Joint-space firmware `TRAP_TRAJ` motion with two-axis time synchronisation
- Raw-turn small-increment nudges
- IK, FK, numerical Jacobian and singularity de-rating
- Cartesian deadman jog and PC-side Cartesian position control
- ODrive watchdog, velocity clamp, acceleration slew clamp, fault polling and clean mode exit
- Global Escape-key E-stop, Stop Motion and verified Resume After E-stop
- Moving-average, one-pole low-pass, Butterworth, median and raw velocity filters
- Always-visible live raw-turn and mapped-angle readout in IDLE, closed loop and after E-stop
- Raw/filtered velocity overlay, position and current telemetry plots
- QPainter linkage view with true annulus-intersection workspace overlay
- Axis calibration, verified software-offset synchronisation, startup/pre-calibrated flags and SPI encoder configuration
- Read-only connected-hardware snapshot so ODrive-resident settings are not confused with dashboard-only geometry/mapping
- Scrollable ODrive error-report window with decoded register values, refresh, copy and clear controls
- PID/current tuning, gain nudge buttons and rolling four-run step-response comparison
- Cartesian coordinate-sequence editor with JSON save/load, IK preflight, dwell, repeat and progress controls
- Built-in PID tuning guide tailored to the cascaded ODrive controller and five-bar mechanism
- JSON dashboard configuration persistence
- Built-in simulator for UI and workflow testing without hardware

## Installation

Use Python 3.10 or 3.11. The legacy ODrive 0.5.4 stack is not a good match for the newest Python releases. For the exact hardware stack, the requirements pin `odrive==0.5.4`.

```bash
python -m venv .venv

# Windows
.venv\Scripts\activate

# Linux
source .venv/bin/activate

pip install -r requirements.txt
```

## Run

Physical ODrive:

```bash
python run_dashboard.py
```

Simulator:

```bash
python run_dashboard.py --simulate
```

Optional configuration path:

```bash
python run_dashboard.py --simulate --config ./my_scara_config.json
```

The default dashboard config is created as `five_bar_dashboard_config.json` beside `run_dashboard.py`.

## First hardware bring-up

1. Mechanically support the arm and use conservative ODrive current/velocity limits.
2. Connect and confirm that the always-visible live encoder bar updates while both axes remain **IDLE**.
3. Open **Show Errors** and inspect the structured axis, motor and encoder registers.
4. Run **Calibrate Both**.
5. Enter the arm's known current physical joint angles, keep it stationary, and press **Sync Software Offsets**. This updates `offset_turns`, verifies the mapped angles, saves the JSON, and resets telemetry filtering without terminating the polling thread.
6. Enable closed loop and test very small raw nudges.
7. Test small firmware-planned joint moves.
8. Tune filters and PID only after raw telemetry is understood.
9. Mark calibrated and enable startup closed loop only after a clean calibration, with the arm physically supported on the first reboot.

## Important conversion note

The supplied specification's printed `turns_per_sec` equation conflicts dimensionally with its own position mapping. This code differentiates the stated position equation:

```text
turns = offset + direction * ((angle-home)/360) * gear_ratio
```

therefore:

```text
turns_per_second = direction * gear_ratio * degrees_per_second / 360
```

This preserves position/rate consistency and avoids an unsafe 360×/gear-ratio scaling error.

## Project layout

```text
five_bar_dashboard/
├── run_dashboard.py
├── requirements.txt
├── pyproject.toml
├── five_bar_dashboard/
│   ├── app.py
│   ├── backend.py
│   ├── config_store.py
│   ├── constants.py
│   ├── conversion.py
│   ├── filters.py
│   ├── kinematics.py
│   ├── main_window.py
│   ├── models.py
│   ├── sequence.py
│   ├── panels.py
│   ├── styles.py
│   ├── trajectory.py
│   ├── workers.py
│   └── widgets/
│       ├── error_dialog.py
│       ├── linkage_view.py
│       ├── live_position.py
│       └── telemetry_plots.py
├── PID_TUNING_GUIDE.md
├── TRAPEZOID_EDITOR_GUIDE.md
├── examples/coordinate_sequence_example.json
└── tests/
```

## Verification performed here

The package includes 32 unit tests covering kinematics, conversions, filters, trajectory synchronisation, config loading, IDLE-state encoder reads and structured error reporting. Hardware operations cannot be validated without the actual ODrive, encoders, motors and linkage, so perform staged low-energy commissioning.

## Dashboard configuration versus ODrive configuration

The **Config** tab contains two different categories:

- **Dashboard-only settings:** linkage geometry, gear ratio, direction, software encoder offset, home-angle convention, Cartesian limits, filters and display settings. The ODrive does not know these values, so they must be entered or calibrated in the dashboard JSON.
- **ODrive-resident settings:** SPI mode/CS pin, controller gains, velocity limit, current limit, calibration flags, startup flags, watchdog state and live encoder values. Use **Read Hardware Configuration** to read these from the connected controller.

Reading the hardware snapshot intentionally does not overwrite geometry, gear ratio or direction because those values cannot be inferred safely from the controller.


## Live position and error report behaviour in v1.2.0

- Encoder `pos_estimate` is polled without requiring `CLOSED_LOOP_CONTROL`; the live bar therefore continues to update in `IDLE` and after the software E-stop.
- The live bar shows raw turns, mapped joint angle, raw velocity, axis state and axis/motor/encoder error words for both axes.
- The telemetry worker tolerates up to five consecutive transient Fibre/libusb read failures before treating the connection as lost.
- **Show Errors** now opens a dedicated scrollable window even when all error values are zero. It supports refresh, copy and clear operations and falls back to direct register reads if `odrive.utils.dump_errors()` is unavailable.

## Encoder-noise handling in v1.3.0

The ODrive `encoder.vel_estimate` value can remain non-zero while an SPI absolute encoder is physically stationary. Version 1.3.0 no longer treats that single raw velocity value as proof of motor movement.

- **Software reference sync** samples both encoder positions for a configurable time window, unwraps possible one-turn discontinuities, rejects isolated SPI glitches, and uses the median position to calculate `offset_turns`.
- Stationary detection is based on sustained **position drift** between the first and final parts of the sample window. Raw `vel_estimate` is retained only as a diagnostic value.
- The live linkage and joint-angle display use a median-filtered position. A separate position-drift estimator reports `STATIONARY` or `MOVING` with a configurable deadband.
- The live bar displays raw position, stable position, position-derived drift, and raw velocity sensor output separately, so encoder noise is visible without being mistaken for physical motion.
- All relevant parameters are available under **Config → Encoder noise handling**.

Recommended starting values:

```text
position_median_window          7 samples
motion_window_s                 0.40 s
motion_deadband_turns_s         0.003 turns/s
sync_sample_duration_s          1.50 s
sync_sample_hz                  60 Hz
sync_max_drift_turns_s          0.003 turns/s
sync_noise_warning_span_turns   0.003 turns
sync_hard_span_turns            0.015 turns
```

Increase the motion deadband or sync drift limit only after confirming that the position is genuinely stationary. A high noise span should be investigated electrically: encoder supply decoupling, SPI wiring length, shielding/grounding, magnet alignment, and encoder mode/CS configuration can all contribute.


## Continuous closed-loop telemetry in v1.4.0

Version 1.4.0 removes the poller stop/restart cycle that previously occurred when the axes entered `CLOSED_LOOP_CONTROL` or resumed after E-stop.

- The dedicated `PollWorker` remains alive across `IDLE` → `CLOSED_LOOP_CONTROL` state transitions.
- Filter and position-motion estimator state is reset **inside the existing polling thread** through a thread-safe reset request.
- The Resume After E-stop action no longer pauses the state-independent poller.
- Bursts of Fibre/libusb read failures no longer terminate telemetry after six failures. The poller reports the interruption and keeps retrying until explicitly cancelled by disconnect or application shutdown.
- ODrive access remains serialised by `ODriveManager`, so state-change writes and telemetry reads cannot corrupt one another. A short gap while the state-change worker owns the USB lock is expected, but the polling worker itself does not stop.


## Coordinate sequences in v1.5.0

The **Coordinate Sequence** tab executes a list of Cartesian X/Y targets. Add or load points, set an optional dwell time, then press **Validate + Preview**. The entire list is checked with inverse kinematics before the first move, and numbered markers appear in the linkage view. This preflight checks geometric reachability only; it cannot detect collisions, cable limits, fixture interference, or whether the joint-space path between two valid endpoints is safe.

Press **Run Sequence** to execute each row. Each segment uses the ODrive firmware trapezoidal planner and waits for a stable position-derived settled state before continuing. The sequence can repeat, and **Stop Sequence**, **Stop Motion**, Escape and E-stop all cancel it and hold or idle the axes as appropriate. State-independent telemetry remains active during the sequence.

Example file: `examples/coordinate_sequence_example.json`. The example coordinates are only a format demonstration; validate them against your physical geometry and limits before enabling the motors.

## PID tuning guide in v1.5.0

Open the **PID Guide** tab for the integrated workflow, or read `PID_TUNING_GUIDE.md`. The recommended order is: establish conservative current/velocity limits, tune `vel_gain` with the integrator disabled, tune `pos_gain`, then add only enough `vel_integrator_gain` to remove repeatable load error. Use small step tests, change one value per run, compare the four-run overlay, and save to flash only after testing both directions and multiple linkage poses.

## Editable trapezoids in v1.6.0

The **Coordinate Sequence** tab now includes a trapezoid editor with independent joint-space limits for maximum velocity, acceleration and deceleration. The graph is a preview for the selected editor values and preview distance. It indicates whether the requested profile is triangular or has a constant-speed section.

Every waypoint row stores its own `Vmax`, `Accel` and `Decel` values. Edit the cells directly, or select rows and use **Apply Profile to Selected** / **Apply Profile to All**. Press **Load Selected Profile** to copy a row back into the editor. New rows use the profile currently shown in the editor.

The profile values are requests, not a way to bypass safety limits. During execution:

1. joint-space limits are converted to turns-based ODrive units per axis;
2. velocity is capped at 95% of each physical `controller.config.vel_limit`;
3. both axes are time-synchronised by reducing V/A/D on the faster axis;
4. independent values are written to `trap_traj.config.vel_limit`, `accel_limit` and `decel_limit`;
5. each target is written once through `controller.input_pos`.

Use **Set as Global Move Defaults** to apply the editor values to Joint Control and IK moves. See `TRAPEZOID_EDITOR_GUIDE.md` for the exact execution pipeline.

Saved sequence files now use `five-bar-cartesian-sequence-v2`. Version-1 files still load and receive the editor's current V/A/D defaults.


## Coordinate sequence UI and feedforward in v1.7.0

The Coordinate Sequence tab is now divided into **Waypoints**, **Motion & Feedforward**, and **Validate & Run** pages. This removes the previous vertically crowded layout, gives the table horizontal scrolling, and keeps the editor reachable through a scroll area on smaller displays.

Each point now stores independent axis0/axis1 velocity feedforward and torque feedforward in addition to its V/A/D trapezoid. Feedforward is disarmed by default, and loading a sequence never arms it automatically. Velocity feedforward is removed before settling; torque feedforward can remain through dwell and is cleared on completion, stop or fault. See `SEQUENCE_FEEDFORWARD_GUIDE.md`.

Saved files use `five-bar-cartesian-sequence-v3`; v1/v2 files remain compatible and receive zero feedforward defaults.
