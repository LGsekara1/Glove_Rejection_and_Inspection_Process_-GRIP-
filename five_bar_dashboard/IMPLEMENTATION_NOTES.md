# Implementation Notes

This codebase follows the supplied rewrite specification as the functional baseline.

## Deliberate safety correction

The specification's printed velocity conversion conflicts with its own position conversion. The implementation uses the derivative of the stated position mapping:

```text
turns_per_second = direction × gear_ratio × degrees_per_second / 360
```

Using the printed rate expression would create a severe scaling error, so it was not reproduced.

## Hardware validation boundary

The kinematics, conversion, filter, trajectory synchronisation, configuration merge and simulator-backed worker flows were tested in this environment. The following still require staged validation on the actual machine:

- ODrive v3.6 firmware 0.5.6 USB behaviour
- SPI absolute encoder modes and CS GPIO wiring
- Motor directions, gear ratios and software-zero reference
- Safe current, velocity and acceleration limits
- Five-bar FK branch/elbow configuration for the physical assembly
- Watchdog and E-stop response under real USB contention
- PID step response and mechanism resonance

Start with the arm mechanically supported, conservative current limits and small raw-turn nudges.

## 2026-07-29 hardware-sync correction

- Fixed software sync semantics: it now computes `offset_turns` from known physical angles instead of changing `home_angle_deg`.
- Fixed the stale telemetry bug: the poll worker held a deep copy of the old mapping and previously resumed after sync, making the GUI appear unsynchronised. Telemetry is now restarted after a successful sync.
- Sync refuses to run while either encoder velocity exceeds 0.02 turns/s and verifies the resulting mapped angles.
- Added a read-only ODrive hardware snapshot and clearer separation between ODrive-resident settings and dashboard-only robot geometry/mapping.

## 2026-07-29 IDLE telemetry and error-dialog correction

- Added an always-visible raw encoder and mapped joint-position readout that is independent of `current_state`.
- Kept short register actions concurrent with polling while retaining lock-serialised ODrive USB access.
- Added retry tolerance for transient telemetry read failures.
- Replaced the static error message box with a structured, scrollable report dialog and direct-register fallback.


## v1.5 coordinate-sequence design

The coordinate list is intentionally a sequence of discrete point-to-point moves. Each waypoint is solved with IK before motion begins, then both axes are commanded once through the ODrive firmware `TRAP_TRAJ` planner. No PC-side interpolation or streamed joint trajectory was added.

The worker uses the existing global ODrive lock for each short read/write while the independent telemetry poller remains alive. Waypoint completion uses median-filtered position and position-derived drift rather than noisy raw `encoder.vel_estimate`.

Preflight validation proves only that every endpoint is kinematically reachable for the selected elbow branches. It does not prove collision-free travel, avoid singularities between endpoints, enforce unmodeled cable/joint limits, or guarantee a straight Cartesian path. These remain physical commissioning responsibilities.
