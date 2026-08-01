# v1.7.0 - Compact Coordinate Sequence UI + Per-Point Feedforward

## Coordinate Sequence UI cleanup

The previous sequence page placed the table, trapezoid graph, row controls, file controls, run controls, progress and status in one vertical layout. On smaller windows this caused clipping and made the workflow difficult to follow.

The tab is now divided into three internal pages:

1. **Waypoints**: coordinate table, row editing and JSON load/save.
2. **Motion & Feedforward**: trapezoid plot/editor and per-axis feedforward editor.
3. **Validate & Run**: repeat count, feedforward arming, validation, run/stop, progress and status.

The waypoint table has explicit horizontal scrolling, interactive column widths and a larger state column. The profile page is inside a scroll area, so controls remain reachable at reduced window heights. The main left control pane is also slightly wider by default.

## Per-waypoint feedforward

Every waypoint can now store four additive feedforward values:

- `velocity_ff0_turns_s`
- `velocity_ff1_turns_s`
- `torque_ff0_nm`
- `torque_ff1_nm`

The editor supports loading one row, applying only the trapezoid to selected/all rows, and independently applying feedforward to selected/all rows. Values are saved in sequence JSON format v3. Version-1 and version-2 files remain loadable and default all feedforward fields to zero.

## Safety behaviour

- Sequence feedforward is **disarmed by default**.
- Loading a file never silently arms feedforward, even if the file says it was previously enabled.
- Non-zero values require a confirmation before execution.
- Velocity feedforward is clamped to 95% of the axis firmware velocity limit and automatically reset to zero before waypoint settling.
- Torque feedforward is clamped to 80% of `current_lim × torque_constant` when those legacy ODrive fields are available.
- If the firmware does not expose the required feedforward or torque-constant fields, a non-zero request is rejected instead of being applied without a safety bound.
- Torque feedforward remains active through the waypoint dwell, then changes with the next waypoint. All feedforward is reset on sequence completion, stop or fault.

## Validation completed

- All Python files compile.
- 11 sequence-format/validation tests pass.
- A simulator-backed worker smoke test confirms feedforward writing, velocity clamping, torque clamping and zero reset.
- The full Qt window still requires testing on a machine with PySide6 installed.
