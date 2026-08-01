# Coordinate Sequence Feedforward Guide

## New layout

Open **Coordinate Sequence** and use the three internal pages in order:

1. **Waypoints** to add, reorder, duplicate, delete, load and save coordinates.
2. **Motion & Feedforward** to edit the selected row's trapezoid and feedforward values.
3. **Validate & Run** to validate, arm feedforward and execute the sequence.

The waypoint table scrolls horizontally. This avoids compressing twelve editable/status columns until their labels and values are clipped.

## Per-point fields

Each waypoint contains:

- Cartesian target `X`, `Y`
- dwell time
- `Vmax`, acceleration and deceleration
- axis0/axis1 velocity feedforward in turns/s
- axis0/axis1 torque feedforward in Nm

Use **Load Selected Row** to copy one row into the editor. The trapezoid and feedforward can then be applied independently to selected rows or to every row.

## Feedforward timing

For each point:

1. the requested trapezoid is converted and synchronized;
2. the selected velocity and torque feedforward values are written;
3. the target position is written once to the ODrive firmware trap planner;
4. velocity feedforward is cleared when the planned moving phase ends;
5. torque feedforward remains through the dwell;
6. the next point replaces it, or sequence completion/stop/fault clears it.

This is a constant per-segment additive feedforward, not a continuously calculated inverse-dynamics model. A future black-box/grey-box model can populate these fields or replace them with time-varying commands on the STM32 real-time controller.

## Arming and safe commissioning

Stored feedforward is ignored unless **Arm per-waypoint feedforward** is checked on the Validate & Run page. Loading JSON always leaves this unchecked.

Commission in this order:

1. Run the same sequence with feedforward disabled and confirm stable PID tracking.
2. Start with torque and velocity feedforward equal to zero.
3. Change only one axis and one feedforward type at a time.
4. Use conservative current, velocity, acceleration and workspace limits.
5. Increase in very small steps while recording position error, current, overshoot and settling.
6. Test both directions and multiple linkage poses.

Velocity feedforward can create final-position bias if it remains active, which is why the dashboard removes it before settling. Torque feedforward can cause immediate motion or sustained force, so it is bounded using the ODrive current limit and torque constant where possible.

## JSON v3 example fields

```json
{
  "x_mm": 0.0,
  "y_mm": 450.0,
  "dwell_s": 0.25,
  "max_vel_deg_s": 35.0,
  "max_accel_deg_s2": 80.0,
  "max_decel_deg_s2": 60.0,
  "velocity_ff0_turns_s": 0.0,
  "velocity_ff1_turns_s": 0.0,
  "torque_ff0_nm": 0.0,
  "torque_ff1_nm": 0.0
}
```
