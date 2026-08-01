# Editable Trapezoid Guide

## What can be edited

The dashboard exposes three independent joint-space limits:

- **Vmax**: maximum joint velocity in degrees per second
- **Accel**: maximum positive acceleration in degrees per second squared
- **Decel**: maximum braking/deceleration in degrees per second squared

These values define the requested ODrive firmware trap trajectory. Short moves may become triangular because there is not enough distance to reach Vmax.

## Coordinate Sequence tab

1. Enter Vmax, Accel and Decel in the trapezoid editor.
2. Change **Preview joint distance** to inspect how the same settings behave on a short or long joint move.
3. Add a coordinate. New rows inherit the editor values.
4. Select one or more rows and press **Apply Profile to Selected**, or use **Apply Profile to All**.
5. Values can also be typed directly into each row's Vmax, Accel and Decel cells.
6. Press **Validate + Preview**, then **Run Sequence**.

Use **Load Selected Profile** to copy a row's values into the editor. Use **Set as Global Move Defaults** to make the same V/A/D values the defaults for Joint Control and IK point-to-point moves.

## What is actually sent to ODrive

For every waypoint, the dashboard:

1. converts V/A/D from joint degrees to turns for each axis using that axis's gear ratio;
2. clamps velocity to 95% of the physical ODrive `controller.config.vel_limit`;
3. computes each axis's asymmetric trapezoid or triangle;
4. reduces V/A/D on the faster axis until both axes have the same planned duration;
5. writes `trap_traj.config.vel_limit`, `accel_limit`, and `decel_limit`;
6. writes the target once to `controller.input_pos`.

The graph shows the requested profile before axis-specific firmware clamping and synchronisation. The system log records the requested profile, planned duration, and applied turns-based values.

## Safe first values

Start below the final operating speed. Use a modest Vmax, and keep acceleration and deceleration low enough that the mechanism does not jerk, flex, slip, or draw excessive current. Increase one parameter at a time while observing current, vibration, overshoot, encoder stability, and settling.

A valid endpoint and a valid trapezoid do not prove that the joint-space path is collision-free. Test at reduced current and speed before running the complete sequence.
