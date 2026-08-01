# PID Tuning Guide for the ODrive Five-Bar SCARA

This dashboard exposes ODrive's cascaded position/velocity controller values:

- `pos_gain`
- `vel_gain`
- `vel_integrator_gain`
- `vel_limit`
- `current_lim`

There is no universally correct numeric gain set. Motor torque constant, encoder quality, gear ratio, inertia, linkage pose, friction and payload all change the result. Tune from measurements, not from copied values.

## 1. Safety and mechanical preparation

1. Keep a physical power disconnect within reach. The GUI E-stop is not a safety-rated circuit.
2. Start with the end effector unloaded and the arm supported so an unexpected motion cannot strike the frame.
3. Check link fasteners, bearings, gearbox backlash, encoder magnet alignment, grounding and SPI wiring first. PID cannot repair mechanical looseness or encoder corruption.
4. Confirm the dashboard's direction, gear ratio and software zero using a small manually measured rotation.
5. Use conservative `current_lim`, `vel_limit`, trajectory velocity and acceleration.
6. Read the existing values and record them before changing anything.
7. Apply changes **in RAM** while tuning. Do not save to flash until the response has been validated repeatedly.

For the cleanest result, tune each motor/gearbox with the closed-chain distal linkage safely disconnected or supported, if the mechanism permits it. Afterward, validate the fully assembled five-bar because the coupled load changes strongly with pose.

## 2. Understand the loops

ODrive uses cascaded control:

1. The outer position loop converts position error into a requested velocity. `pos_gain` determines how aggressively it does this.
2. The inner velocity loop converts velocity error into motor effort. `vel_gain` is its proportional term.
3. `vel_integrator_gain` accumulates persistent velocity error and removes steady error under load.

Because the velocity loop is inside the position loop, tune the velocity behavior before making the position loop aggressive.

## 3. Establish a conservative baseline

- Set `vel_integrator_gain = 0` initially.
- Keep `pos_gain` low enough that a small step produces gentle motion.
- Use a low `current_lim` that can move the unloaded axis but cannot produce dangerous torque.
- Use a low `vel_limit` during the first tests.
- In the Step-response panel, start with a `0.01` to `0.02` turn step, even though the GUI default is `0.05` turns.
- Use a sample duration around `1.5` to `3.0` seconds.

Do not begin by increasing all three gains together. Change one value between runs.

## 4. Tune `vel_gain`

1. Keep the integrator at zero and position gain conservative.
2. Run a small positive step.
3. Increase `vel_gain` with the **+25%** button.
4. Repeat until the response becomes firm and follows the command promptly.
5. At the first sign of high-frequency buzz, current chatter or rapid oscillation, reduce `vel_gain` by at least **20%**.
6. Repeat with a negative step.

A high `vel_gain` converts encoder velocity noise into current commands. If the raw velocity remains noisy while the shaft is stationary, solve the encoder/magnet/wiring issue before making this gain aggressive. Filtering the GUI trace does not remove noise from the ODrive's internal controller.

## 5. Tune `pos_gain`

1. Keep the chosen stable `vel_gain`.
2. Increase `pos_gain` in small steps.
3. Look for faster settling and improved stiffness.
4. Stop increasing when overshoot or repeated position oscillation grows.
5. Reduce `pos_gain` by 20% after the first clearly excessive response.

Too low: slow and compliant.  
Too high: overshoot, oscillation and large velocity demands.

## 6. Add `vel_integrator_gain` last

Only add the integrator after the proportional gains are stable.

1. Apply a realistic payload or disturbance.
2. Observe whether a repeatable final error remains.
3. Increase `vel_integrator_gain` slightly.
4. Stop when the persistent error is removed.
5. Reduce it if you see slow oscillation, delayed overshoot, drift after saturation or long recovery.

Do not use the integrator to hide backlash, an incorrect zero reference or encoder noise.

## 7. Read the step-response results

The dashboard reports:

- **Overshoot %**: peak travel beyond the target, relative to step size.
- **2% settling time**: the last time the response was outside a ±2% band around the target.
- **Final error**: final sampled position minus target.
- **Four-run overlay**: compares the latest gain changes directly.

Interpretation:

| Observation | Likely response |
|---|---|
| Slow, no overshoot, low current | Raise `vel_gain` if velocity tracking is soft, then raise `pos_gain`. |
| Fast overshoot after a `pos_gain` increase | Reduce `pos_gain`; verify the velocity loop has adequate damping. |
| High-frequency buzz at rest | Reduce `vel_gain`; inspect encoder noise and mechanical resonance. |
| Persistent final error under load | Add a small integrator only after P gains are stable. |
| Slow oscillation or delayed overshoot | Reduce `vel_integrator_gain`; check saturation. |
| Current limit reached | Reduce step, load or acceleration. Do not raise gains to fight saturation. |
| Good in one pose, poor in another | Five-bar inertia and leverage vary with pose; tune for the worst intended pose. |

Do not judge the tune only from a heavily filtered velocity line. Overlay raw and filtered velocity because a low cutoff can hide oscillation and delay.

## 8. Validate the assembled robot

After tuning each axis:

1. Reassemble the complete closed-chain linkage.
2. Test small positive and negative moves at the center of the workspace.
3. Test several poses, including the highest-load and lowest-manipulability regions you intend to use.
4. Test with the expected end-effector payload.
5. Check motor and controller temperature.
6. Confirm no sustained current, buzz or drift while holding position.
7. Increase speed and acceleration gradually, not simultaneously.
8. Re-run step tests after any mechanical or payload change.

## 9. Save to flash only after validation

Use **Save Config to Flash** only when:

- both directions are stable,
- at least four repeatable step tests have been recorded,
- the arm is stable across the intended workspace,
- currents and temperatures are acceptable,
- the raw encoder signal is credible.

Saving reboots the ODrive and drops the USB connection. Wait for reboot, reconnect, read the values back, and run a final low-energy test.
