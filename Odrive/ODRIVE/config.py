import odrive
odrv0 = odrive.find_any()

print("Connected")
odrv0 = dev0
# ===== 0. Connect / alias =====
# ===== 3. Power supply config (24V, 30A PSU + cap bank) =====
odrv0.config.dc_bus_overvoltage_trip_level = 26
odrv0.config.dc_bus_undervoltage_trip_level = 10.5
odrv0.config.dc_max_positive_current = 60
odrv0.config.dc_max_negative_current = -1      # raise slightly if regen with cap bank needs headroom
odrv0.config.brake_resistance = 2.0

# ===== 4. Motor config — axis0 (D5065, 270KV) =====
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.torque_constant = 8.27 / 270
odrv0.axis0.motor.config.current_lim = 30
odrv0.axis0.motor.config.calibration_current = 15
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4

# ===== 5. Motor config — axis1 (D5065, 270KV) =====
odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis1.motor.config.pole_pairs = 7
odrv0.axis1.motor.config.torque_constant = 8.27 / 270
odrv0.axis1.motor.config.current_lim = 30
odrv0.axis1.motor.config.calibration_current = 15
odrv0.axis1.motor.config.resistance_calib_max_voltage = 4

# ===== 6. Encoder config — axis0 on GPIO4 =====

odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.cpr = 4000
odrv0.axis0.encoder.config.use_index = False

odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.cpr = 4000
odrv0.axis1.encoder.config.use_index = False
# ===== 8. Save and reboot (required for GPIO/UART/I2C changes) =====
odrv0.save_configuration()
# Reconnect after reboot, then:
odrv0 = dev0

# =odrv0.axis0.encoder.pos_estimate ==== 9. Verify both encoders are alive BEFORE calibrating =====
  # hand-turn axis0 shaft, confirm smooth tracking
odrv0.axis0.encoder.pos_estimate 
odrv0.axis1.encoder.pos_estimate   # hand-turn axis1 shaft, confirm smooth tracking
dump_errors(odrv0)                 # must be clean before proceeding

# ===== 10. Motor calibration — both axes =====
odrv0.axis0.requested_state = 3
# wait for beep
odrv0.axis1.requested_state = 3
# wait for beep
dump_errors(odrv0)

# ===== 11. Encoder offset calibration — both axes =====
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
dump_errors(odrv0)

# ===== 12. Persist calibration for auto-start =====
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.config.startup_closed_loop_control = True

odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.axis1.config.startup_closed_loop_control = True

odrv0.save_configuration()
# Reconnect after reboot, then:
odrv0 = dev0

# ===== 13. Closed loop test =====
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.controller.input_vel = 1
odrv0.axis1.controller.input_vel = 1