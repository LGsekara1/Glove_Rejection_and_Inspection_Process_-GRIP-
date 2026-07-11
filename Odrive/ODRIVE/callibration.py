import odrive
import time

odrv0 = odrive.find_any()

MOTOR_TYPE_HIGH_CURRENT = 0
ENCODER_MODE_INCREMENTAL = 0
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_HOMING = 11
AXIS_STATE_IDLE = 1

print("Connected")

odrv0.clear_errors()

odrv0.axis0.encoder.config.calib_scan_distance = 6.283185*2
odrv0.axis1.encoder.config.calib_scan_distance = 6.283185*2

odrv0.config.dc_bus_overvoltage_trip_level = 26
odrv0.config.dc_bus_undervoltage_trip_level = 10.5
odrv0.config.dc_max_positive_current = 60
odrv0.config.dc_max_negative_current = -1
odrv0.config.brake_resistance = 2.0


# Motor config axis0
for axis in [odrv0.axis0, odrv0.axis1]:
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    axis.motor.config.pole_pairs = 7
    axis.motor.config.torque_constant = 8.27 / 270
    axis.motor.config.current_lim = 30
    axis.motor.config.calibration_current = 15
    axis.motor.config.resistance_calib_max_voltage = 4


# Encoder config
for axis in [odrv0.axis0, odrv0.axis1]:
    axis.encoder.config.mode = ENCODER_MODE_INCREMENTAL
    axis.encoder.config.cpr = 4000
    axis.encoder.config.use_index = False




def wait_for_idle(axis, name):
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    print(name, "finished")


# Motor calibration
print("Motor calibration")

odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION


wait_for_idle(odrv0.axis0, "Axis0 motor calibration")
wait_for_idle(odrv0.axis1, "Axis1 motor calibration")


# Encoder calibration
print("Encoder offset calibration")

odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION


wait_for_idle(odrv0.axis0, "Axis0 encoder calibration")
wait_for_idle(odrv0.axis1, "Axis1 encoder calibration")


# Set calibrated flags
for axis in [odrv0.axis0, odrv0.axis1]:
    axis.motor.config.pre_calibrated = True
    axis.encoder.config.pre_calibrated = True

odrv0.config.gpio5_mode = 0
odrv0.axis0.min_endstop.config.gpio_num = 5
odrv0.axis0.min_endstop.config.is_active_high = True
odrv0.axis0.min_endstop.config.offset = -0.08
odrv0.axis0.min_endstop.config.enabled = True
odrv0.config.gpio5_mode = 1
odrv0.axis0.controller.config.homing_speed = -0.25

odrv0.config.gpio6_mode = 0
odrv0.axis1.min_endstop.config.gpio_num = 6
odrv0.axis1.min_endstop.config.is_active_high = True
odrv0.axis1.min_endstop.config.offset = 0.08
odrv0.axis1.min_endstop.config.enabled = True
odrv0.config.gpio6_mode = 1
odrv0.axis1.controller.config.homing_speed = 0.25

odrv0.axis0.requested_state = AXIS_STATE_HOMING
odrv0.axis1.requested_state = AXIS_STATE_HOMING

print("Axis0 errors:")
print(odrv0.axis0.error)
print(odrv0.axis0.motor.error)
print(odrv0.axis0.encoder.error)

print("Axis1 errors:")
print(odrv0.axis1.error)
print(odrv0.axis1.motor.error)
print(odrv0.axis1.encoder.error)  


# print("Saving final calibration...")
odrv0.save_configuration()
# print("Saving final calibration...")
# time.sleep(20)

print("Calibration complete")