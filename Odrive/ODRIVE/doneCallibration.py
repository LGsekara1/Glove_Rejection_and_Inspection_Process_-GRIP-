import odrive
import time
AXIS_STATE_HOMING = 11
AXIS_STATE_IDLE = 1

odrv0 = odrive.find_any()

print("Connected")

odrv0.clear_errors()

odrv0.axis0.requested_state = 7


while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

odrv0.axis1.requested_state = 7

while odrv0.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

# odrv0.axis0.requested_state = 8
# odrv0.axis1.requested_state = 8

odrv0.axis0.requested_state = AXIS_STATE_HOMING
odrv0.axis1.requested_state = AXIS_STATE_HOMING

print("HOMING")

print("Axis0 errors:")
print(odrv0.axis0.error)
print(odrv0.axis0.motor.error)
print(odrv0.axis0.encoder.error)

print("Axis1 errors:")
print(odrv0.axis1.error)
print(odrv0.axis1.motor.error)
print(odrv0.axis1.encoder.error) 