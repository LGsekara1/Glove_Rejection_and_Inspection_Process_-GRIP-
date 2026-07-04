import odrive
import time

def check_errors(axis):
    if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0 or axis.controller.error != 0:
        print(f"\n[!] ERROR: Axis {hex(axis.error)} | Motor {hex(axis.motor.error)} | Encoder {hex(axis.encoder.error)}")
        return True
    return False

print("Connecting...")
my_drive = odrive.find_any()

# 1. Setup
my_drive.clear_errors()
my_drive.axis0.controller.config.control_mode = 3
my_drive.axis0.controller.config.pos_gain = 20.0 # Increased slightly for stiffness
my_drive.axis0.controller.config.vel_gain = 0.1
my_drive.axis0.controller.config.vel_limit = 10

# 2. Calibration with safety wait
print("Calibrating...")
my_drive.axis0.requested_state = 3

time.sleep(20)

# 3. Enter Closed Loop
print("Engaging...")
my_drive.axis0.requested_state = 8
time.sleep(1)

if check_errors(my_drive.axis0):
    exit()

# 4. Move to a FIXED position once
home_pos = my_drive.axis0.encoder.pos_estimate
target = home_pos - 1
print(f"Moving to {target}...")
my_drive.axis0.controller.input_pos = target


# 5. Monitor
try:
    while True:
        if check_errors(my_drive.axis0):
            break
        print(f"Current Pos: {my_drive.axis0.encoder.pos_estimate:.2f}", end='\r')
        time.sleep(0.5)
except KeyboardInterrupt:
    my_drive.axis0.requested_state = 1