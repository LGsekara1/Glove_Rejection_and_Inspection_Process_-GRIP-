import odrive
import time
import math

# 1. Connect to the ODrive
print("Finding ODrive...")
my_drive = odrive.find_any()

# 2. Configure for Position Control
# Ensure these are already set via odrivetool or set them here
my_drive.axis0.controller.config.input_mode = 1  # INPUT_MODE_POS_FILTER (smoother)
my_drive.axis0.requested_state = 8               # AXIS_STATE_CLOSED_LOOP_CONTROL

# 3. Simple Position Loop
print("Moving to position...")
my_drive.axis0.controller.input_pos = 10000      # Set target position
time.sleep(2)
my_drive.axis0.controller.input_pos = 0          # Return to start

# 4. Stop
my_drive.axis0.requested_state = 1               # AXIS_STATE_IDLE