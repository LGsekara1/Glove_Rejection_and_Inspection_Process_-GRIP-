    from nicegui import ui
import odrive
import sys
import time

# --- Wrap the entire execution block to prevent double-running ---
if __name__ in {"__main__", "__mp_main__"}:
    
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
    my_drive.axis0.controller.config.pos_gain = 40.0
    my_drive.axis0.controller.config.vel_gain = 0.2
    my_drive.axis0.controller.config.vel_limit = 5

    # 2. Calibration with safety wait
    # print("Calibrating...")
    # my_drive.axis0.requested_state = 3
    # time.sleep(20)

    # 3. Enter Closed Loop
    print("Engaging...")
    my_drive.axis0.requested_state = 8
    time.sleep(10)

    if check_errors(my_drive.axis0):
        sys.exit()

    # 4. Move to a FIXED position once
    home_pos = my_drive.axis0.encoder.pos_estimate
    target = home_pos - 1
    print(f"Moving to {target}...")
    my_drive.axis0.controller.input_pos = target

    # --- GUI Section ---
    
    def set_position(e):
        """Sends the new slider value to the ODrive."""
        my_drive.axis0.controller.input_pos = e.value

    def monitor_status():
        """Runs every 0.5 seconds to check errors and update the position display."""
        if check_errors(my_drive.axis0):
            ui.notify('ODrive Error! Check terminal.', color='negative')
            my_drive.axis0.requested_state = 1 # Set to idle on error
        else:
            current_pos_label.set_text(f"Current Pos: {my_drive.axis0.encoder.pos_estimate:.2f}")

    # UI Layout
    with ui.card().classes('w-96 mx-auto mt-10 p-5'):
        ui.label('Motor Position Control').classes('text-xl font-bold mb-4')
        
        ui.label('Target Position:')
        slider = ui.slider(min=home_pos - 1, max=home_pos + 1, value=target, step=0.02, on_change=set_position)
        
        current_pos_label = ui.label(f"Current Pos: {home_pos:.2f}").classes('mt-4 text-lg')

    ui.timer(0.5, monitor_status)

    # CRITICAL FIX: Turn off reload so the server doesn't restart the script
    ui.run(reload=False)