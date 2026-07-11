import sys
import asyncio
from nicegui import ui
import odrive
from odrive.enums import *

# --- Wrap the entire execution block to prevent double-running ---
if __name__ in {"__main__", "__mp_main__"}:

    def check_errors(axis, name="Axis"):
        """Checks specific axis for errors and logs them to the terminal."""
        if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0 or axis.controller.error != 0:
            print(f"\n[!] ERROR: {name} {hex(axis.error)} | Motor {hex(axis.motor.error)} | Encoder {hex(axis.encoder.error)}")
            return True
        return False

    print("Connecting to ODrive...")
    my_drive = odrive.find_any()
    print("Connected.")

    # ===== 1. Power Supply Config (24V, 30A PSU + Cap Bank) =====
    my_drive.clear_errors()
    my_drive.config.dc_bus_overvoltage_trip_level = 26.0
    my_drive.config.dc_bus_undervoltage_trip_level = 10.5
    my_drive.config.dc_max_positive_current = 60.0
    my_drive.config.dc_max_negative_current = -1.0  
    my_drive.config.brake_resistance = 2.0

    # ===== 2. Motor & Encoder Config — Axis 0 (D5065, 270KV) =====
    my_drive.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    my_drive.axis0.motor.config.pole_pairs = 7
    my_drive.axis0.motor.config.torque_constant = 8.27 / 270
    my_drive.axis0.motor.config.current_lim = 30.0
    my_drive.axis0.motor.config.calibration_current = 15.0
    my_drive.axis0.motor.config.resistance_calib_max_voltage = 4.0
    my_drive.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
    my_drive.axis0.encoder.config.cpr = 4000
    my_drive.axis0.encoder.config.use_index = False

    # ===== 3. Motor & Encoder Config — Axis 1 (D5065, 270KV) =====
    my_drive.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    my_drive.axis1.motor.config.pole_pairs = 7
    my_drive.axis1.motor.config.torque_constant = 8.27 / 270
    my_drive.axis1.motor.config.current_lim = 30.0
    my_drive.axis1.motor.config.calibration_current = 15.0
    my_drive.axis1.motor.config.resistance_calib_max_voltage = 4.0
    my_drive.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL
    my_drive.axis1.encoder.config.cpr = 4000
    my_drive.axis1.encoder.config.use_index = False

    # ===== 4. Default Controller Tuning Config =====
    for axis in [my_drive.axis0, my_drive.axis1]:
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.pos_gain = 40.0
        axis.controller.config.vel_gain = 0.2
        axis.controller.config.vel_limit = 5.0

    # Establish initial reference positions
    home_pos_0 = my_drive.axis0.encoder.pos_estimate
    home_pos_1 = my_drive.axis1.encoder.pos_estimate

    # --- GUI Control Functions ---

    def clear_all_errors():
        """Clears hardware errors across the ODrive board."""
        my_drive.clear_errors()
        ui.notify("Errors cleared on ODrive", color='info')

    def engage_closed_loop(axis, name):
        """Switches the selected axis into closed loop control mode."""
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        ui.notify(f"{name} entered Closed Loop Control", color='success')

    async def run_calibration_sequence(axis, name):
        """Asynchronously runs motor and encoder calibration without freezing the UI."""
        try:
            ui.notify(f"Starting Motor Calibration for {name}...", color='warning')
            axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            while axis.current_state != AXIS_STATE_IDLE:
                await asyncio.sleep(0.2)
            
            if check_errors(axis, name):
                ui.notify(f"{name} Motor Calibration Failed!", color='negative')
                return

            ui.notify(f"Starting Encoder Calibration for {name}...", color='warning')
            axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            while axis.current_state != AXIS_STATE_IDLE:
                await asyncio.sleep(0.2)

            if check_errors(axis, name):
                ui.notify(f"{name} Encoder Calibration Failed!", color='negative')
                return

            ui.notify(f"{name} Calibration Completed Successfully!", color='success')
        except Exception as e:
            ui.notify(f"Calibration Error: {str(e)}", color='negative')

    def update_tuning(axis, parameter, value):
        """Updates controller configurations live."""
        setattr(axis.controller.config, parameter, value)

    def monitor_status():
        """Cyclic check for tracking errors and rendering live positions."""
        err_0 = check_errors(my_drive.axis0, "Axis 0")
        err_1 = check_errors(my_drive.axis1, "Axis 1")
        
        if err_0 or err_1:
            ui.notify('ODrive Error Detected! Check system terminal.', color='negative')
            if err_0: my_drive.axis0.requested_state = AXIS_STATE_IDLE
            if err_1: my_drive.axis1.requested_state = AXIS_STATE_IDLE
        
        current_pos_label_0.set_text(f"Current Pos: {my_drive.axis0.encoder.pos_estimate:.2f}")
        current_pos_label_1.set_text(f"Current Pos: {my_drive.axis1.encoder.pos_estimate:.2f}")

    # --- UI Layout Design ---

    with ui.row().classes('w-full justify-center gap-6 mt-6'):
        ui.button('Global Clear Errors', on_click=clear_all_errors, color='blue').classes('px-6 text-lg')

    with ui.row().classes('w-full max-w-5xl mx-auto justify-center gap-6 mt-4'):
        
        # --- AXIS 0 CONTROL CARD ---
        with ui.card().classes('w-96 p-5 border shadow-md'):
            ui.label('Axis 0 Controller').classes('text-xl font-bold border-b pb-2 mb-3 text-blue-600')
            
            current_pos_label_0 = ui.label(f"Current Pos: {home_pos_0:.2f}").classes('text-lg font-semibold mb-2')
            
            ui.label('Target Position Slider:')
            slider_0 = ui.slider(min=home_pos_0 - 1, max=home_pos_0 + 1, value=home_pos_0, step=0.005, 
                                 on_change=lambda e: setattr(my_drive.axis0.controller, 'input_pos', e.value))
            
            ui.label('Tuning & Limits:').classes('font-bold mt-4 mb-1 text-sm text-gray-500')
            ui.number('Position Gain', value=40.0, format='%.2f', on_change=lambda e: update_tuning(my_drive.axis0, 'pos_gain', e.value)).classes('w-full density-compact')
            ui.number('Velocity Gain', value=0.2, format='%.3f', on_change=lambda e: update_tuning(my_drive.axis0, 'vel_gain', e.value)).classes('w-full density-compact')
            ui.number('Velocity Limit', value=5.0, format='%.1f', on_change=lambda e: update_tuning(my_drive.axis0, 'vel_limit', e.value)).classes('w-full density-compact')
            
            ui.label('Actions:').classes('font-bold mt-4 mb-1 text-sm text-gray-500')
            ui.button('Run Full Calibration', on_click=lambda: run_calibration_sequence(my_drive.axis0, "Axis 0"), color='orange').classes('w-full mb-2')
            ui.button('Enter Closed Loop Mode', on_click=lambda: engage_closed_loop(my_drive.axis0, "Axis 0"), color='green').classes('w-full')

        # --- AXIS 1 CONTROL CARD ---
        with ui.card().classes('w-96 p-5 border shadow-md'):
            ui.label('Axis 1 Controller').classes('text-xl font-bold border-b pb-2 mb-3 text-purple-600')
            
            current_pos_label_1 = ui.label(f"Current Pos: {home_pos_1:.2f}").classes('text-lg font-semibold mb-2')
            
            ui.label('Target Position Slider:')
            slider_1 = ui.slider(min=home_pos_0 - 1, max=home_pos_0 + 1, value=home_pos_0, step=0.005, 
                                 on_change=lambda e: setattr(my_drive.axis1.controller, 'input_pos', e.value))
            
            ui.label('Tuning & Limits:').classes('font-bold mt-4 mb-1 text-sm text-gray-500')
            ui.number('Position Gain', value=40.0, format='%.2f', on_change=lambda e: update_tuning(my_drive.axis1, 'pos_gain', e.value)).classes('w-full density-compact')
            ui.number('Velocity Gain', value=0.2, format='%.3f', on_change=lambda e: update_tuning(my_drive.axis1, 'vel_gain', e.value)).classes('w-full density-compact')
            ui.number('Velocity Limit', value=5.0, format='%.1f', on_change=lambda e: update_tuning(my_drive.axis1, 'vel_limit', e.value)).classes('w-full density-compact')
            
            ui.label('Actions:').classes('font-bold mt-4 mb-1 text-sm text-gray-500')
            ui.button('Run Full Calibration', on_click=lambda: run_calibration_sequence(my_drive.axis1, "Axis 1"), color='orange').classes('w-full mb-2')
            ui.button('Enter Closed Loop Mode', on_click=lambda: engage_closed_loop(my_drive.axis1, "Axis 1"), color='green').classes('w-full')

    ui.timer(0.5, monitor_status)

    # Disable autoreload so server background threads don't loop-initialize hardware
    ui.run(reload=False)