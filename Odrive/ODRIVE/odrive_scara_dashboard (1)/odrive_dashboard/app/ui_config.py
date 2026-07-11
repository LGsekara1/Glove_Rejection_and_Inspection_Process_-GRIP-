"""ui_config.py -- initial motor / encoder / board configuration."""
from nicegui import ui, run

from state import state
import odrive_interface as odi


def _axis_config_card(axis_num: int, color: str):
    cfg = state.axis_cfg[axis_num]
    with ui.card().classes(f"border-l-4 border-{color}-6 w-full"):
        ui.label(f"Axis {axis_num} -- motor + encoder").classes(f"text-{color}-8 font-bold")

        with ui.grid(columns=2).classes("gap-x-4 w-full"):
            ui.label("Pole pairs")
            pp = ui.number(value=cfg.motor.pole_pairs, format="%.0f").props("dense")
            ui.label("Kv (rpm/V)")
            kv = ui.number(value=cfg.motor.kv, format="%.0f").props("dense")
            ui.label("Current limit (A)")
            ilim = ui.number(value=cfg.motor.current_lim, format="%.1f").props("dense")
            ui.label("Calibration current (A)")
            ical = ui.number(value=cfg.motor.calibration_current, format="%.1f").props("dense")
            ui.label("Resistance calib max V")
            rvmax = ui.number(value=cfg.motor.resistance_calib_max_voltage, format="%.1f").props("dense")
            ui.label("Torque constant (Nm/A)")
            tc_label = ui.label(f"{cfg.motor.torque_constant:.5f}  (= 8.27 / Kv)").classes("font-mono text-caption")

            ui.separator()
            ui.separator()

            ui.label("Encoder")
            ui.label("AS5047P, ABZ incremental").classes("text-caption text-grey-5")
            ui.label("CPR (counts/rev)")
            cpr = ui.number(value=cfg.encoder.cpr, format="%.0f").props("dense")
            ui.label("Use index (Z) channel")
            use_index = ui.switch(value=cfg.encoder.use_index)
            ui.label("Encoder bandwidth (Hz)")
            bw = ui.number(value=cfg.encoder.bandwidth, format="%.0f").props("dense")

        ui.label(
            "CPR must match the AS5047P's programmed ABI resolution -- 4096 "
            "is the sensor's factory default (binary mode, 1024 ppr x4 "
            "quadrature). With the Z line wired in and 'use index' on, the "
            "axis will run an encoder index search (state 6) as part of "
            "calibration and on every boot, so it re-homes to a repeatable "
            "reference each power cycle."
        ).classes("text-caption text-grey-6 col-span-2")

        def on_kv_change(e):
            tc_label.set_text(f"{8.27 / e.value if e.value else 0:.5f}  (= 8.27 / Kv)")
        kv.on_value_change(on_kv_change)

        async def apply():
            cfg.motor.pole_pairs = int(pp.value)
            cfg.motor.kv = float(kv.value)
            cfg.motor.current_lim = float(ilim.value)
            cfg.motor.calibration_current = float(ical.value)
            cfg.motor.resistance_calib_max_voltage = float(rvmax.value)
            cfg.motor.__post_init__()
            cfg.encoder.cpr = int(cpr.value)
            cfg.encoder.use_index = bool(use_index.value)
            cfg.encoder.bandwidth = float(bw.value)

            if not state.manager.connected:
                ui.notify("Not connected -- values saved locally only.", type="warning")
                return
            try:
                axis = state.manager.axis(axis_num)

                def work():
                    axis.configure_motor(cfg.motor)
                    axis.configure_encoder(cfg.encoder)
                await run.io_bound(work)
                ui.notify(f"Axis {axis_num} motor + encoder config applied.", type="positive")
                state.log(f"Applied motor+encoder config to axis {axis_num}")
            except Exception as ex:
                ui.notify(f"Failed to apply config: {ex}", type="negative")
                state.log(f"Config error axis {axis_num}: {ex}")

        ui.button(f"Apply to Axis {axis_num}", on_click=apply, icon="save").classes("mt-2")


def build():
    ui.label("Initial Configuration").classes("text-h6")
    ui.label(
        "Set motor and encoder parameters for each axis, then Save Configuration "
        "and Reboot before calibrating. Values are only written to the ODrive "
        "when you click Apply / Save -- editing a field alone does not touch hardware."
    ).classes("text-caption text-grey-5 mb-2")

    with ui.row().classes("gap-4 w-full"):
        with ui.column().classes("flex-1"):
            _axis_config_card(0, "cyan")
        with ui.column().classes("flex-1"):
            _axis_config_card(1, "amber")

    ui.separator().classes("my-3")
    ui.label("Board level").classes("text-subtitle2")
    with ui.grid(columns=2).classes("gap-x-4"):
        ui.label("Brake resistance (Ohm)")
        brake_r = ui.number(value=state.brake_resistance, format="%.2f").props("dense")
        ui.label("Enable brake resistor")
        brake_en = ui.switch(value=state.enable_brake_resistor)
        ui.label("DC bus overvoltage trip (V)")
        ov = ui.number(value=state.dc_bus_overvoltage_trip_level, format="%.1f").props("dense")
        ui.label("DC max negative (regen) current (A)")
        neg_i = ui.number(value=state.dc_max_negative_current, format="%.1f").props("dense")

    async def apply_board():
        state.brake_resistance = float(brake_r.value)
        state.enable_brake_resistor = bool(brake_en.value)
        state.dc_bus_overvoltage_trip_level = float(ov.value)
        state.dc_max_negative_current = float(neg_i.value)
        if not state.manager.connected:
            ui.notify("Not connected -- values saved locally only.", type="warning")
            return
        try:
            await run.io_bound(
                state.manager.configure_board,
                state.brake_resistance, state.enable_brake_resistor,
                state.dc_bus_overvoltage_trip_level, state.dc_max_negative_current,
            )
            ui.notify("Board config applied.", type="positive")
            state.log("Applied board-level config")
        except Exception as ex:
            ui.notify(f"Failed: {ex}", type="negative")

    with ui.row().classes("gap-2 mt-2"):
        ui.button("Apply Board Config", on_click=apply_board, icon="save")

        async def do_save():
            try:
                await run.io_bound(state.manager.save_configuration)
                ui.notify("Configuration saved to ODrive (device will reboot).", type="positive")
                state.log("save_configuration() called")
            except Exception as ex:
                ui.notify(f"Save failed: {ex}", type="negative")
        ui.button("Save Configuration", on_click=do_save, icon="download").props("outline")

        async def do_erase():
            try:
                await run.io_bound(state.manager.erase_configuration)
                ui.notify("Configuration erased.", type="warning")
                state.log("erase_configuration() called")
            except Exception as ex:
                ui.notify(f"Erase failed: {ex}", type="negative")
        ui.button("Erase Configuration", on_click=do_erase, icon="delete").props("outline color=red")

        async def do_reboot():
            try:
                await run.io_bound(state.manager.reboot)
                ui.notify("Reboot requested.", type="info")
                state.log("reboot() called")
            except Exception as ex:
                ui.notify(f"Reboot failed: {ex}", type="negative")
        ui.button("Reboot ODrive", on_click=do_reboot, icon="restart_alt").props("outline")
