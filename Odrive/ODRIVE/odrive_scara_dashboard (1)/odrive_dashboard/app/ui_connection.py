"""ui_connection.py -- connect/disconnect and live board status."""
from nicegui import ui, run

from state import state


def build():
    ui.label("Connection").classes("text-h6")
    ui.label(
        "Connect to the physical ODrive over USB, or run fully simulated to "
        "develop and test the dashboard with no hardware attached."
    ).classes("text-caption text-grey-5")

    with ui.row().classes("items-center gap-4 mt-2"):
        status_badge = ui.badge("DISCONNECTED", color="grey").classes("text-body2")
        serial_label = ui.label("").classes("font-mono text-caption")
        vbus_label = ui.label("").classes("font-mono text-caption")

    with ui.row().classes("gap-2 mt-3"):
        async def do_connect():
            connect_btn.props("loading")
            try:
                msg = await run.io_bound(state.manager.connect, 8.0, False)
                state.log(msg)
                ui.notify(msg, type="positive")
            except Exception as ex:
                ui.notify(str(ex), type="negative")
                state.log(f"Connection failed: {ex}")
            finally:
                connect_btn.props(remove="loading")

        async def do_simulate():
            import odrive_interface as odi
            msg = state.manager.connect(simulate=True)
            odi.start_simulation_clock(state.manager)
            state.log(msg)
            ui.notify(msg, type="positive")

        def do_disconnect():
            state.manager.disconnect()
            state.log("Disconnected")
            ui.notify("Disconnected", type="warning")

        connect_btn = ui.button("Connect to Hardware", on_click=do_connect, icon="usb")
        ui.button("Run Simulated", on_click=do_simulate, icon="science").props("outline")
        ui.button("Disconnect", on_click=do_disconnect, icon="link_off").props("outline color=red")

    ui.separator().classes("my-3")
    ui.label("Axis status").classes("text-subtitle2")
    with ui.row().classes("gap-6"):
        axis_cards = {}
        for axis_num, color in ((0, "cyan"), (1, "amber")):
            with ui.card().classes(f"border-l-4 border-{color}-6"):
                ui.label(f"Axis {axis_num}").classes(f"text-{color}-8 font-bold")
                state_lbl = ui.label("state: -").classes("font-mono text-caption")
                calib_lbl = ui.label("calibrated: -").classes("font-mono text-caption")
                err_lbl = ui.label("errors: -").classes("font-mono text-caption text-red")
                axis_cards[axis_num] = (state_lbl, calib_lbl, err_lbl)

    AXIS_STATE_NAMES = {
        1: "IDLE", 3: "FULL_CALIBRATION", 4: "MOTOR_CALIBRATION",
        6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION", 8: "CLOSED_LOOP_CONTROL",
    }

    def refresh():
        if state.manager.connected:
            status_badge.set_text("SIMULATED" if state.manager.is_sim else "CONNECTED")
            status_badge.props(f"color={'orange' if state.manager.is_sim else 'positive'}")
            serial_label.set_text(f"serial: {state.manager.serial_number}")
            vbus_label.set_text(f"vbus: {state.manager.vbus_voltage():.1f} V")
        else:
            status_badge.set_text("DISCONNECTED")
            status_badge.props("color=grey")
            serial_label.set_text("")
            vbus_label.set_text("")

        for axis_num, (state_lbl, calib_lbl, err_lbl) in axis_cards.items():
            if state.manager.connected:
                a = state.manager.axis(axis_num)
                t = a.read_telemetry()
                state_lbl.set_text(f"state: {AXIS_STATE_NAMES.get(t.current_state, t.current_state)}")
                calib_lbl.set_text(f"calibrated: {t.is_calibrated}")
                has_err = t.axis_error or t.motor_error or t.encoder_error or t.controller_error
                err_lbl.set_text(
                    f"errors: axis={t.axis_error} motor={t.motor_error} "
                    f"enc={t.encoder_error} ctrl={t.controller_error}" if has_err else "errors: none"
                )
                err_lbl.classes(remove="text-red" if not has_err else "", add="text-red" if has_err else "")
            else:
                state_lbl.set_text("state: -")
                calib_lbl.set_text("calibrated: -")
                err_lbl.set_text("errors: -")

    ui.timer(0.5, refresh)
