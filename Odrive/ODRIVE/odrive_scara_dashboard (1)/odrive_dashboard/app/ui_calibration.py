"""ui_calibration.py -- calibration sequences and axis state control."""
import asyncio
from nicegui import ui, run

from state import state
import odrive_interface as odi

AXIS_STATE_NAMES = {
    1: "IDLE", 3: "FULL_CALIBRATION_SEQUENCE", 4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION", 8: "CLOSED_LOOP_CONTROL",
}


async def _run_state_and_wait(axis_num: int, requested_state: int, label: str, log_box):
    if not state.manager.connected:
        ui.notify("Not connected.", type="negative")
        return
    axis = state.manager.axis(axis_num)
    state.log(f"Axis {axis_num}: requesting {label}")
    log_box.refresh()

    def request():
        axis.request_state(requested_state)

    await run.io_bound(request)

    # Poll until the axis returns to IDLE (calibration/homing routines return
    # to idle on completion) or the state stops changing, with a safety
    # timeout so a lost connection can't hang the UI forever.
    start = asyncio.get_event_loop().time()
    timeout = 40.0
    while True:
        await asyncio.sleep(0.3)
        try:
            cur = await run.io_bound(axis.current_state)
        except Exception:
            break
        if cur == odi.AXIS_STATE_IDLE and requested_state != odi.AXIS_STATE_IDLE:
            break
        if requested_state == odi.AXIS_STATE_IDLE:
            break
        if asyncio.get_event_loop().time() - start > timeout:
            state.log(f"Axis {axis_num}: {label} timed out waiting for IDLE")
            break

    calibrated = axis.is_calibrated()
    errs = axis.errors_text()
    state.log(f"Axis {axis_num}: {label} finished. calibrated={calibrated}")
    if errs and "no errors" not in errs and errs.strip():
        state.log(f"Axis {axis_num} errors:\n{errs}")
    log_box.refresh()
    ui.notify(f"Axis {axis_num}: {label} complete", type="positive" if calibrated or requested_state == odi.AXIS_STATE_IDLE else "warning")


def _axis_calibration_card(axis_num: int, color: str, log_box):
    with ui.card().classes(f"border-l-4 border-{color}-6"):
        ui.label(f"Axis {axis_num}").classes(f"text-{color}-8 font-bold")

        with ui.row().classes("gap-2 flex-wrap mt-2"):
            ui.button(
                "Full Calibration Sequence",
                on_click=lambda: _run_state_and_wait(
                    axis_num, odi.AXIS_STATE_FULL_CALIBRATION_SEQUENCE, "full calibration sequence", log_box
                ),
                icon="tune",
            )
            ui.button(
                "Motor Calibration Only",
                on_click=lambda: _run_state_and_wait(
                    axis_num, odi.AXIS_STATE_MOTOR_CALIBRATION, "motor calibration", log_box
                ),
            ).props("outline")
            ui.button(
                "Encoder Index Search",
                on_click=lambda: _run_state_and_wait(
                    axis_num, odi.AXIS_STATE_ENCODER_INDEX_SEARCH, "encoder index search", log_box
                ),
            ).props("outline")
            ui.button(
                "Encoder Offset Calibration",
                on_click=lambda: _run_state_and_wait(
                    axis_num, odi.AXIS_STATE_ENCODER_OFFSET_CALIBRATION, "encoder offset calibration", log_box
                ),
            ).props("outline")

        with ui.row().classes("gap-2 mt-2"):
            ui.button(
                "Enter Closed Loop Control",
                on_click=lambda: _run_state_and_wait(
                    axis_num, odi.AXIS_STATE_CLOSED_LOOP_CONTROL, "enter closed loop control", log_box
                ),
                icon="play_arrow",
            ).props("color=positive")
            ui.button(
                "Set Idle",
                on_click=lambda: _run_state_and_wait(axis_num, odi.AXIS_STATE_IDLE, "set idle", log_box),
                icon="stop",
            ).props("color=red outline")

        status_lbl = ui.label("").classes("font-mono text-caption mt-2")

        def refresh_status():
            if state.manager.connected:
                a = state.manager.axis(axis_num)
                t = a.read_telemetry()
                status_lbl.set_text(
                    f"state={AXIS_STATE_NAMES.get(t.current_state, t.current_state)}  "
                    f"calibrated={t.is_calibrated}"
                )
            else:
                status_lbl.set_text("not connected")

        ui.timer(0.5, refresh_status)


def build():
    ui.label("Calibration").classes("text-h6")
    ui.label(
        "AS5047P is now wired in ABZ incremental mode. Unlike absolute mode, "
        "an incremental encoder has no memory of position across a power "
        "cycle, so the axis re-runs an encoder index search on every boot "
        "(the Z pulse gives it a repeatable zero) -- this happens "
        "automatically as part of the Full Calibration Sequence, or you can "
        "trigger it on its own with 'Encoder Index Search'. Motor "
        "calibration itself only needs to be redone if you change the "
        "motor/wiring. Make sure the rotor is free to spin before "
        "calibrating."
    ).classes("text-caption text-grey-5 mb-2")

    @ui.refreshable
    def log_box():
        with ui.scroll_area().classes("w-full h-40 bg-grey-10 rounded p-2"):
            for line in list(state.log_lines)[::-1]:
                ui.label(line).classes("font-mono text-caption text-grey-3")

    with ui.row().classes("gap-4"):
        _axis_calibration_card(0, "cyan", log_box)
        _axis_calibration_card(1, "amber", log_box)

    ui.separator().classes("my-3")
    ui.label("Activity log").classes("text-subtitle2")
    log_box()
