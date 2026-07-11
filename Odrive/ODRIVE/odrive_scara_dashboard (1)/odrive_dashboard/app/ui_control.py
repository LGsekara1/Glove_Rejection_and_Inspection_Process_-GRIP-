"""ui_control.py -- control mode selection, gain tuning, manual setpoints."""
from nicegui import ui, run

from state import state
import odrive_interface as odi


def _axis_control_card(axis_num: int, color: str):
    cfg = state.axis_cfg[axis_num]
    with ui.card().classes(f"border-l-4 border-{color}-6 w-full"):
        ui.label(f"Axis {axis_num} -- control").classes(f"text-{color}-8 font-bold")

        with ui.row().classes("gap-4 items-end mt-2"):
            cmode = ui.select(list(odi.CONTROL_MODES.keys()), value=cfg.control_mode_label, label="Control mode").classes("w-40")
            imode = ui.select(list(odi.INPUT_MODES.keys()), value=cfg.input_mode_label, label="Input mode").classes("w-64")

            async def apply_mode():
                cfg.control_mode_label = cmode.value
                cfg.input_mode_label = imode.value
                if not state.manager.connected:
                    ui.notify("Not connected.", type="warning")
                    return
                axis = state.manager.axis(axis_num)
                await run.io_bound(
                    axis.set_control_mode,
                    odi.CONTROL_MODES[cmode.value], odi.INPUT_MODES[imode.value],
                )
                ui.notify(f"Axis {axis_num} -> {cmode.value} / {imode.value}", type="positive")
                state.log(f"Axis {axis_num} control_mode={cmode.value} input_mode={imode.value}")

            ui.button("Apply", on_click=apply_mode, icon="check")

        ui.separator().classes("my-2")
        ui.label("Gain tuning").classes("text-subtitle2")
        with ui.grid(columns=2).classes("gap-x-4"):
            ui.label("Position gain")
            pos_gain = ui.number(value=cfg.pos_gain, format="%.3f", step=0.5).props("dense")
            ui.label("Velocity gain")
            vel_gain = ui.number(value=cfg.vel_gain, format="%.4f", step=0.01).props("dense")
            ui.label("Velocity integrator gain")
            vel_int_gain = ui.number(value=cfg.vel_integrator_gain, format="%.4f", step=0.01).props("dense")
            ui.label("Velocity limit (turns/s)")
            vel_limit = ui.number(value=cfg.vel_limit, format="%.2f", step=0.5).props("dense")
            ui.label("Input filter bandwidth (Hz)")
            filt_bw = ui.number(value=cfg.input_filter_bandwidth, format="%.2f", step=0.5).props("dense")

        async def apply_gains():
            cfg.pos_gain = float(pos_gain.value)
            cfg.vel_gain = float(vel_gain.value)
            cfg.vel_integrator_gain = float(vel_int_gain.value)
            cfg.vel_limit = float(vel_limit.value)
            cfg.input_filter_bandwidth = float(filt_bw.value)
            if not state.manager.connected:
                ui.notify("Not connected -- values saved locally only.", type="warning")
                return
            axis = state.manager.axis(axis_num)
            await run.io_bound(
                axis.set_gains, cfg.pos_gain, cfg.vel_gain, cfg.vel_integrator_gain,
                cfg.vel_limit, cfg.input_filter_bandwidth,
            )
            ui.notify(f"Axis {axis_num} gains applied.", type="positive")
            state.log(f"Axis {axis_num} gains: pos={cfg.pos_gain} vel={cfg.vel_gain} "
                      f"vel_int={cfg.vel_integrator_gain} vel_limit={cfg.vel_limit}")

        ui.button("Apply Gains", on_click=apply_gains, icon="tune").classes("mt-2")

        ui.separator().classes("my-2")
        ui.label("Manual setpoint").classes("text-subtitle2")
        with ui.row().classes("gap-2 items-end"):
            sp_value = ui.number(value=0.0, label="target (turns / turns/s / Nm)", format="%.4f").classes("w-56")

            async def send_setpoint():
                if not state.manager.connected:
                    ui.notify("Not connected.", type="warning")
                    return
                axis = state.manager.axis(axis_num)
                mode = cfg.control_mode_label
                v = float(sp_value.value)
                if mode == "Position":
                    await run.io_bound(axis.set_input_pos, v)
                elif mode == "Velocity":
                    await run.io_bound(axis.set_input_vel, v)
                elif mode == "Torque":
                    await run.io_bound(axis.set_input_torque, v)
                else:
                    ui.notify("Voltage control setpoint not exposed in this dashboard.", type="warning")
                    return
                state.log(f"Axis {axis_num} setpoint ({mode}) -> {v}")

            ui.button("Send", on_click=send_setpoint, icon="send")

        with ui.row().classes("gap-2 mt-1"):
            ui.label("Step test:").classes("text-caption self-center")
            step_size = ui.number(value=0.25, label="step size", format="%.3f").classes("w-32")

            async def step_test():
                if not state.manager.connected:
                    ui.notify("Not connected.", type="warning")
                    return
                axis = state.manager.axis(axis_num)
                t = await run.io_bound(axis.read_telemetry)
                target = t.pos + float(step_size.value)
                await run.io_bound(axis.set_input_pos, target)
                state.log(f"Axis {axis_num} step test: {t.pos:.3f} -> {target:.3f} (watch the Graphs tab)")
                ui.notify("Step commanded -- see the Graphs tab for the response.", type="info")

            ui.button("Command Step", on_click=step_test, icon="stairs").props("outline")


def build():
    ui.label("Control & Gain Tuning").classes("text-h6")
    ui.label(
        "Choose a control mode + input mode, tune the position/velocity loop "
        "gains, and issue manual setpoints. 'Pos Filter' input mode is "
        "recommended when later streaming custom trajectories."
    ).classes("text-caption text-grey-5 mb-2")

    with ui.row().classes("gap-4 w-full items-start"):
        with ui.column().classes("flex-1"):
            _axis_control_card(0, "cyan")
        with ui.column().classes("flex-1"):
            _axis_control_card(1, "amber")
