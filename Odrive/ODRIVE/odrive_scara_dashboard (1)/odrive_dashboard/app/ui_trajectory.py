"""ui_trajectory.py -- build custom trajectories, preview them, and stream
sampled setpoints to the ODrive at a fixed rate."""
import asyncio

import numpy as np
import plotly.graph_objects as go
from nicegui import ui, run

from state import state
import trajectory as tr
import kinematics as kin
import odrive_interface as odi


def _label_for(mode: str):
    return ("axis0 (turns)", "axis1 (turns)") if mode == "Joint space" else ("x (m)", "y (m)")


def build():
    ui.label("Trajectory Planner").classes("text-h6")
    ui.label(
        "Define waypoints, generate a time-sampled trajectory, preview it, then "
        "stream it to the ODrive as position + velocity-feedforward setpoints."
    ).classes("text-caption text-grey-5 mb-2")

    with ui.row().classes("gap-4 items-end mb-2"):
        mode_sel = ui.select(["Joint space", "Cartesian (SCARA)"], value=state.trajectory_mode, label="Waypoint space")
        profile_sel = ui.select(
            ["Trapezoidal (stop-to-stop)", "Cubic spline (smooth)"], value=state.profile_type, label="Profile type"
        )

    with ui.row().classes("gap-4 items-end mb-2"):
        lim_a_vel = ui.number(value=state.traj_vel_limit0, label="vel limit A", format="%.3f")
        lim_a_acc = ui.number(value=state.traj_accel_limit0, label="accel limit A", format="%.3f")
        lim_b_vel = ui.number(value=state.traj_vel_limit1, label="vel limit B", format="%.3f")
        lim_b_acc = ui.number(value=state.traj_accel_limit1, label="accel limit B", format="%.3f")
        dt_field = ui.number(value=state.traj_dt, label="sample dt (s)", format="%.4f")
        dwell_field = ui.number(value=state.traj_dwell, label="dwell between segments (s)", format="%.3f")

    label_a, label_b = _label_for(state.trajectory_mode)
    col_label_a = ui.label(label_a)
    col_label_b = ui.label(label_b)

    @ui.refreshable
    def waypoint_rows():
        mode = mode_sel.value
        la, lb = _label_for(mode)
        col_label_a.set_text(la)
        col_label_b.set_text(lb)
        show_time = profile_sel.value.startswith("Cubic")

        with ui.column().classes("gap-1 w-full"):
            header_cols = "auto 1fr 1fr" + (" 1fr" if show_time else "") + " auto"
            with ui.row().classes("gap-2 text-caption text-grey-5"):
                ui.label("#").classes("w-8")
                ui.label(la).classes("w-28")
                ui.label(lb).classes("w-28")
                if show_time:
                    ui.label("t (s)").classes("w-24")
                ui.label("")

            for i, wp in enumerate(state.waypoints):
                with ui.row().classes("gap-2 items-center"):
                    ui.label(str(i)).classes("w-8 text-caption")
                    a_in = ui.number(value=wp["a"], format="%.4f").classes("w-28").props("dense")
                    b_in = ui.number(value=wp["b"], format="%.4f").classes("w-28").props("dense")

                    def bind_a(e, idx=i):
                        state.waypoints[idx]["a"] = float(e.value)
                    def bind_b(e, idx=i):
                        state.waypoints[idx]["b"] = float(e.value)
                    a_in.on_value_change(bind_a)
                    b_in.on_value_change(bind_b)

                    if show_time:
                        t_in = ui.number(value=wp["t"], format="%.3f").classes("w-24").props("dense")
                        def bind_t(e, idx=i):
                            state.waypoints[idx]["t"] = float(e.value)
                        t_in.on_value_change(bind_t)

                    def remove(idx=i):
                        if len(state.waypoints) > 2:
                            state.waypoints.pop(idx)
                            waypoint_rows.refresh()
                        else:
                            ui.notify("Need at least 2 waypoints.", type="warning")
                    ui.button(icon="delete", on_click=remove).props("flat dense round")

    waypoint_rows()

    def add_waypoint():
        last = state.waypoints[-1]
        state.waypoints.append({"t": last["t"] + 1.0, "a": last["a"], "b": last["b"]})
        waypoint_rows.refresh()

    def mode_or_profile_changed():
        state.trajectory_mode = mode_sel.value
        state.profile_type = profile_sel.value
        waypoint_rows.refresh()

    mode_sel.on_value_change(lambda e: mode_or_profile_changed())
    profile_sel.on_value_change(lambda e: mode_or_profile_changed())

    ui.button("Add Waypoint", on_click=add_waypoint, icon="add").classes("mt-1")

    ui.separator().classes("my-3")

    preview_fig = go.Figure()
    preview_fig.add_trace(go.Scatter(x=[], y=[], name="A", line=dict(color="#22d3ee")))
    preview_fig.add_trace(go.Scatter(x=[], y=[], name="B", line=dict(color="#f59e0b")))
    preview_fig.update_layout(
        height=280, margin=dict(l=50, r=20, t=30, b=30), title="Preview: position vs time",
        paper_bgcolor="rgba(0,0,0,0)", plot_bgcolor="rgba(255,255,255,0.03)",
        font=dict(color="#cbd5e1"), legend=dict(orientation="h", y=1.15),
    )
    preview_fig.update_xaxes(gridcolor="rgba(255,255,255,0.08)", title_text="time (s)")
    preview_fig.update_yaxes(gridcolor="rgba(255,255,255,0.08)")
    preview_plot = ui.plotly(preview_fig).classes("w-full")

    info_label = ui.label("").classes("text-caption text-grey-5")

    def generate():
        state.traj_vel_limit0 = float(lim_a_vel.value)
        state.traj_accel_limit0 = float(lim_a_acc.value)
        state.traj_vel_limit1 = float(lim_b_vel.value)
        state.traj_accel_limit1 = float(lim_b_acc.value)
        state.traj_dt = float(dt_field.value)
        state.traj_dwell = float(dwell_field.value)

        wps = [(w["a"], w["b"]) for w in state.waypoints]
        lim_a = tr.AxisLimits(vel_limit=state.traj_vel_limit0, accel_limit=state.traj_accel_limit0)
        lim_b = tr.AxisLimits(vel_limit=state.traj_vel_limit1, accel_limit=state.traj_accel_limit1)

        try:
            if state.trajectory_mode == "Joint space":
                if state.profile_type.startswith("Trapezoidal"):
                    samples = tr.trapezoidal_multiaxis(wps, lim_a, lim_b, state.traj_dt, state.traj_dwell)
                else:
                    times = [w["t"] for w in state.waypoints]
                    samples = tr.spline_multiaxis(times, wps, state.traj_dt)
            else:
                if state.profile_type.startswith("Trapezoidal"):
                    samples = tr.cartesian_trapezoidal(state.geo, wps, lim_a, lim_b, state.traj_dt, state.traj_dwell)
                else:
                    times = [w["t"] for w in state.waypoints]
                    samples = tr.cartesian_spline(state.geo, times, wps, state.traj_dt)
        except (tr.TrajectoryError, kin.KinematicsError) as ex:
            ui.notify(f"Trajectory generation failed: {ex}", type="negative")
            return

        state.last_samples = samples
        preview_fig.data[0].x = samples.t
        preview_fig.data[0].y = samples.pos0
        preview_fig.data[1].x = samples.t
        preview_fig.data[1].y = samples.pos1
        preview_plot.update()
        info_label.set_text(
            f"Generated {len(samples.t)} samples, duration {samples.duration:.3f} s "
            f"({len(samples.t)/max(samples.duration,1e-6):.0f} Hz effective)."
        )
        ui.notify("Trajectory generated -- see preview below.", type="positive")

    ui.button("Generate & Preview", on_click=generate, icon="show_chart").classes("mt-2")
    info_label

    ui.separator().classes("my-3")
    ui.label("Stream to ODrive").classes("text-subtitle2")
    with ui.row().classes("gap-3 items-center"):
        loop_cb = ui.checkbox("Loop", value=state.stream_loop)
        progress = ui.linear_progress(value=0.0).classes("w-64")
        progress_lbl = ui.label("idle").classes("text-caption")

        async def stream_once(samples):
            axis0 = state.manager.axis(0)
            axis1 = state.manager.axis(1)
            n = len(samples.t)
            loop = asyncio.get_event_loop()
            t_start = loop.time()
            for i in range(n):
                if not state.stream_running:
                    return
                axis0.set_input_pos(float(samples.pos0[i]), vel_ff=float(samples.vel0[i]))
                axis1.set_input_pos(float(samples.pos1[i]), vel_ff=float(samples.vel1[i]))
                state.stream_progress = (i + 1) / n
                progress.set_value(state.stream_progress)
                progress_lbl.set_text(f"streaming... {i+1}/{n}")
                target = t_start + float(samples.t[i])
                delay = target - loop.time()
                if delay > 0:
                    await asyncio.sleep(delay)
                else:
                    await asyncio.sleep(0)  # yield to event loop even if we're behind

        async def stream_task():
            state.stream_running = True
            try:
                if state.last_samples is None:
                    ui.notify("Generate a trajectory first.", type="warning")
                    return
                if not state.manager.connected:
                    ui.notify("Not connected.", type="negative")
                    return
                first = True
                while state.stream_running and (first or state.stream_loop):
                    first = False
                    await stream_once(state.last_samples)
                progress_lbl.set_text("done")
                state.log("Trajectory stream finished")
            finally:
                state.stream_running = False

        def start():
            if state.last_samples is None:
                ui.notify("Generate a trajectory first.", type="warning")
                return
            state.stream_loop = loop_cb.value
            state.stream_task = asyncio.create_task(stream_task())

        def stop():
            state.stream_running = False
            progress_lbl.set_text("stopped")

        ui.button("Send to ODrive", on_click=start, icon="play_arrow").props("color=positive")
        ui.button("Stop", on_click=stop, icon="stop").props("color=red outline")
