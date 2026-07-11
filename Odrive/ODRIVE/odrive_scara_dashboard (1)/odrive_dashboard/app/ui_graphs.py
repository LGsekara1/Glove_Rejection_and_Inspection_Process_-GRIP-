"""ui_graphs.py -- toggleable real-time telemetry graphs."""
import time

import plotly.graph_objects as go
from plotly.subplots import make_subplots
from nicegui import ui

from state import state

AXIS_COLOR = {0: "#22d3ee", 1: "#f59e0b"}  # cyan / amber, matches the rest of the UI


def _make_figure():
    fig = make_subplots(
        rows=3, cols=1, shared_xaxes=True, vertical_spacing=0.06,
        subplot_titles=("Position (turns)", "Velocity (turns/s)", "Current Iq (A)"),
    )
    for row, key in ((1, "pos"), (2, "vel"), (3, "iq")):
        for axis_num in (0, 1):
            fig.add_trace(
                go.Scattergl(
                    x=[], y=[], mode="lines", name=f"axis{axis_num}",
                    legendgroup=f"axis{axis_num}", showlegend=(row == 1),
                    line=dict(color=AXIS_COLOR[axis_num], width=1.6),
                ),
                row=row, col=1,
            )
    fig.update_layout(
        height=650, margin=dict(l=50, r=20, t=40, b=30),
        paper_bgcolor="rgba(0,0,0,0)", plot_bgcolor="rgba(255,255,255,0.03)",
        font=dict(color="#cbd5e1"), legend=dict(orientation="h", y=1.08),
        uirevision="keep",
    )
    fig.update_xaxes(gridcolor="rgba(255,255,255,0.08)", title_text="time (s)", row=3, col=1)
    fig.update_yaxes(gridcolor="rgba(255,255,255,0.08)")
    return fig


def build():
    ui.label("Live Graphs").classes("text-h6")
    ui.label("Toggle quantities and axes on/off; the chart keeps a rolling time window.").classes(
        "text-caption text-grey-5 mb-2"
    )

    with ui.row().classes("gap-4 items-center mb-2 flex-wrap"):
        cb_pos = ui.checkbox("Position", value=state.show_position)
        cb_vel = ui.checkbox("Velocity", value=state.show_velocity)
        cb_cur = ui.checkbox("Current", value=state.show_current)
        ui.separator().props("vertical")
        cb_a0 = ui.checkbox("Axis 0", value=state.show_axis0).classes("text-cyan-6")
        cb_a1 = ui.checkbox("Axis 1", value=state.show_axis1).classes("text-amber-6")
        ui.separator().props("vertical")
        window = ui.slider(min=2, max=120, value=state.time_window_s).props("label-always").classes("w-48")
        ui.label("window (s)").classes("text-caption")
        paused = {"value": False}

        def toggle_pause():
            paused["value"] = not paused["value"]
            pause_btn.set_text("Resume" if paused["value"] else "Pause")

        pause_btn = ui.button("Pause", on_click=toggle_pause).props("outline")

        def clear_buffers():
            for buf in state.telemetry.values():
                buf.clear()

        ui.button("Clear", on_click=clear_buffers, icon="delete").props("outline")

    fig = _make_figure()
    plot = ui.plotly(fig).classes("w-full")

    row_visible = {"pos": True, "vel": True, "iq": True}
    axis_visible = {0: True, 1: True}

    def update_visibility():
        row_visible["pos"] = cb_pos.value
        row_visible["vel"] = cb_vel.value
        row_visible["iq"] = cb_cur.value
        axis_visible[0] = cb_a0.value
        axis_visible[1] = cb_a1.value
        state.show_position, state.show_velocity, state.show_current = cb_pos.value, cb_vel.value, cb_cur.value
        state.show_axis0, state.show_axis1 = cb_a0.value, cb_a1.value
        state.time_window_s = window.value

    for cb in (cb_pos, cb_vel, cb_cur, cb_a0, cb_a1):
        cb.on_value_change(lambda e: update_visibility())
    window.on_value_change(lambda e: update_visibility())

    def poll_and_redraw():
        # Poll hardware/simulator regardless of pause state (so data isn't
        # lost), but only push a redraw to the browser when not paused.
        now = state.elapsed()
        if state.manager.connected:
            for axis_num in (0, 1):
                axis = state.manager.axis(axis_num)
                t = axis.read_telemetry()
                state.telemetry[axis_num].append(now, t)

        if paused["value"]:
            return

        window_s = state.time_window_s
        trace_i = 0
        for key in ("pos", "vel", "iq"):
            for axis_num in (0, 1):
                buf = state.telemetry[axis_num]
                if buf.t:
                    tarr = list(buf.t)
                    cutoff = tarr[-1] - window_s
                    idx0 = 0
                    for i, tv in enumerate(tarr):
                        if tv >= cutoff:
                            idx0 = i
                            break
                    xs = tarr[idx0:]
                    ys = list(getattr(buf, key))[idx0:]
                else:
                    xs, ys = [], []
                visible = row_visible[key] and axis_visible[axis_num]
                fig.data[trace_i].x = xs
                fig.data[trace_i].y = ys
                fig.data[trace_i].visible = True if visible else "legendonly"
                trace_i += 1
        plot.update()

    ui.timer(0.1, poll_and_redraw)
