"""ui_scara.py -- 5-bar parallel linkage geometry, workspace, and IK calculator."""
import math

import numpy as np
import plotly.graph_objects as go
from nicegui import ui, run

from state import state
import kinematics as kin
import odrive_interface as odi


def _make_figure():
    fig = go.Figure()
    # 0: workspace heatmap (filled region)
    fig.add_trace(go.Heatmap(
        x=[], y=[], z=[], showscale=False,
        colorscale=[[0, "rgba(0,0,0,0)"], [1, "rgba(34,211,238,0.18)"]],
        zmin=0, zmax=1, hoverinfo="skip",
    ))
    # 1,2: crank links (axis0 cyan, axis1 amber)
    fig.add_trace(go.Scatter(x=[], y=[], mode="lines+markers", name="crank A (axis0)",
                              line=dict(color="#22d3ee", width=4), marker=dict(size=6)))
    fig.add_trace(go.Scatter(x=[], y=[], mode="lines+markers", name="crank B (axis1)",
                              line=dict(color="#f59e0b", width=4), marker=dict(size=6)))
    # 3,4: coupler links
    fig.add_trace(go.Scatter(x=[], y=[], mode="lines", name="coupler A-P",
                              line=dict(color="#67e8f9", width=3, dash="dot")))
    fig.add_trace(go.Scatter(x=[], y=[], mode="lines", name="coupler B-P",
                              line=dict(color="#fcd34d", width=3, dash="dot")))
    # 5: end effector
    fig.add_trace(go.Scatter(x=[], y=[], mode="markers", name="end effector",
                              marker=dict(size=11, color="#f8fafc", symbol="diamond")))
    # 6: target marker from the calculator
    fig.add_trace(go.Scatter(x=[], y=[], mode="markers", name="target",
                              marker=dict(size=13, color="#ef4444", symbol="x")))
    # 7: last trajectory path (if a Cartesian trajectory was generated)
    fig.add_trace(go.Scatter(x=[], y=[], mode="lines", name="planned path",
                              line=dict(color="#a78bfa", width=2)))

    fig.update_layout(
        height=560, margin=dict(l=40, r=20, t=20, b=40),
        paper_bgcolor="rgba(0,0,0,0)", plot_bgcolor="rgba(255,255,255,0.03)",
        font=dict(color="#cbd5e1"), legend=dict(orientation="h", y=1.06),
        xaxis=dict(scaleanchor="y", scaleratio=1, gridcolor="rgba(255,255,255,0.08)", title="x (m)"),
        yaxis=dict(gridcolor="rgba(255,255,255,0.08)", title="y (m)"),
        uirevision="keep",
    )
    return fig


def build():
    ui.label("5-Bar Parallel SCARA").classes("text-h6")
    ui.label(
        "O1 (axis0) and O2 (axis1) are the two fixed motor pivots. Each drives a "
        "crank link to a passive elbow; the two coupler links meet at the shared "
        "end effector. Inverse kinematics solves each side independently for the "
        "crank angle that lets its coupler reach the target point."
    ).classes("text-caption text-grey-5 mb-2")

    with ui.row().classes("gap-6 items-start w-full"):
        with ui.column().classes("gap-2"):
            ui.label("Geometry").classes("text-subtitle2")
            with ui.grid(columns=2).classes("gap-x-3"):
                ui.label("Base separation d (m)")
                d_in = ui.number(value=state.geo.d, format="%.4f", step=0.005).props("dense")
                ui.label("Crank A length L1a (m)")
                l1a_in = ui.number(value=state.geo.l1a, format="%.4f", step=0.005).props("dense")
                ui.label("Coupler A length L2a (m)")
                l2a_in = ui.number(value=state.geo.l2a, format="%.4f", step=0.005).props("dense")
                ui.label("Crank B length L1b (m)")
                l1b_in = ui.number(value=state.geo.l1b, format="%.4f", step=0.005).props("dense")
                ui.label("Coupler B length L2b (m)")
                l2b_in = ui.number(value=state.geo.l2b, format="%.4f", step=0.005).props("dense")
                ui.label("Elbow sign A")
                sign_a = ui.select([1, -1], value=state.geo.elbow_sign_a).props("dense")
                ui.label("Elbow sign B")
                sign_b = ui.select([1, -1], value=state.geo.elbow_sign_b).props("dense")

            limit_cb = ui.checkbox("Restrict joint travel", value=False)
            with ui.grid(columns=2).classes("gap-x-3"):
                ui.label("axis0 min / max (deg)")
                with ui.row().classes("gap-1"):
                    t1min = ui.number(value=-180, format="%.0f").classes("w-20").props("dense")
                    t1max = ui.number(value=180, format="%.0f").classes("w-20").props("dense")
                ui.label("axis1 min / max (deg)")
                with ui.row().classes("gap-1"):
                    t2min = ui.number(value=-180, format="%.0f").classes("w-20").props("dense")
                    t2max = ui.number(value=180, format="%.0f").classes("w-20").props("dense")

            def apply_geometry():
                state.geo.d = float(d_in.value)
                state.geo.l1a = float(l1a_in.value)
                state.geo.l2a = float(l2a_in.value)
                state.geo.l1b = float(l1b_in.value)
                state.geo.l2b = float(l2b_in.value)
                state.geo.elbow_sign_a = int(sign_a.value)
                state.geo.elbow_sign_b = int(sign_b.value)
                if limit_cb.value:
                    state.geo.theta1_min = math.radians(float(t1min.value))
                    state.geo.theta1_max = math.radians(float(t1max.value))
                    state.geo.theta2_min = math.radians(float(t2min.value))
                    state.geo.theta2_max = math.radians(float(t2max.value))
                else:
                    state.geo.theta1_min, state.geo.theta1_max = -math.pi, math.pi
                    state.geo.theta2_min, state.geo.theta2_max = -math.pi, math.pi
                redraw_workspace()
                ui.notify("Geometry updated.", type="positive")

            ui.button("Update Geometry", on_click=apply_geometry, icon="architecture").classes("mt-2")

            ui.separator().classes("my-2")
            ui.label("Coordinate -> angle calculator").classes("text-subtitle2")
            with ui.row().classes("gap-2 items-end"):
                x_in = ui.number(value=0.0, label="x (m)", format="%.4f").classes("w-28")
                y_in = ui.number(value=0.15, label="y (m)", format="%.4f").classes("w-28")
            result_lbl = ui.label("").classes("font-mono text-caption mt-1")

            def compute():
                x, y = float(x_in.value), float(y_in.value)
                ok, msg = kin.is_point_reachable(state.geo, x, y)
                target_trace.x = [x]
                target_trace.y = [y]
                plot.update()
                if not ok:
                    result_lbl.set_text(f"UNREACHABLE: {msg}")
                    result_lbl.classes(add="text-red", remove="text-green-5")
                    return
                t1, t2 = kin.inverse_kinematics(state.geo, x, y)
                result_lbl.set_text(
                    f"theta1 = {math.degrees(t1):8.3f} deg  ({t1/(2*math.pi):+.4f} turns)\n"
                    f"theta2 = {math.degrees(t2):8.3f} deg  ({t2/(2*math.pi):+.4f} turns)"
                )
                result_lbl.classes(add="text-green-5", remove="text-red")
                compute._last = (t1, t2)

            compute._last = None
            ui.button("Compute Angles", on_click=compute, icon="calculate").classes("mt-1")

            async def move_here():
                if compute._last is None:
                    ui.notify("Compute angles first.", type="warning")
                    return
                if not state.manager.connected:
                    ui.notify("Not connected.", type="negative")
                    return
                t1, t2 = compute._last
                a0, a1 = state.manager.axis(0), state.manager.axis(1)
                await run.io_bound(a0.set_input_pos, t1 / (2 * math.pi))
                await run.io_bound(a1.set_input_pos, t2 / (2 * math.pi))
                state.log(f"SCARA move-here -> theta1={math.degrees(t1):.2f} deg theta2={math.degrees(t2):.2f} deg")

            ui.button("Move Here", on_click=move_here, icon="my_location").props("outline").classes("mt-1")

        fig = _make_figure()
        plot = ui.plotly(fig).classes("flex-1")

    heatmap_trace = fig.data[0]
    crankA_trace = fig.data[1]
    crankB_trace = fig.data[2]
    couplerA_trace = fig.data[3]
    couplerB_trace = fig.data[4]
    ee_trace = fig.data[5]
    target_trace = fig.data[6]
    path_trace = fig.data[7]

    def redraw_workspace():
        xs, ys, mask = state.workspace(resolution=180)
        heatmap_trace.x = xs
        heatmap_trace.y = ys
        heatmap_trace.z = mask.astype(int)
        plot.update()

    redraw_workspace()

    def refresh_pose():
        geo = state.geo
        if state.manager.connected:
            t0 = state.manager.axis(0).read_telemetry()
            t1 = state.manager.axis(1).read_telemetry()
            theta1 = t0.pos * 2 * math.pi
            theta2 = t1.pos * 2 * math.pi
        else:
            theta1 = theta2 = None

        if theta1 is not None:
            try:
                A, B = kin.elbow_points(geo, theta1, theta2)
                P = kin.forward_kinematics(geo, theta1, theta2)
                O1, O2 = geo.O1, geo.O2
                crankA_trace.x, crankA_trace.y = [O1[0], A[0]], [O1[1], A[1]]
                crankB_trace.x, crankB_trace.y = [O2[0], B[0]], [O2[1], B[1]]
                couplerA_trace.x, couplerA_trace.y = [A[0], P[0]], [A[1], P[1]]
                couplerB_trace.x, couplerB_trace.y = [B[0], P[0]], [B[1], P[1]]
                ee_trace.x, ee_trace.y = [P[0]], [P[1]]
            except kin.KinematicsError:
                pass

        if state.last_samples is not None and state.last_samples.x is not None:
            path_trace.x = state.last_samples.x
            path_trace.y = state.last_samples.y
        plot.update()

    ui.timer(0.3, refresh_pose)
