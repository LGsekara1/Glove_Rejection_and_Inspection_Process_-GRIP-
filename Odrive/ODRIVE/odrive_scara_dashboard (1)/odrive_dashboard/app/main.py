"""
main.py
=======
ODrive v3.6 dual-axis control dashboard.

Run with:
    python3 main.py

Then open the printed URL (defaults to http://localhost:8080) in a browser.
Everything here is a thin layout shell -- the actual logic lives in
odrive_interface.py, kinematics.py, trajectory.py, state.py and the ui_*.py
panel modules alongside this file.
"""
from nicegui import ui

import ui_connection
import ui_config
import ui_calibration
import ui_control
import ui_graphs
import ui_trajectory
import ui_scara


CUSTOM_CSS = """
body { background: #0f172a !important; }
.q-card { background: #111827; }
.font-mono { font-family: 'JetBrains Mono', 'Fira Code', monospace; }
::-webkit-scrollbar { width: 8px; height: 8px; }
::-webkit-scrollbar-thumb { background: #334155; border-radius: 4px; }
"""


@ui.page("/")
def index():
    ui.dark_mode().enable()
    ui.add_head_html(f"<style>{CUSTOM_CSS}</style>")
    ui.colors(primary="#22d3ee", secondary="#f59e0b", positive="#22c55e", negative="#ef4444")

    with ui.header().classes("items-center justify-between bg-slate-9 border-b border-slate-7"):
        with ui.row().classes("items-center gap-3"):
            ui.icon("precision_manufacturing").classes("text-2xl text-cyan-4")
            ui.label("ODrive v3.6 Dual-Axis Dashboard").classes("text-lg font-bold")
            ui.label("5065 / 270KV / AS5047P SPI").classes("text-caption text-grey-5")

    with ui.tabs().classes("w-full") as tabs:
        t_conn = ui.tab("Connection", icon="usb")
        t_conf = ui.tab("Configuration", icon="settings")
        t_calib = ui.tab("Calibration", icon="tune")
        t_ctrl = ui.tab("Control & Gains", icon="speed")
        t_graph = ui.tab("Graphs", icon="show_chart")
        t_traj = ui.tab("Trajectory", icon="timeline")
        t_scara = ui.tab("5-Bar SCARA", icon="architecture")

    with ui.tab_panels(tabs, value=t_conn).classes("w-full p-4"):
        with ui.tab_panel(t_conn):
            ui_connection.build()
        with ui.tab_panel(t_conf):
            ui_config.build()
        with ui.tab_panel(t_calib):
            ui_calibration.build()
        with ui.tab_panel(t_ctrl):
            ui_control.build()
        with ui.tab_panel(t_graph):
            ui_graphs.build()
        with ui.tab_panel(t_traj):
            ui_trajectory.build()
        with ui.tab_panel(t_scara):
            ui_scara.build()


ui.run(title="ODrive Dashboard", port=8080, reload=False, dark=True)
