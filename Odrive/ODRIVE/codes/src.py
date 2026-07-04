import odrive
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import sys

# 1. Connect
my_drive = odrive.find_any()
my_drive.axis0.requested_state = 8  # AXIS_STATE_CLOSED_LOOP_CONTROL

# 2. GUI Setup
app = QtWidgets.QApplication(sys.argv)
win = QtWidgets.QWidget()
layout = QtWidgets.QHBoxLayout()
win.setLayout(layout)

# Create Plot
plot_widget = pg.PlotWidget(title="Real-time Position")
curve = plot_widget.plot(pen='y')
layout.addWidget(plot_widget)

# Create Slider
slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Vertical)
slider.setRange(-1, 1)  # Adjust range based on your encoder CPR
slider.valueChanged.connect(lambda v: setattr(my_drive.axis0.controller, 'input_pos', v))
layout.addWidget(slider)

win.show()

# 3. Update Loop
data = []
def update():
    pos = my_drive.axis0.encoder.pos_estimate
    data.append(pos)
    if len(data) > 200: data.pop(0)
    curve.setData(data)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

# 4. Run
if hasattr(app, "exec"): sys.exit(app.exec())
else: sys.exit(app.exec_())