APP_STYLESHEET = r"""
QMainWindow, QWidget {
    background: #0F172A;
    color: #E5E7EB;
    font-family: "Segoe UI", "Arial";
    font-size: 10pt;
}
QFrame#TopBar {
    background: #111827;
    border-bottom: 1px solid #334155;
}
QFrame#LivePositionBar {
    background: #0B1220;
    border-bottom: 1px solid #334155;
}
QGroupBox {
    border: 1px solid #334155;
    border-radius: 7px;
    margin-top: 10px;
    padding-top: 8px;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
}
QPushButton {
    background: #1E293B;
    border: 1px solid #475569;
    border-radius: 6px;
    padding: 7px 10px;
}
QPushButton:hover { background: #334155; }
QPushButton:pressed { background: #0B1220; }
QPushButton:disabled { color: #64748B; border-color: #334155; }
QPushButton#EmergencyButton {
    background: #B91C1C;
    border: 2px solid #FCA5A5;
    color: white;
    font-weight: 800;
    padding: 9px 16px;
}
QPushButton#EmergencyButton:hover { background: #DC2626; }
QPushButton#ResumeButton { background: #166534; border-color: #4ADE80; font-weight: 700; }
QLineEdit, QPlainTextEdit, QComboBox, QSpinBox, QDoubleSpinBox {
    background: #111827;
    border: 1px solid #475569;
    border-radius: 4px;
    padding: 4px;
    selection-background-color: #2563EB;
}
QTabWidget::pane { border: 1px solid #334155; }
QTabBar::tab {
    background: #111827;
    border: 1px solid #334155;
    padding: 7px 10px;
}
QTabBar::tab:selected { background: #1D4ED8; }
QScrollBar:vertical { background: #0F172A; width: 12px; }
QScrollBar::handle:vertical { background: #475569; min-height: 28px; border-radius: 5px; }
QLabel[class="warning"] { color: #FBBF24; }
QLabel[state="ok"] { color: #4ADE80; }
QLabel[state="error"] { color: #F87171; }
QLabel[state="warn"] { color: #FBBF24; }
QLabel[state="info"] { color: #93C5FD; }
QSplitter::handle { background: #334155; }
"""
