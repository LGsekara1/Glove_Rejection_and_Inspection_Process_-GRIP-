"""Scrollable, refreshable ODrive error-report dialog."""
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Signal
from PySide6.QtGui import QFontDatabase, QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class ErrorReportDialog(QDialog):
    refresh_requested = Signal()
    clear_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("ODrive Error Report")
        self.resize(860, 580)
        self.setModal(False)

        layout = QVBoxLayout(self)
        self.summary = QLabel("No report loaded.")
        self.summary.setStyleSheet("font-size: 12pt; font-weight: 800;")
        layout.addWidget(self.summary)

        self.report = QPlainTextEdit()
        self.report.setReadOnly(True)
        self.report.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.report.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        layout.addWidget(self.report, 1)

        buttons = QHBoxLayout()
        refresh = QPushButton("Refresh")
        clear = QPushButton("Clear Errors")
        copy = QPushButton("Copy Report")
        close = QPushButton("Close")
        refresh.clicked.connect(self.refresh_requested)
        clear.clicked.connect(self.clear_requested)
        copy.clicked.connect(self._copy)
        close.clicked.connect(self.close)
        buttons.addWidget(refresh)
        buttons.addWidget(clear)
        buttons.addStretch(1)
        buttons.addWidget(copy)
        buttons.addWidget(close)
        layout.addLayout(buttons)

    def set_loading(self) -> None:
        self.summary.setText("Reading ODrive error registers...")
        self.summary.setStyleSheet("font-size: 12pt; font-weight: 800; color: #FBBF24;")

    def set_report(self, payload: object) -> None:
        if isinstance(payload, dict):
            has_errors = bool(payload.get("has_errors", False))
            text = str(payload.get("formatted_text", "No formatted report was returned."))
            captured = str(payload.get("captured_at", "unknown time"))
            if has_errors:
                self.summary.setText(f"ACTIVE ERROR(S) PRESENT  |  {captured}")
                self.summary.setStyleSheet(
                    "font-size: 12pt; font-weight: 800; color: #F87171;"
                )
            else:
                self.summary.setText(f"No active errors  |  {captured}")
                self.summary.setStyleSheet(
                    "font-size: 12pt; font-weight: 800; color: #34D399;"
                )
        else:
            text = str(payload) if payload else "No error data was returned."
            self.summary.setText("Error report returned as plain text")
            self.summary.setStyleSheet(
                "font-size: 12pt; font-weight: 800; color: #FBBF24;"
            )
        self.report.setPlainText(text)
        self.report.moveCursor(QTextCursor.MoveOperation.Start)

    def set_failure(self, message: str) -> None:
        self.summary.setText("Failed to read ODrive errors")
        self.summary.setStyleSheet("font-size: 12pt; font-weight: 800; color: #F87171;")
        self.report.setPlainText(message)

    def _copy(self) -> None:
        QApplication.clipboard().setText(self.report.toPlainText())
