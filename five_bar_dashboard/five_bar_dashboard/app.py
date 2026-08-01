"""Application entry point."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from .main_window import MainWindow
from .styles import APP_STYLESHEET


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Five-bar parallel SCARA ODrive dashboard")
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Run against the built-in ODrive simulator instead of physical hardware.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Optional dashboard JSON config path.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    app = QApplication(sys.argv if argv is None else [sys.argv[0], *argv])
    app.setApplicationName("Five-Bar SCARA Dashboard")
    app.setStyleSheet(APP_STYLESHEET)
    window = MainWindow(simulate=args.simulate, config_path=args.config)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
