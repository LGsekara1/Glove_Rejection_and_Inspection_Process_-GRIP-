"""JSON persistence for dashboard-side settings."""
from __future__ import annotations

import json
from pathlib import Path

from .constants import DEFAULT_CONFIG_FILENAME
from .models import DashboardConfig


class ConfigStore:
    def __init__(self, path: Path | None = None) -> None:
        if path is None:
            path = Path(__file__).resolve().parents[1] / DEFAULT_CONFIG_FILENAME
        self.path = path

    def load(self) -> DashboardConfig:
        if not self.path.exists():
            cfg = DashboardConfig()
            self.save(cfg)
            return cfg
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("Config root must be a JSON object.")
            return DashboardConfig.from_dict(data)
        except Exception as exc:
            raise RuntimeError(f"Could not load config file {self.path}: {exc}") from exc

    def save(self, cfg: DashboardConfig) -> None:
        cfg.validate()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.path.with_suffix(self.path.suffix + ".tmp")
        temp.write_text(json.dumps(cfg.to_dict(), indent=2, sort_keys=True), encoding="utf-8")
        temp.replace(self.path)
