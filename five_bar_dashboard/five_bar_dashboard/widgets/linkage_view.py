"""QPainter-based live five-bar linkage visualisation."""
from __future__ import annotations

import math
from typing import Iterable

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from ..kinematics import anchors
from ..models import DashboardConfig, TelemetrySample


class LinkageView(QWidget):
    def __init__(self, config: DashboardConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.config = config
        self.sample: TelemetrySample | None = None
        self.targets: list[tuple[float, float]] = []
        self.velocity_vector: tuple[float, float] | None = None
        self.setMinimumSize(500, 380)
        self.setToolTip("Top-down five-bar linkage view")

    def set_config(self, config: DashboardConfig) -> None:
        self.config = config
        self.update()

    def set_sample(self, sample: TelemetrySample) -> None:
        self.sample = sample
        self.update()

    def set_targets(self, targets: Iterable[tuple[float, float]]) -> None:
        self.targets = list(targets)
        self.update()

    def set_velocity_vector(self, vector: tuple[float, float] | None) -> None:
        self.velocity_vector = vector
        self.update()

    def _scale_and_origin(self) -> tuple[float, QPointF]:
        g = self.config.geometry
        if self.config.display.auto_fit:
            max_x = g.L0 / 2.0 + max(g.l1a + g.l2a, g.l1b + g.l2b)
            max_y = max(g.l1a + g.l2a, g.l1b + g.l2b)
            sx = max(0.05, (self.width() - 70.0) / (2.0 * max_x))
            sy = max(0.05, (self.height() - 70.0) / (1.15 * max_y))
            scale = min(sx, sy)
        else:
            scale = max(0.05, self.config.display.px_per_mm)
        origin = QPointF(self.width() / 2.0, self.height() * 0.82)
        return scale, origin

    @staticmethod
    def _screen(point: tuple[float, float], scale: float, origin: QPointF) -> QPointF:
        return QPointF(origin.x() + point[0] * scale, origin.y() - point[1] * scale)

    def _ring_path(
        self,
        centre: tuple[float, float],
        inner: float,
        outer: float,
        scale: float,
        origin: QPointF,
    ) -> QPainterPath:
        c = self._screen(centre, scale, origin)
        path = QPainterPath()
        path.setFillRule(Qt.FillRule.OddEvenFill)
        path.addEllipse(c, outer * scale, outer * scale)
        if inner > 0:
            path.addEllipse(c, inner * scale, inner * scale)
        return path

    def _draw_workspace(self, painter: QPainter, scale: float, origin: QPointF) -> None:
        g = self.config.geometry
        a, b = anchors(g)
        ring_a = self._ring_path(a, abs(g.l1a - g.l2a), g.l1a + g.l2a, scale, origin)
        ring_b = self._ring_path(b, abs(g.l1b - g.l2b), g.l1b + g.l2b, scale, origin)
        intersection = ring_a.intersected(ring_b)
        painter.save()
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(80, 170, 255, 38))
        painter.drawPath(intersection)
        painter.restore()

    @staticmethod
    def _draw_labeled_dot(
        painter: QPainter,
        point: QPointF,
        label: str,
        radius: float,
        colour: QColor,
        label_offset: QPointF = QPointF(7, -7),
    ) -> None:
        painter.setPen(QPen(QColor("#E5E7EB"), 1.0))
        painter.setBrush(colour)
        painter.drawEllipse(point, radius, radius)
        painter.drawText(point + label_offset, label)

    def _draw_arrow(
        self,
        painter: QPainter,
        start: QPointF,
        vector: tuple[float, float],
        scale: float,
    ) -> None:
        magnitude = math.hypot(*vector)
        if magnitude < 1e-9:
            return
        visual = min(110.0, max(25.0, magnitude * 1.5))
        ux, uy = vector[0] / magnitude, -vector[1] / magnitude
        end = QPointF(start.x() + ux * visual, start.y() + uy * visual)
        painter.setPen(QPen(QColor("#C084FC"), 3.0))
        painter.drawLine(start, end)
        angle = math.atan2(end.y() - start.y(), end.x() - start.x())
        head = 10.0
        p1 = QPointF(
            end.x() - head * math.cos(angle - math.pi / 6),
            end.y() - head * math.sin(angle - math.pi / 6),
        )
        p2 = QPointF(
            end.x() - head * math.cos(angle + math.pi / 6),
            end.y() - head * math.sin(angle + math.pi / 6),
        )
        painter.setBrush(QColor("#C084FC"))
        painter.drawPolygon(QPolygonF([end, p1, p2]))

    def paintEvent(self, event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#0B1220"))
        painter.setFont(QFont("Segoe UI", 9))
        scale, origin = self._scale_and_origin()
        g = self.config.geometry
        a, b = anchors(g)
        sa, sb = self._screen(a, scale, origin), self._screen(b, scale, origin)

        if self.config.display.show_workspace:
            self._draw_workspace(painter, scale, origin)

        base_pen = QPen(QColor("#64748B"), 1.5, Qt.PenStyle.DashLine)
        painter.setPen(base_pen)
        painter.drawLine(sa, sb)
        self._draw_labeled_dot(painter, sa, "A / axis0", 5, QColor("#60A5FA"))
        self._draw_labeled_dot(painter, sb, "B / axis1", 5, QColor("#34D399"))

        if self.sample and self.sample.p1 and self.sample.p2 and self.sample.end_effector:
            sp1 = self._screen(self.sample.p1, scale, origin)
            sp2 = self._screen(self.sample.p2, scale, origin)
            se = self._screen(self.sample.end_effector, scale, origin)

            painter.setPen(QPen(QColor("#3B82F6"), 5.0))
            painter.drawLine(sa, sp1)
            painter.setPen(QPen(QColor("#10B981"), 5.0))
            painter.drawLine(sb, sp2)
            painter.setPen(QPen(QColor("#60A5FA"), 3.0, Qt.PenStyle.DashLine))
            painter.drawLine(sp1, se)
            painter.setPen(QPen(QColor("#34D399"), 3.0, Qt.PenStyle.DashLine))
            painter.drawLine(sp2, se)

            self._draw_labeled_dot(painter, sp1, "P1", 4, QColor("#93C5FD"))
            self._draw_labeled_dot(painter, sp2, "P2", 4, QColor("#6EE7B7"))
            e = self.sample.end_effector
            self._draw_labeled_dot(
                painter,
                se,
                f"E  ({e[0]:.1f}, {e[1]:.1f}) mm",
                7,
                QColor("#EF4444"),
                QPointF(10, -10),
            )
            if self.velocity_vector is not None:
                self._draw_arrow(painter, se, self.velocity_vector, scale)
        else:
            painter.setPen(QColor("#FCA5A5"))
            painter.drawText(
                QRectF(12, 12, self.width() - 24, 30),
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                "FK pose unavailable / linkage not closed",
            )

        painter.setPen(QPen(QColor("#FBBF24"), 2.0))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        for index, target in enumerate(self.targets, start=1):
            point = self._screen(target, scale, origin)
            painter.drawEllipse(point, 8, 8)
            painter.drawText(point + QPointF(10, -5), str(index))

        painter.setPen(QColor("#94A3B8"))
        painter.drawText(
            12,
            self.height() - 12,
            f"Scale: {scale:.3f} px/mm | θ1={self.sample.theta_deg[0]:.2f}° θ2={self.sample.theta_deg[1]:.2f}°"
            if self.sample
            else f"Scale: {scale:.3f} px/mm",
        )
