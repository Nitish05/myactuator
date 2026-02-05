"""
Monitor tab widget for Motor Recording Studio.

Real-time joint position graphs.
"""

from collections import deque
from typing import Dict, List

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QSpinBox, QFrame
)
from PyQt6.QtGui import QPainter, QPen, QColor


class PlotWidget(QWidget):
    """Simple line plot widget for joint values."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._data: Dict[str, deque] = {}
        self._colors = [
            QColor("#2196f3"),  # Blue
            QColor("#4caf50"),  # Green
            QColor("#ff9800"),  # Orange
            QColor("#e91e63"),  # Pink
            QColor("#9c27b0"),  # Purple
            QColor("#00bcd4"),  # Cyan
        ]
        self._max_points = 500
        self._y_min = -3.14159
        self._y_max = 3.14159
        self._show_grid = True

        self.setMinimumHeight(200)

    def set_joints(self, joint_names: List[str]):
        """Set the joints to plot."""
        self._data.clear()
        for name in joint_names:
            self._data[name] = deque(maxlen=self._max_points)

    def add_values(self, values: Dict[str, float]):
        """Add new values to the plot."""
        for name, value in values.items():
            if name in self._data:
                self._data[name].append(value)
        self.update()

    def set_y_range(self, y_min: float, y_max: float):
        """Set the Y axis range."""
        self._y_min = y_min
        self._y_max = y_max
        self.update()

    def set_max_points(self, max_points: int):
        """Set the maximum number of points to display."""
        self._max_points = max_points
        for name in self._data:
            old_data = list(self._data[name])
            self._data[name] = deque(old_data[-max_points:], maxlen=max_points)
        self.update()

    def clear(self):
        """Clear all data."""
        for name in self._data:
            self._data[name].clear()
        self.update()

    def paintEvent(self, event):
        """Paint the plot."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        margin = 40

        # Background
        painter.fillRect(0, 0, w, h, QColor("#1e1e1e"))

        plot_w = w - 2 * margin
        plot_h = h - 2 * margin

        if plot_w <= 0 or plot_h <= 0:
            return

        # Grid
        if self._show_grid:
            painter.setPen(QPen(QColor("#333333"), 1))

            # Horizontal grid lines
            for i in range(5):
                y = margin + int(i * plot_h / 4)
                painter.drawLine(margin, y, w - margin, y)

            # Vertical grid lines
            for i in range(5):
                x = margin + int(i * plot_w / 4)
                painter.drawLine(x, margin, x, h - margin)

        # Y axis labels
        painter.setPen(QPen(QColor("#888888"), 1))
        y_range = self._y_max - self._y_min
        for i in range(5):
            y = margin + int(i * plot_h / 4)
            value = self._y_max - (i / 4) * y_range
            painter.drawText(5, y + 4, f"{value:.2f}")

        # Plot lines
        for idx, (name, data) in enumerate(self._data.items()):
            if len(data) < 2:
                continue

            color = self._colors[idx % len(self._colors)]
            painter.setPen(QPen(color, 2))

            points = list(data)
            x_step = plot_w / (self._max_points - 1) if self._max_points > 1 else 1

            prev_x = None
            prev_y = None

            for i, value in enumerate(points):
                # X position based on age of point
                x = margin + int((self._max_points - len(points) + i) * x_step)

                # Y position based on value
                if y_range != 0:
                    normalized = (value - self._y_min) / y_range
                else:
                    normalized = 0.5
                y = margin + int((1 - normalized) * plot_h)

                # Clamp to plot area
                y = max(margin, min(h - margin, y))

                if prev_x is not None:
                    painter.drawLine(prev_x, prev_y, x, y)

                prev_x = x
                prev_y = y

        # Legend
        legend_x = margin + 10
        legend_y = margin + 15
        for idx, name in enumerate(self._data.keys()):
            color = self._colors[idx % len(self._colors)]
            painter.setPen(QPen(color, 2))
            painter.drawLine(legend_x, legend_y, legend_x + 20, legend_y)
            painter.setPen(QPen(QColor("#ffffff"), 1))
            painter.drawText(legend_x + 25, legend_y + 4, name)
            legend_y += 18


class MonitorTab(QWidget):
    """
    Monitor tab showing real-time joint position graphs.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._joint_names: List[str] = []
        self._setup_ui()

    def _setup_ui(self):
        """Set up the UI components."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        # Controls
        controls_layout = QHBoxLayout()

        # Y range
        controls_layout.addWidget(QLabel("Y Range:"))

        self._y_min_spin = QSpinBox()
        self._y_min_spin.setRange(-360, 360)
        self._y_min_spin.setValue(-180)
        self._y_min_spin.setSuffix(" deg")
        self._y_min_spin.valueChanged.connect(self._update_range)
        controls_layout.addWidget(self._y_min_spin)

        controls_layout.addWidget(QLabel("to"))

        self._y_max_spin = QSpinBox()
        self._y_max_spin.setRange(-360, 360)
        self._y_max_spin.setValue(180)
        self._y_max_spin.setSuffix(" deg")
        self._y_max_spin.valueChanged.connect(self._update_range)
        controls_layout.addWidget(self._y_max_spin)

        controls_layout.addSpacing(20)

        # History length
        controls_layout.addWidget(QLabel("History:"))
        self._history_spin = QSpinBox()
        self._history_spin.setRange(100, 5000)
        self._history_spin.setValue(500)
        self._history_spin.setSingleStep(100)
        self._history_spin.setSuffix(" pts")
        self._history_spin.valueChanged.connect(self._update_history)
        controls_layout.addWidget(self._history_spin)

        controls_layout.addStretch()

        layout.addLayout(controls_layout)

        # Plot
        self._plot = PlotWidget()
        layout.addWidget(self._plot, 1)

        # Apply initial range
        self._update_range()

    def _update_range(self):
        """Update the plot Y range."""
        import math
        y_min = math.radians(self._y_min_spin.value())
        y_max = math.radians(self._y_max_spin.value())
        self._plot.set_y_range(y_min, y_max)

    def _update_history(self):
        """Update the plot history length."""
        self._plot.set_max_points(self._history_spin.value())

    def update_joint_state(self, msg):
        """Update the plot with new joint state data."""
        # Check if joints changed
        new_names = list(msg.name)
        if new_names != self._joint_names:
            self._joint_names = new_names
            self._plot.set_joints(new_names)

        # Add values
        values = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                values[name] = msg.position[i]

        self._plot.add_values(values)

    def clear(self):
        """Clear the plot."""
        self._plot.clear()
