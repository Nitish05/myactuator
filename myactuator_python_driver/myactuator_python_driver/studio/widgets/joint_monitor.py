"""
Joint monitor widget for Motor Recording Studio.

Bottom dock widget showing real-time joint state values.
"""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTableView,
    QHeaderView, QAbstractItemView
)

from myactuator_python_driver.studio.models.joint_state_model import JointStateModel


class JointMonitor(QWidget):
    """
    Joint monitor widget showing live joint values.

    Displays a table with:
    - Joint name
    - Position (rad)
    - Velocity (rad/s)
    - Effort (Nm)
    - Temperature (C)
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        """Set up the UI components."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Header
        header_layout = QHBoxLayout()
        header_label = QLabel("Joint States")
        header_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                font-size: 11px;
                color: #90caf9;
            }
        """)
        header_layout.addWidget(header_label)
        header_layout.addStretch()

        # Update rate indicator
        self._update_label = QLabel("-- Hz")
        self._update_label.setStyleSheet("color: #757575;")
        header_layout.addWidget(self._update_label)

        layout.addLayout(header_layout)

        # Table view
        self._model = JointStateModel()
        self._table = QTableView()
        self._table.setObjectName("jointMonitorTable")
        self._table.setModel(self._model)

        # Table styling
        self._table.setAlternatingRowColors(True)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setShowGrid(True)
        self._table.verticalHeader().setVisible(False)

        # Column sizing
        header = self._table.horizontalHeader()
        header.setStretchLastSection(False)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)  # Name stretches
        for i in range(1, len(JointStateModel.COLUMNS)):
            header.setSectionResizeMode(i, QHeaderView.ResizeMode.ResizeToContents)

        # Row height
        self._table.verticalHeader().setDefaultSectionSize(24)

        layout.addWidget(self._table)

        # Update rate tracking
        self._update_count = 0
        self._last_update_time = 0.0

    def update_joint_state(self, msg):
        """Update the joint state display from a JointState message."""
        self._model.update_joint_state(msg)

        # Track update rate
        import time
        now = time.time()
        self._update_count += 1

        if now - self._last_update_time >= 1.0:
            rate = self._update_count / (now - self._last_update_time)
            self._update_label.setText(f"{rate:.0f} Hz")
            self._update_count = 0
            self._last_update_time = now

    def update_temperatures(self, temps):
        """Update temperature values."""
        self._model.update_temperatures(temps)

    def get_model(self) -> JointStateModel:
        """Get the joint state model."""
        return self._model

    def get_joint_names(self):
        """Get list of joint names."""
        return self._model.get_joint_names()

    def get_joint_positions(self):
        """Get dict of joint positions."""
        return self._model.get_joint_positions()
