"""
Calibration window -- PyQt6 GUI for torque threshold calibration.

Provides CalibrationWindow (QMainWindow) that exposes CalibrationController
through a graphical interface with joint selection, torque/offset configuration,
recording name, live position display, and Record/Stop/Emergency Stop controls.

All calibration logic lives in CalibrationController. This module handles
only layout, signal wiring, and display updates.
"""

import math

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QFormLayout, QGroupBox,
    QComboBox, QDoubleSpinBox, QLineEdit, QLabel, QPushButton,
)

from myactuator_python_driver.studio.ros_bridge import RosBridge
from myactuator_python_driver.studio.recording_manager import RecordingManager
from .controller import CalibrationController
from .config import CalibrationConfig, CalibrationState


class CalibrationWindow(QMainWindow):
    """
    Main GUI window for torque threshold calibration.

    Owns RosBridge, RecordingManager, and CalibrationController.
    Follows the same ownership pattern as studio/main_window.py.
    """

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Torque Threshold Calibrator")
        self.setMinimumSize(500, 600)

        # Core components (same ownership pattern as MainWindow)
        self._ros_bridge = RosBridge(self)
        self._recording_manager = RecordingManager(self)
        self._controller = CalibrationController(
            self._ros_bridge, self._recording_manager, parent=self
        )

        # Connection state
        self._connected = False

        # Setup
        self._setup_ui()
        self._connect_signals()
        self._ros_bridge.start()

    # -- UI Setup ------------------------------------------------------------

    def _setup_ui(self):
        """Build the complete UI layout."""
        central = QWidget()
        self.setCentralWidget(central)

        layout = QVBoxLayout(central)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        # -- Connection status --
        self._connection_label = QLabel("Disconnected")
        self._connection_label.setStyleSheet(
            "color: #ef5350; font-weight: bold; font-size: 14px;"
        )
        layout.addWidget(self._connection_label)

        # -- Configuration group --
        config_group = QGroupBox("Configuration")
        config_layout = QFormLayout()
        config_group.setLayout(config_layout)

        self._joint_combo = QComboBox()
        self._joint_combo.setPlaceholderText("(waiting for joints...)")
        config_layout.addRow("Joint:", self._joint_combo)

        self._torque_spin = QDoubleSpinBox()
        self._torque_spin.setRange(-50.0, 50.0)
        self._torque_spin.setDecimals(2)
        self._torque_spin.setSingleStep(0.1)
        self._torque_spin.setSuffix(" Nm")
        self._torque_spin.setValue(1.0)
        config_layout.addRow("Torque:", self._torque_spin)

        self._offset_spin = QDoubleSpinBox()
        self._offset_spin.setRange(0.0, 90.0)
        self._offset_spin.setDecimals(1)
        self._offset_spin.setSingleStep(0.1)
        self._offset_spin.setSuffix(" deg")
        self._offset_spin.setValue(0.5)
        config_layout.addRow("Threshold Offset:", self._offset_spin)

        self._settle_spin = QDoubleSpinBox()
        self._settle_spin.setRange(0.5, 30.0)
        self._settle_spin.setDecimals(1)
        self._settle_spin.setSingleStep(0.5)
        self._settle_spin.setSuffix(" sec")
        self._settle_spin.setValue(3.0)
        config_layout.addRow("Settle Time:", self._settle_spin)

        self._name_edit = QLineEdit()
        self._name_edit.setPlaceholderText("Auto-generated if empty")
        config_layout.addRow("Recording Name:", self._name_edit)

        layout.addWidget(config_group)

        # -- Direction indicator --
        self._direction_label = QLabel("Direction: Positive (increasing position)")
        self._direction_label.setStyleSheet(
            "color: #90caf9; font-size: 13px; padding: 4px;"
        )
        layout.addWidget(self._direction_label)

        # -- Live data group --
        live_group = QGroupBox("Live Data")
        live_layout = QFormLayout()
        live_group.setLayout(live_layout)

        self._position_label = QLabel("--")
        self._position_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        live_layout.addRow("Position:", self._position_label)

        self._max_position_label = QLabel("--")
        self._max_position_label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: #ff9800;"
        )
        live_layout.addRow("Max Position:", self._max_position_label)

        layout.addWidget(live_group)

        # -- Record / Stop button --
        self._record_btn = QPushButton("Record")
        self._record_btn.setStyleSheet(
            "font-weight: bold; font-size: 16px; padding: 16px;"
        )
        self._record_btn.setMinimumHeight(60)
        layout.addWidget(self._record_btn)

        # -- Emergency Stop button --
        self._estop_btn = QPushButton("EMERGENCY STOP")
        self._estop_btn.setStyleSheet(
            "background-color: #d32f2f; color: white; font-weight: bold; "
            "font-size: 14px; border-radius: 8px; padding: 12px;"
        )
        self._estop_btn.setMinimumHeight(50)
        layout.addWidget(self._estop_btn)

        # Stretch at bottom
        layout.addStretch()

    # -- Signal Wiring -------------------------------------------------------

    def _connect_signals(self):
        """Connect all signals between components and UI."""
        # RosBridge signals
        self._ros_bridge.connection_status_changed.connect(
            self._on_connection_changed
        )
        self._ros_bridge.joint_state_received.connect(self._on_joint_state)

        # Controller signals
        self._controller.state_changed.connect(self._on_state_changed)
        self._controller.position_updated.connect(self._on_position_updated)
        self._controller.max_position_updated.connect(
            self._on_max_position_updated
        )
        self._controller.error_occurred.connect(self._on_error)
        self._controller.calibration_complete.connect(
            self._on_calibration_complete
        )

        # RecordingManager errors (shows actual exception details)
        self._recording_manager.error_occurred.connect(self._on_error)

        # Widget signals
        self._torque_spin.valueChanged.connect(self._on_torque_changed)
        self._record_btn.clicked.connect(self._on_record_clicked)
        self._estop_btn.clicked.connect(self._on_estop)

    # -- Handlers ------------------------------------------------------------

    def _on_connection_changed(self, connected: bool):
        """Update connection status display."""
        self._connected = connected
        if connected:
            self._connection_label.setText("Connected")
            self._connection_label.setStyleSheet(
                "color: #81c784; font-weight: bold; font-size: 14px;"
            )
        else:
            self._connection_label.setText("Disconnected")
            self._connection_label.setStyleSheet(
                "color: #ef5350; font-weight: bold; font-size: 14px;"
            )
        self._update_record_button_state()

    def _on_joint_state(self, msg):
        """Populate joint combo from first joint state message."""
        if self._joint_combo.count() == 0 and msg.name:
            self._joint_combo.addItems(list(msg.name))

    def _on_state_changed(self, state: CalibrationState):
        """Update UI based on calibration state transitions."""
        is_active = state in (
            CalibrationState.RECORDING, CalibrationState.STOPPING
        )

        # Disable/enable configuration inputs during recording (INPT-05)
        self._joint_combo.setEnabled(not is_active)
        self._torque_spin.setEnabled(not is_active)
        self._offset_spin.setEnabled(not is_active)
        self._settle_spin.setEnabled(not is_active)
        self._name_edit.setEnabled(not is_active)

        # Update record button appearance
        if state == CalibrationState.RECORDING:
            self._record_btn.setText("Stop")
            self._record_btn.setStyleSheet(
                "background-color: #d32f2f; color: white; "
                "font-weight: bold; font-size: 16px; padding: 16px;"
            )
        elif state == CalibrationState.IDLE:
            self._record_btn.setText("Record")
            self._record_btn.setStyleSheet(
                "font-weight: bold; font-size: 16px; padding: 16px;"
            )

        self._update_record_button_state()

    def _on_position_updated(self, position_rad: float):
        """Update live position display."""
        self._position_label.setText(
            f"{position_rad:.4f} rad ({math.degrees(position_rad):.2f} deg)"
        )

    def _on_max_position_updated(self, max_pos_rad: float):
        """Update max position display."""
        self._max_position_label.setText(
            f"{max_pos_rad:.4f} rad ({math.degrees(max_pos_rad):.2f} deg)"
        )

    def _on_error(self, message: str):
        """Display error message in status bar."""
        self.statusBar().showMessage(f"Error: {message}", 5000)

    def _on_calibration_complete(self, result):
        """Handle calibration completion."""
        self.statusBar().showMessage(
            f"Calibration complete: {result.joint_name} "
            f"threshold={math.degrees(result.max_position_rad):.2f} deg",
            10000,
        )
        # Reset position labels
        self._position_label.setText("--")
        self._max_position_label.setText("--")

    def _on_torque_changed(self, value: float):
        """Update direction indicator based on torque sign."""
        if value > 0:
            self._direction_label.setText(
                "Direction: Positive (increasing position)"
            )
        elif value < 0:
            self._direction_label.setText(
                "Direction: Negative (decreasing position)"
            )
        else:
            self._direction_label.setText("Direction: None (zero torque)")

    def _on_record_clicked(self):
        """Handle Record/Stop button click."""
        if self._controller.is_active:
            self._controller.stop_calibration()
        else:
            config = CalibrationConfig(
                joint_name=self._joint_combo.currentText(),
                torque_nm=self._torque_spin.value(),
                offset_deg=self._offset_spin.value(),
                settle_time_sec=self._settle_spin.value(),
                recording_name=self._name_edit.text().strip(),
            )
            self._controller.start_calibration(config)

    def _on_estop(self):
        """Handle Emergency Stop button click."""
        self._controller.emergency_stop()

    def _update_record_button_state(self):
        """Enable/disable Record button based on connection and state.

        Allows clicking Stop even if connection drops mid-calibration,
        but prevents starting a new calibration when disconnected.
        """
        self._record_btn.setEnabled(
            self._connected or self._controller.is_active
        )

    # -- Window lifecycle ----------------------------------------------------

    def closeEvent(self, event):
        """Clean up on window close."""
        if self._controller.is_active:
            self._controller.emergency_stop()
        self._ros_bridge.stop()
        event.accept()
