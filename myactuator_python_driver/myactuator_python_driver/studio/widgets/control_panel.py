"""
Control panel widget for Motor Recording Studio.

Left dock widget containing motor control options.
"""

from typing import List

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QPushButton, QFrame, QSizePolicy, QDoubleSpinBox
)

from myactuator_python_driver.can_utils import CANScanner


class ControlPanel(QWidget):
    """
    Control panel widget with motor control options.

    Contains:
    - CAN interface selector
    - Mode selector
    - Enable/disable toggle
    - Set zero button
    - Go to zero button
    - E-Stop button
    """

    # Signals
    mode_requested = Signal(str)
    enable_requested = Signal(bool)
    set_zero_requested = Signal()
    go_to_zero_requested = Signal()
    emergency_stop_requested = Signal()

    MODES = ["position", "velocity", "torque", "force_position", "admittance", "free", "disabled"]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._enabled = True
        self._current_mode = "position"
        self._setup_ui()

    def _setup_ui(self):
        """Set up the UI components."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(12)

        # CAN Interface section
        layout.addWidget(self._create_section_label("CAN Interface"))

        can_layout = QHBoxLayout()
        self._can_combo = QComboBox()
        self._can_combo.setMinimumWidth(100)
        can_layout.addWidget(self._can_combo, 1)

        self._can_refresh_btn = QPushButton("Refresh")
        self._can_refresh_btn.setMaximumWidth(70)
        self._can_refresh_btn.clicked.connect(self._refresh_can_interfaces)
        can_layout.addWidget(self._can_refresh_btn)

        layout.addLayout(can_layout)

        # Mode section
        layout.addWidget(self._create_section_label("Control Mode"))

        self._mode_combo = QComboBox()
        self._mode_combo.addItems(self.MODES)
        self._mode_combo.currentTextChanged.connect(self._on_mode_changed)
        layout.addWidget(self._mode_combo)

        # Torque limit (for force_position mode)
        self._torque_limit_widget = QWidget()
        tl_layout = QHBoxLayout(self._torque_limit_widget)
        tl_layout.setContentsMargins(0, 4, 0, 0)
        tl_layout.addWidget(QLabel("Max Torque:"))

        self._torque_limit_spin = QDoubleSpinBox()
        self._torque_limit_spin.setRange(5.0, 100.0)
        self._torque_limit_spin.setValue(50.0)
        self._torque_limit_spin.setSuffix(" %")
        self._torque_limit_spin.setDecimals(0)
        self._torque_limit_spin.valueChanged.connect(self._on_torque_limit_changed)
        tl_layout.addWidget(self._torque_limit_spin)

        self._torque_limit_widget.setVisible(False)
        layout.addWidget(self._torque_limit_widget)

        # Enable section
        layout.addWidget(self._create_section_label("Motor Control"))

        enable_layout = QHBoxLayout()
        self._enable_led = QLabel()
        self._enable_led.setObjectName("ledIndicator")
        self._enable_led.setProperty("state", "on")
        self._enable_led.setFixedSize(16, 16)
        self._enable_led.setStyleSheet("""
            QLabel {
                background-color: #4caf50;
                border-radius: 8px;
            }
        """)
        enable_layout.addWidget(self._enable_led)

        self._enable_btn = QPushButton("Enabled")
        self._enable_btn.setCheckable(True)
        self._enable_btn.setChecked(True)
        self._enable_btn.clicked.connect(self._on_enable_clicked)
        enable_layout.addWidget(self._enable_btn, 1)

        layout.addLayout(enable_layout)

        # Zero controls
        layout.addWidget(self._create_section_label("Zero Position"))

        self._set_zero_btn = QPushButton("Set Zero")
        self._set_zero_btn.setToolTip("Set current position as zero for all motors")
        self._set_zero_btn.clicked.connect(self.set_zero_requested)
        layout.addWidget(self._set_zero_btn)

        self._go_zero_btn = QPushButton("Go to Zero")
        self._go_zero_btn.setToolTip("Move all motors to zero position")
        self._go_zero_btn.clicked.connect(self.go_to_zero_requested)
        layout.addWidget(self._go_zero_btn)

        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)
        layout.addWidget(separator)

        # E-Stop button
        layout.addSpacing(8)
        self._estop_btn = QPushButton("EMERGENCY STOP")
        self._estop_btn.setObjectName("estopButton")
        self._estop_btn.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: white;
                font-weight: bold;
                font-size: 14px;
                border-radius: 8px;
                padding: 12px;
                min-height: 50px;
            }
            QPushButton:hover {
                background-color: #f44336;
            }
            QPushButton:pressed {
                background-color: #b71c1c;
            }
        """)
        self._estop_btn.clicked.connect(self.emergency_stop_requested)
        layout.addWidget(self._estop_btn)

        # Spacer
        layout.addStretch()

        # Initial CAN refresh
        self._refresh_can_interfaces()

    def _create_section_label(self, text: str) -> QLabel:
        """Create a section header label."""
        label = QLabel(text)
        label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                font-size: 11px;
                color: #90caf9;
                padding-top: 8px;
            }
        """)
        return label

    def _refresh_can_interfaces(self):
        """Refresh the list of available CAN interfaces."""
        current = self._can_combo.currentText()
        self._can_combo.clear()

        interfaces = CANScanner.get_available_interfaces()
        if interfaces:
            self._can_combo.addItems(interfaces)
            # Try to restore previous selection
            if current and current in interfaces:
                self._can_combo.setCurrentText(current)
        else:
            self._can_combo.addItem("(no CAN interfaces)")

    def _on_mode_changed(self, mode: str):
        """Handle mode combo change."""
        self._torque_limit_widget.setVisible(mode == "force_position")
        if mode != self._current_mode:
            self._current_mode = mode
            if mode == "force_position":
                mode = f"force_position:{self._torque_limit_spin.value()}"
            self.mode_requested.emit(mode)

    def _on_torque_limit_changed(self, value: float):
        """Handle torque limit spinbox change."""
        if self._current_mode == "force_position":
            self.mode_requested.emit(f"force_position:{value}")

    def _on_enable_clicked(self):
        """Handle enable button click."""
        self._enabled = self._enable_btn.isChecked()
        self._update_enable_ui()
        self.enable_requested.emit(self._enabled)

    def _update_enable_ui(self):
        """Update enable button and LED appearance."""
        if self._enabled:
            self._enable_btn.setText("Enabled")
            self._enable_led.setStyleSheet("""
                QLabel {
                    background-color: #4caf50;
                    border-radius: 8px;
                }
            """)
        else:
            self._enable_btn.setText("Disabled")
            self._enable_led.setStyleSheet("""
                QLabel {
                    background-color: #616161;
                    border-radius: 8px;
                }
            """)

    # === Public methods ===

    def set_mode(self, mode: str):
        """Set the current mode (from external source)."""
        # Handle force_position with torque limit
        if mode.startswith("force_position"):
            if ":" in mode:
                try:
                    limit = float(mode.split(":")[1])
                    self._torque_limit_spin.blockSignals(True)
                    self._torque_limit_spin.setValue(limit)
                    self._torque_limit_spin.blockSignals(False)
                except (ValueError, IndexError):
                    pass
            mode = "force_position"

        if mode in self.MODES:
            self._current_mode = mode
            # Block signals to avoid feedback loop
            self._mode_combo.blockSignals(True)
            self._mode_combo.setCurrentText(mode)
            self._mode_combo.blockSignals(False)
            self._torque_limit_widget.setVisible(mode == "force_position")

            # Update enable state based on mode
            if mode == "disabled":
                self._enabled = False
                self._enable_btn.setChecked(False)
            else:
                self._enabled = True
                self._enable_btn.setChecked(True)
            self._update_enable_ui()

    def set_enabled(self, enabled: bool):
        """Set enabled state (from external source)."""
        self._enabled = enabled
        self._enable_btn.blockSignals(True)
        self._enable_btn.setChecked(enabled)
        self._enable_btn.blockSignals(False)
        self._update_enable_ui()

    def get_can_interface(self) -> str:
        """Get selected CAN interface."""
        return self._can_combo.currentText()

    def set_can_interfaces(self, interfaces: List[str], current: str = ""):
        """Set available CAN interfaces."""
        self._can_combo.clear()
        self._can_combo.addItems(interfaces)
        if current and current in interfaces:
            self._can_combo.setCurrentText(current)

    def set_connection_status(self, connected: bool):
        """Update connection status indicator."""
        if connected:
            self._estop_btn.setEnabled(True)
            self._set_zero_btn.setEnabled(True)
            self._go_zero_btn.setEnabled(True)
        else:
            # Keep e-stop always enabled for safety
            self._estop_btn.setEnabled(True)
