"""
Trigger configuration dialog for Motor Recording Studio.

Modal dialog for configuring hysteresis torque triggers.
"""

from typing import Callable, Dict, List, Optional

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel,
    QComboBox, QDoubleSpinBox, QPushButton, QGroupBox, QMessageBox
)

from myactuator_python_driver.config import HysteresisTorqueTrigger


class TriggerDialog(QDialog):
    """
    Dialog for configuring a hysteresis torque trigger.

    Allows the user to:
    - Select a joint
    - Set enter/exit thresholds
    - Set torque value
    - See current position for reference
    """

    def __init__(self, joint_names: List[str], joint_positions: Dict[str, float],
                 parent=None, existing_trigger: Optional[HysteresisTorqueTrigger] = None,
                 position_callback: Optional[Callable[[], Dict[str, float]]] = None):
        super().__init__(parent)
        self._joint_names = joint_names
        self._joint_positions = joint_positions
        self._position_callback = position_callback
        self._existing = existing_trigger
        self._result: Optional[HysteresisTorqueTrigger] = None

        self.setWindowTitle("Configure Torque Trigger")
        self.setMinimumWidth(400)
        self._setup_ui()

        if existing_trigger:
            self._load_existing(existing_trigger)

        # Timer for live position updates
        if position_callback:
            self._update_timer = QTimer(self)
            self._update_timer.timeout.connect(self._refresh_positions)
            self._update_timer.start(50)  # 20Hz updates

    def _setup_ui(self):
        """Set up the UI components."""
        layout = QVBoxLayout(self)
        layout.setSpacing(16)

        # Info
        info_label = QLabel(
            "Configure a torque trigger for hybrid playback.\n"
            "When the joint position crosses the enter threshold, "
            "the motor switches to constant torque mode.\n"
            "When it crosses the exit threshold (with hysteresis), "
            "it returns to position control."
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #757575;")
        layout.addWidget(info_label)

        # Joint selection
        joint_group = QGroupBox("Joint Selection")
        joint_layout = QFormLayout(joint_group)

        self._joint_combo = QComboBox()
        self._joint_combo.addItems(self._joint_names)
        self._joint_combo.currentTextChanged.connect(self._update_position_display)
        joint_layout.addRow("Joint:", self._joint_combo)

        self._position_label = QLabel("--")
        self._position_label.setStyleSheet("font-weight: bold; color: #90caf9;")
        joint_layout.addRow("Current Position:", self._position_label)

        layout.addWidget(joint_group)

        # Thresholds
        threshold_group = QGroupBox("Thresholds")
        threshold_layout = QFormLayout(threshold_group)

        enter_layout = QHBoxLayout()
        self._enter_spin = QDoubleSpinBox()
        self._enter_spin.setRange(-100.0, 100.0)
        self._enter_spin.setDecimals(3)
        self._enter_spin.setSingleStep(0.1)
        self._enter_spin.setSuffix(" rad")
        self._enter_spin.valueChanged.connect(self._update_direction)
        enter_layout.addWidget(self._enter_spin)
        self._enter_use_btn = QPushButton("Use Current")
        self._enter_use_btn.setMaximumWidth(90)
        self._enter_use_btn.clicked.connect(self._use_current_for_enter)
        enter_layout.addWidget(self._enter_use_btn)
        threshold_layout.addRow("Enter Threshold:", enter_layout)

        enter_help = QLabel("Position at which to switch TO torque mode")
        enter_help.setStyleSheet("color: #757575; font-size: 10px;")
        threshold_layout.addRow("", enter_help)

        exit_layout = QHBoxLayout()
        self._exit_spin = QDoubleSpinBox()
        self._exit_spin.setRange(-100.0, 100.0)
        self._exit_spin.setDecimals(3)
        self._exit_spin.setSingleStep(0.1)
        self._exit_spin.setSuffix(" rad")
        self._exit_spin.valueChanged.connect(self._update_direction)
        exit_layout.addWidget(self._exit_spin)
        self._exit_use_btn = QPushButton("Use Current")
        self._exit_use_btn.setMaximumWidth(90)
        self._exit_use_btn.clicked.connect(self._use_current_for_exit)
        exit_layout.addWidget(self._exit_use_btn)
        threshold_layout.addRow("Exit Threshold:", exit_layout)

        exit_help = QLabel("Position at which to switch BACK to position mode")
        exit_help.setStyleSheet("color: #757575; font-size: 10px;")
        threshold_layout.addRow("", exit_help)

        self._direction_label = QLabel("Direction: --")
        self._direction_label.setStyleSheet("font-weight: bold;")
        threshold_layout.addRow("", self._direction_label)

        layout.addWidget(threshold_group)

        # Torque
        torque_group = QGroupBox("Torque Setting")
        torque_layout = QFormLayout(torque_group)

        self._torque_spin = QDoubleSpinBox()
        self._torque_spin.setRange(-50.0, 50.0)
        self._torque_spin.setDecimals(2)
        self._torque_spin.setSingleStep(0.5)
        self._torque_spin.setSuffix(" Nm")
        self._torque_spin.setValue(1.0)
        torque_layout.addRow("Torque to Apply:", self._torque_spin)

        torque_help = QLabel("Constant torque applied when trigger is active")
        torque_help.setStyleSheet("color: #757575; font-size: 10px;")
        torque_layout.addRow("", torque_help)

        layout.addWidget(torque_group)

        # Buttons
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()

        self._cancel_btn = QPushButton("Cancel")
        self._cancel_btn.clicked.connect(self.reject)
        btn_layout.addWidget(self._cancel_btn)

        self._ok_btn = QPushButton("Add Trigger")
        self._ok_btn.setDefault(True)
        self._ok_btn.clicked.connect(self._on_ok)
        btn_layout.addWidget(self._ok_btn)

        layout.addLayout(btn_layout)

        # Initial update
        self._update_position_display()

    def _refresh_positions(self):
        """Refresh positions from callback and update display."""
        if self._position_callback:
            self._joint_positions = self._position_callback()
        self._update_position_display()

    def _update_position_display(self):
        """Update the current position display for selected joint."""
        joint = self._joint_combo.currentText()
        if joint in self._joint_positions:
            pos = self._joint_positions[joint]
            self._position_label.setText(f"{pos:.4f} rad")
        else:
            self._position_label.setText("--")

    def _use_current_for_enter(self):
        """Set enter threshold to current joint position."""
        joint = self._joint_combo.currentText()
        if joint in self._joint_positions:
            self._enter_spin.setValue(self._joint_positions[joint])

    def _use_current_for_exit(self):
        """Set exit threshold to current joint position."""
        joint = self._joint_combo.currentText()
        if joint in self._joint_positions:
            self._exit_spin.setValue(self._joint_positions[joint])

    def _update_direction(self):
        """Update the direction indicator based on thresholds."""
        enter = self._enter_spin.value()
        exit_val = self._exit_spin.value()

        if enter > exit_val:
            self._direction_label.setText("Direction: Rising (torque when angle > enter)")
            self._direction_label.setStyleSheet("font-weight: bold; color: #4caf50;")
        elif enter < exit_val:
            self._direction_label.setText("Direction: Falling (torque when angle < enter)")
            self._direction_label.setStyleSheet("font-weight: bold; color: #2196f3;")
        else:
            self._direction_label.setText("Direction: Invalid (enter must differ from exit)")
            self._direction_label.setStyleSheet("font-weight: bold; color: #f44336;")

    def _load_existing(self, trigger: HysteresisTorqueTrigger):
        """Load an existing trigger for editing."""
        if trigger.joint_name in self._joint_names:
            self._joint_combo.setCurrentText(trigger.joint_name)
        self._enter_spin.setValue(trigger.enter_threshold_rad)
        self._exit_spin.setValue(trigger.exit_threshold_rad)
        self._torque_spin.setValue(trigger.torque_nm)
        self._ok_btn.setText("Update Trigger")

    def _on_ok(self):
        """Handle OK button click."""
        joint = self._joint_combo.currentText()
        enter = self._enter_spin.value()
        exit_val = self._exit_spin.value()
        torque = self._torque_spin.value()

        # Validate
        if not joint:
            QMessageBox.warning(self, "Error", "Please select a joint.")
            return

        if enter == exit_val:
            QMessageBox.warning(
                self, "Error",
                "Enter and exit thresholds must be different for hysteresis."
            )
            return

        # Determine direction
        direction = "rising" if enter > exit_val else "falling"

        try:
            self._result = HysteresisTorqueTrigger(
                joint_name=joint,
                enter_threshold_rad=enter,
                exit_threshold_rad=exit_val,
                torque_nm=torque,
                direction=direction
            )
            self.accept()
        except ValueError as e:
            QMessageBox.warning(self, "Validation Error", str(e))

    def get_trigger(self) -> Optional[HysteresisTorqueTrigger]:
        """Get the configured trigger (after dialog accepts)."""
        return self._result

    @staticmethod
    def create_trigger(joint_names: List[str], joint_positions: Dict[str, float],
                       parent=None,
                       position_callback: Optional[Callable[[], Dict[str, float]]] = None
                       ) -> Optional[HysteresisTorqueTrigger]:
        """
        Static convenience method to show dialog and return trigger.

        Args:
            joint_names: List of joint names
            joint_positions: Initial dict of joint positions
            parent: Parent widget
            position_callback: Optional callback to get live positions (called at 20Hz)

        Returns the trigger if user clicked OK, None if cancelled.
        """
        dialog = TriggerDialog(joint_names, joint_positions, parent,
                               position_callback=position_callback)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            return dialog.get_trigger()
        return None

    @staticmethod
    def edit_trigger(trigger: HysteresisTorqueTrigger,
                     joint_names: List[str], joint_positions: Dict[str, float],
                     parent=None,
                     position_callback: Optional[Callable[[], Dict[str, float]]] = None
                     ) -> Optional[HysteresisTorqueTrigger]:
        """
        Static convenience method to edit an existing trigger.

        Args:
            trigger: Existing trigger to edit
            joint_names: List of joint names
            joint_positions: Initial dict of joint positions
            parent: Parent widget
            position_callback: Optional callback to get live positions (called at 20Hz)

        Returns the updated trigger if user clicked OK, None if cancelled.
        """
        dialog = TriggerDialog(joint_names, joint_positions, parent, trigger,
                               position_callback=position_callback)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            return dialog.get_trigger()
        return None
