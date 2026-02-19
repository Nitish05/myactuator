"""
Trigger configuration dialog for Motor Recording Studio.

Simple dialog for configuring torque triggers that activate when
joint position falls below a threshold.
"""

from typing import Callable, Dict, List, Optional

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel,
    QComboBox, QDoubleSpinBox, QPushButton, QGroupBox, QLineEdit,
    QMessageBox
)

from myactuator_python_driver.config import HysteresisTorqueTrigger


class TriggerDialog(QDialog):
    """
    Simple dialog for configuring a torque trigger.

    The trigger activates when joint position falls below the set threshold.
    User moves joint to desired position and clicks "Capture Position".
    """

    def __init__(self, joint_names: List[str],
                 recording_names: List[str],
                 parent=None,
                 existing_trigger: Optional[HysteresisTorqueTrigger] = None,
                 position_callback: Optional[Callable[[], Dict[str, float]]] = None):
        super().__init__(parent)
        self._joint_names = joint_names
        self._recording_names = recording_names
        self._position_callback = position_callback
        self._joint_positions: Dict[str, float] = {}
        self._existing = existing_trigger
        self._result: Optional[HysteresisTorqueTrigger] = None
        self._threshold_set = False

        self.setWindowTitle("Add Torque Trigger" if not existing_trigger else "Edit Trigger")
        self.setMinimumWidth(400)
        self._setup_ui()

        if existing_trigger:
            self._load_existing(existing_trigger)

        # Timer for live position updates
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._refresh_positions)
        self._update_timer.start(50)  # 20Hz updates

    def _setup_ui(self):
        """Set up the UI components."""
        layout = QVBoxLayout(self)
        layout.setSpacing(16)

        # Info
        info_label = QLabel(
            "Create a trigger that applies constant torque when the joint "
            "position falls below a threshold during playback."
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #90caf9;")
        layout.addWidget(info_label)

        # Name and Recording
        name_group = QGroupBox("Trigger Settings")
        name_layout = QFormLayout(name_group)

        self._name_edit = QLineEdit()
        self._name_edit.setPlaceholderText("e.g., Grasp trigger")
        name_layout.addRow("Name:", self._name_edit)

        self._recording_combo = QComboBox()
        self._recording_combo.addItem("(Any recording)")
        self._recording_combo.addItems(self._recording_names)
        name_layout.addRow("Recording:", self._recording_combo)

        layout.addWidget(name_group)

        # Joint and Position
        joint_group = QGroupBox("Joint & Threshold")
        joint_layout = QFormLayout(joint_group)

        self._joint_combo = QComboBox()
        self._joint_combo.addItems(self._joint_names)
        self._joint_combo.currentTextChanged.connect(self._on_joint_changed)
        joint_layout.addRow("Joint:", self._joint_combo)

        # Current position display
        self._position_label = QLabel("--")
        self._position_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #4caf50;")
        joint_layout.addRow("Current Position:", self._position_label)

        # Capture button
        self._capture_btn = QPushButton("📍 Capture Current Position as Threshold")
        self._capture_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                font-weight: bold;
                padding: 12px;
                border-radius: 6px;
            }
            QPushButton:hover {
                background-color: #2196f3;
            }
        """)
        self._capture_btn.clicked.connect(self._capture_position)
        joint_layout.addRow(self._capture_btn)

        # Threshold display
        self._threshold_label = QLabel("Not set")
        self._threshold_label.setStyleSheet("font-size: 14px; color: #ff9800;")
        joint_layout.addRow("Threshold:", self._threshold_label)

        self._threshold_value = 0.0

        layout.addWidget(joint_group)

        # Torque
        torque_group = QGroupBox("Torque to Apply")
        torque_layout = QFormLayout(torque_group)

        self._torque_spin = QDoubleSpinBox()
        self._torque_spin.setRange(-50.0, 50.0)
        self._torque_spin.setDecimals(2)
        self._torque_spin.setSingleStep(0.5)
        self._torque_spin.setSuffix(" Nm")
        self._torque_spin.setValue(1.0)
        torque_layout.addRow("Torque:", self._torque_spin)

        torque_help = QLabel("Constant torque applied when position falls below threshold")
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

    def _refresh_positions(self):
        """Refresh positions from callback."""
        if self._position_callback:
            self._joint_positions = self._position_callback()
        self._update_position_display()

    def _update_position_display(self):
        """Update the current position display."""
        joint = self._joint_combo.currentText()
        if joint in self._joint_positions:
            pos = self._joint_positions[joint]
            self._position_label.setText(f"{pos:.4f} rad")
        else:
            self._position_label.setText("--")

    def _on_joint_changed(self):
        """Handle joint selection change."""
        self._update_position_display()
        # Reset threshold when joint changes
        if not self._existing:
            self._threshold_set = False
            self._threshold_label.setText("Not set")
            self._threshold_label.setStyleSheet("font-size: 14px; color: #ff9800;")

    def _capture_position(self):
        """Capture current position as threshold."""
        joint = self._joint_combo.currentText()
        if joint in self._joint_positions:
            self._threshold_value = self._joint_positions[joint]
            self._threshold_set = True
            self._threshold_label.setText(f"{self._threshold_value:.4f} rad")
            self._threshold_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #4caf50;")

    def _load_existing(self, trigger: HysteresisTorqueTrigger):
        """Load an existing trigger for editing."""
        self._name_edit.setText(trigger.name)
        if trigger.recording_name:
            idx = self._recording_combo.findText(trigger.recording_name)
            if idx >= 0:
                self._recording_combo.setCurrentIndex(idx)
        if trigger.joint_name in self._joint_names:
            self._joint_combo.setCurrentText(trigger.joint_name)
        self._threshold_value = trigger.enter_threshold_rad
        self._threshold_set = True
        self._threshold_label.setText(f"{self._threshold_value:.4f} rad")
        self._threshold_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #4caf50;")
        self._torque_spin.setValue(trigger.torque_nm)
        self._ok_btn.setText("Update Trigger")

    def _on_ok(self):
        """Handle OK button click."""
        name = self._name_edit.text().strip()
        joint = self._joint_combo.currentText()
        torque = self._torque_spin.value()
        recording = self._recording_combo.currentText()
        if recording == "(Any recording)":
            recording = ""

        # Validate
        if not name:
            QMessageBox.warning(self, "Error", "Please enter a trigger name.")
            return

        if not joint:
            QMessageBox.warning(self, "Error", "Please select a joint.")
            return

        if not self._threshold_set:
            QMessageBox.warning(
                self, "Error",
                "Please capture a position threshold.\n"
                "Move the joint to the desired position and click 'Capture'."
            )
            return

        # Create trigger (always falling direction)
        self._result = HysteresisTorqueTrigger.create_falling(
            name=name,
            joint_name=joint,
            threshold_rad=self._threshold_value,
            torque_nm=torque,
            recording_name=recording,
        )
        self.accept()

    def get_trigger(self) -> Optional[HysteresisTorqueTrigger]:
        """Get the configured trigger (after dialog accepts)."""
        return self._result

    @staticmethod
    def create_trigger(joint_names: List[str],
                       recording_names: List[str],
                       parent=None,
                       position_callback: Optional[Callable[[], Dict[str, float]]] = None
                       ) -> Optional[HysteresisTorqueTrigger]:
        """Show dialog and return new trigger, or None if cancelled."""
        dialog = TriggerDialog(joint_names, recording_names, parent,
                               position_callback=position_callback)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            return dialog.get_trigger()
        return None

    @staticmethod
    def edit_trigger(trigger: HysteresisTorqueTrigger,
                     joint_names: List[str],
                     recording_names: List[str],
                     parent=None,
                     position_callback: Optional[Callable[[], Dict[str, float]]] = None
                     ) -> Optional[HysteresisTorqueTrigger]:
        """Show dialog to edit existing trigger, or None if cancelled."""
        dialog = TriggerDialog(joint_names, recording_names, parent, trigger,
                               position_callback=position_callback)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            return dialog.get_trigger()
        return None
