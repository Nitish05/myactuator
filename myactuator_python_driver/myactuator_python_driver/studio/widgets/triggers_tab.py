"""
Triggers management tab for Motor Recording Studio.

Lists all saved triggers with inline editing of force, threshold, and lock joint.
"""

from typing import Callable, Dict, List, Optional

from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QFrame, QGroupBox, QMessageBox
)

from myactuator_python_driver.config import HysteresisTorqueTrigger


class TriggerCard(QFrame):
    """Editable card for a single trigger."""

    updated = Signal(object, object)  # (old_trigger, new_trigger)
    removed = Signal(object)  # trigger

    def __init__(self, trigger: HysteresisTorqueTrigger,
                 position_callback: Optional[Callable[[], Dict[str, float]]] = None,
                 joint_names: Optional[List[str]] = None):
        super().__init__()
        self._trigger = trigger
        self._position_callback = position_callback
        self._joint_names = joint_names or []
        self.setFrameStyle(QFrame.Shape.StyledPanel | QFrame.Shadow.Raised)
        self.setStyleSheet("""
            TriggerCard {
                background-color: #263238;
                border: 1px solid #37474f;
                border-radius: 8px;
                padding: 8px;
            }
        """)
        self._setup_ui()

    def _get_lock_joint(self) -> str:
        """Get the joint that is NOT the trigger joint."""
        for j in self._joint_names:
            if j != self._trigger.joint_name:
                return j
        return ""

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # Header: name + recording + delete
        header = QHBoxLayout()
        rec = self._trigger.recording_name or "Any"
        self._title = QLabel(f"<b>{self._trigger.name}</b>  →  {rec}")
        self._title.setStyleSheet("font-size: 14px; color: #e0e0e0;")
        header.addWidget(self._title)
        header.addStretch()

        delete_btn = QPushButton("Delete")
        delete_btn.setFixedWidth(70)
        delete_btn.setStyleSheet("""
            QPushButton {
                background-color: #c62828; color: white;
                font-size: 11px; font-weight: bold;
                padding: 4px 8px; border-radius: 4px;
            }
            QPushButton:hover { background-color: #e53935; }
        """)
        delete_btn.clicked.connect(lambda: self.removed.emit(self._trigger))
        header.addWidget(delete_btn)
        layout.addLayout(header)

        # --- Threshold row ---
        thresh_row = QHBoxLayout()
        thresh_row.setSpacing(8)
        self._thresh_label = QLabel(
            f"Threshold: {self._trigger.enter_threshold_rad:.4f} rad"
        )
        self._thresh_label.setStyleSheet("font-size: 13px; color: #ff9800;")
        thresh_row.addWidget(self._thresh_label)
        thresh_row.addStretch()

        self._pos_label = QLabel("--")
        self._pos_label.setStyleSheet("font-size: 12px; color: #4caf50;")
        self._pos_label.setFixedWidth(120)
        thresh_row.addWidget(self._pos_label)

        capture_btn = QPushButton("Capture")
        capture_btn.setFixedWidth(80)
        capture_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2; color: white;
                font-size: 12px; font-weight: bold;
                padding: 6px 8px; border-radius: 4px;
            }
            QPushButton:hover { background-color: #2196f3; }
        """)
        capture_btn.clicked.connect(self._capture_threshold)
        thresh_row.addWidget(capture_btn)
        layout.addLayout(thresh_row)

        # --- Force row ---
        force_row = QHBoxLayout()
        force_row.setSpacing(8)

        force_label = QLabel("Force:")
        force_label.setStyleSheet("font-size: 13px; color: #bdbdbd;")
        force_row.addWidget(force_label)

        minus_btn = QPushButton("-")
        minus_btn.setFixedSize(40, 40)
        minus_btn.setStyleSheet("""
            QPushButton {
                font-size: 20px; font-weight: bold;
                background-color: #c62828; color: white;
                border-radius: 6px;
            }
            QPushButton:hover { background-color: #e53935; }
        """)
        minus_btn.setAutoRepeat(True)
        minus_btn.setAutoRepeatDelay(400)
        minus_btn.setAutoRepeatInterval(100)
        minus_btn.clicked.connect(lambda: self._adjust_force(-0.5))
        force_row.addWidget(minus_btn)

        self._force_label = QLabel(f"{abs(self._trigger.torque_nm):.1f} Nm")
        self._force_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._force_label.setFixedWidth(80)
        self._force_label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: #4caf50;"
        )
        force_row.addWidget(self._force_label)

        plus_btn = QPushButton("+")
        plus_btn.setFixedSize(40, 40)
        plus_btn.setStyleSheet("""
            QPushButton {
                font-size: 20px; font-weight: bold;
                background-color: #2e7d32; color: white;
                border-radius: 6px;
            }
            QPushButton:hover { background-color: #43a047; }
        """)
        plus_btn.setAutoRepeat(True)
        plus_btn.setAutoRepeatDelay(400)
        plus_btn.setAutoRepeatInterval(100)
        plus_btn.clicked.connect(lambda: self._adjust_force(0.5))
        force_row.addWidget(plus_btn)

        force_row.addStretch()
        layout.addLayout(force_row)

        # --- Lock row ---
        lock_row = QHBoxLayout()
        lock_row.setSpacing(8)

        lock_joint = self._get_lock_joint()
        if self._trigger.lock_joint_name:
            lock_text = (f"Lock: {self._trigger.lock_joint_name} "
                         f"@ {self._trigger.lock_position_rad:.4f} rad")
        else:
            lock_text = "Lock: Not set"

        self._lock_label = QLabel(lock_text)
        self._lock_label.setStyleSheet("font-size: 13px; color: #e65100;" if self._trigger.lock_joint_name else "font-size: 13px; color: #757575;")
        lock_row.addWidget(self._lock_label)
        lock_row.addStretch()

        if lock_joint:
            btn_text = f"Unlock {lock_joint}" if (self._trigger.lock_joint_name == lock_joint) else f"Lock {lock_joint}"
            lock_btn = QPushButton(btn_text)
            lock_btn.setFixedWidth(110)
            lock_btn.setStyleSheet("""
                QPushButton {
                    background-color: #37474f; color: white;
                    font-size: 12px; font-weight: bold;
                    padding: 6px 8px; border-radius: 4px;
                }
                QPushButton:hover { background-color: #455a64; }
            """)
            lock_btn.clicked.connect(lambda: self._toggle_lock(lock_joint, lock_btn))
            lock_row.addWidget(lock_btn)

        layout.addLayout(lock_row)

    def update_position(self, positions: Dict[str, float]):
        """Update live position display."""
        joint = self._trigger.joint_name
        if joint in positions:
            self._pos_label.setText(f"now: {positions[joint]:.4f} rad")

    def _capture_threshold(self):
        """Capture current position as new threshold."""
        if not self._position_callback:
            return
        positions = self._position_callback()
        joint = self._trigger.joint_name
        if joint not in positions:
            return

        old = self._trigger
        new_threshold = positions[joint]
        new_trigger = HysteresisTorqueTrigger.create_falling(
            name=old.name,
            joint_name=old.joint_name,
            threshold_rad=new_threshold,
            torque_nm=old.torque_nm,
            recording_name=old.recording_name,
            lock_joint_name=old.lock_joint_name,
            lock_position_rad=old.lock_position_rad,
        )
        self._trigger = new_trigger
        self._thresh_label.setText(f"Threshold: {new_threshold:.4f} rad")
        self._thresh_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #4caf50;")
        self.updated.emit(old, new_trigger)

    def _adjust_force(self, delta: float):
        """Adjust force by delta."""
        old = self._trigger
        current = abs(old.torque_nm)
        new_val = max(0.5, min(20.0, current + delta))
        new_trigger = HysteresisTorqueTrigger.create_falling(
            name=old.name,
            joint_name=old.joint_name,
            threshold_rad=old.enter_threshold_rad,
            torque_nm=-new_val,
            recording_name=old.recording_name,
            lock_joint_name=old.lock_joint_name,
            lock_position_rad=old.lock_position_rad,
        )
        self._trigger = new_trigger
        self._force_label.setText(f"{new_val:.1f} Nm")
        self.updated.emit(old, new_trigger)

    def _toggle_lock(self, lock_joint: str, btn: QPushButton):
        """Toggle lock on/off for the other joint."""
        old = self._trigger
        if old.lock_joint_name == lock_joint:
            # Clear lock
            new_trigger = HysteresisTorqueTrigger.create_falling(
                name=old.name,
                joint_name=old.joint_name,
                threshold_rad=old.enter_threshold_rad,
                torque_nm=old.torque_nm,
                recording_name=old.recording_name,
                lock_joint_name="",
                lock_position_rad=0.0,
            )
            self._trigger = new_trigger
            self._lock_label.setText("Lock: Not set")
            self._lock_label.setStyleSheet("font-size: 13px; color: #757575;")
            btn.setText(f"Lock {lock_joint}")
        else:
            # Capture lock at current position
            if not self._position_callback:
                return
            positions = self._position_callback()
            pos = positions.get(lock_joint)
            if pos is None:
                return
            new_trigger = HysteresisTorqueTrigger.create_falling(
                name=old.name,
                joint_name=old.joint_name,
                threshold_rad=old.enter_threshold_rad,
                torque_nm=old.torque_nm,
                recording_name=old.recording_name,
                lock_joint_name=lock_joint,
                lock_position_rad=pos,
            )
            self._trigger = new_trigger
            self._lock_label.setText(f"Lock: {lock_joint} @ {pos:.4f} rad")
            self._lock_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e65100;")
            btn.setText(f"Unlock {lock_joint}")
        self.updated.emit(old, new_trigger)


class TriggersTab(QWidget):
    """
    Triggers management panel.

    Shows all saved triggers as editable cards. Changes are persisted
    immediately via signals back to MainWindow.
    """

    trigger_updated = Signal(object, object)  # (old_trigger, new_trigger)
    trigger_removed = Signal(object)  # trigger

    def __init__(self):
        super().__init__()
        self._triggers: List[HysteresisTorqueTrigger] = []
        self._cards: List[TriggerCard] = []
        self._position_callback = None
        self._joint_names: List[str] = []
        self._setup_ui()

        # Timer for live position updates on cards (started/stopped via show/hide)
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._refresh_positions)

    def showEvent(self, event):
        super().showEvent(event)
        self._update_timer.start(50)

    def hideEvent(self, event):
        self._update_timer.stop()
        super().hideEvent(event)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        # Header
        header = QHBoxLayout()
        title = QLabel("Triggers")
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        header.addWidget(title)
        header.addStretch()
        layout.addLayout(header)

        # Scrollable card list
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._scroll.setStyleSheet("QScrollArea { border: none; }")

        self._card_container = QWidget()
        self._card_layout = QVBoxLayout(self._card_container)
        self._card_layout.setSpacing(10)
        self._card_layout.setContentsMargins(0, 0, 0, 0)
        self._card_layout.addStretch()

        self._scroll.setWidget(self._card_container)
        layout.addWidget(self._scroll)

        # Empty state
        self._empty_label = QLabel("No triggers configured.\n"
                                   "Record a trajectory and add a trigger,\n"
                                   "or use Playback → Add Trigger.")
        self._empty_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._empty_label.setStyleSheet("color: #757575; font-size: 13px; padding: 40px;")
        self._card_layout.insertWidget(0, self._empty_label)

    def set_position_callback(self, callback, joint_names: List[str]):
        """Set the callback for reading live joint positions."""
        self._position_callback = callback
        self._joint_names = joint_names

    def set_triggers(self, triggers: List[HysteresisTorqueTrigger]):
        """Replace all triggers and rebuild cards."""
        self._triggers = list(triggers)
        self._rebuild_cards()

    def _rebuild_cards(self):
        """Rebuild all trigger cards from current list."""
        # Remove old cards
        for card in self._cards:
            self._card_layout.removeWidget(card)
            card.deleteLater()
        self._cards.clear()

        self._empty_label.setVisible(len(self._triggers) == 0)

        for trigger in self._triggers:
            card = TriggerCard(
                trigger,
                position_callback=self._position_callback,
                joint_names=self._joint_names,
            )
            card.updated.connect(self._on_card_updated)
            card.removed.connect(self._on_card_removed)
            # Insert before the stretch
            self._card_layout.insertWidget(self._card_layout.count() - 1, card)
            self._cards.append(card)

    def _on_card_updated(self, old_trigger, new_trigger):
        """Handle a card editing a trigger."""
        # Update local list
        for i, t in enumerate(self._triggers):
            if (t.name == old_trigger.name and t.joint_name == old_trigger.joint_name
                    and t.recording_name == old_trigger.recording_name):
                self._triggers[i] = new_trigger
                break
        self.trigger_updated.emit(old_trigger, new_trigger)

    def _on_card_removed(self, trigger):
        """Handle a card removing a trigger."""
        self._triggers = [t for t in self._triggers if not (
            t.name == trigger.name and t.joint_name == trigger.joint_name
            and t.recording_name == trigger.recording_name
        )]
        self.trigger_removed.emit(trigger)
        self._rebuild_cards()

    def _refresh_positions(self):
        """Push live positions to all cards."""
        if not self._position_callback or not self._cards:
            return
        if not self.isVisible():
            return
        positions = self._position_callback()
        for card in self._cards:
            card.update_position(positions)
