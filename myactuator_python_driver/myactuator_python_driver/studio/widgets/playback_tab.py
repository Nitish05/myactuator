"""
Playback tab widget for Motor Recording Studio.

Playback controls, speed adjustment, and trigger configuration.
"""

from typing import Dict, List, Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QPushButton, QSlider, QCheckBox, QGroupBox, QListWidget,
    QListWidgetItem, QProgressBar
)

from myactuator_python_driver.config import HysteresisTorqueTrigger, PlaybackTriggerConfig, TriggerStore
from myactuator_python_driver.studio.recording_manager import RecordingInfo


class PlaybackTab(QWidget):
    """
    Playback tab widget for playing back recorded trajectories.

    Contains:
    - Recording selector
    - Play/Pause/Stop controls
    - Progress slider
    - Speed control
    - Loop toggle
    - Torque trigger configuration
    """

    # Signals
    play_requested = pyqtSignal(object)  # RecordingInfo
    stop_requested = pyqtSignal()
    pause_requested = pyqtSignal()
    speed_changed = pyqtSignal(float)
    loop_changed = pyqtSignal(bool)
    triggers_changed = pyqtSignal(object)  # PlaybackTriggerConfig
    configure_triggers_requested = pyqtSignal()
    trigger_removed = pyqtSignal(object)  # HysteresisTorqueTrigger

    SPEEDS = [0.25, 0.5, 1.0, 2.0, 4.0]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._playing = False
        self._paused = False
        self._selected_recording: Optional[RecordingInfo] = None
        self._triggers: List[HysteresisTorqueTrigger] = []
        self._trigger_states: Dict[str, str] = {}
        self._setup_ui()

    def _setup_ui(self):
        """Set up the UI components."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        # Recording selection
        select_group = QGroupBox("Recording")
        select_layout = QVBoxLayout(select_group)

        self._recording_combo = QComboBox()
        self._recording_combo.currentIndexChanged.connect(self._on_recording_changed)
        select_layout.addWidget(self._recording_combo)

        # Recording info
        self._info_label = QLabel("No recording selected")
        self._info_label.setStyleSheet("color: #757575;")
        select_layout.addWidget(self._info_label)

        layout.addWidget(select_group)

        # Playback controls group
        controls_group = QGroupBox("Playback Controls")
        controls_layout = QVBoxLayout(controls_group)

        # Progress bar
        self._progress = QProgressBar()
        self._progress.setRange(0, 1000)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        self._progress.setFormat("0.0s / 0.0s")
        controls_layout.addWidget(self._progress)

        # Play/Pause/Stop buttons
        btn_layout = QHBoxLayout()

        self._play_btn = QPushButton("Play")
        self._play_btn.setObjectName("playButton")
        self._play_btn.setMinimumWidth(80)
        self._play_btn.clicked.connect(self._on_play_clicked)
        btn_layout.addWidget(self._play_btn)

        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setMinimumWidth(80)
        self._pause_btn.setEnabled(False)
        self._pause_btn.clicked.connect(self._on_pause_clicked)
        btn_layout.addWidget(self._pause_btn)

        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setMinimumWidth(80)
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop_clicked)
        btn_layout.addWidget(self._stop_btn)

        btn_layout.addStretch()

        controls_layout.addLayout(btn_layout)

        # Speed and loop
        options_layout = QHBoxLayout()

        options_layout.addWidget(QLabel("Speed:"))
        self._speed_combo = QComboBox()
        for speed in self.SPEEDS:
            self._speed_combo.addItem(f"{speed}x", speed)
        self._speed_combo.setCurrentIndex(2)  # 1.0x
        self._speed_combo.currentIndexChanged.connect(self._on_speed_changed)
        options_layout.addWidget(self._speed_combo)

        options_layout.addSpacing(20)

        self._loop_check = QCheckBox("Loop")
        self._loop_check.stateChanged.connect(self._on_loop_changed)
        options_layout.addWidget(self._loop_check)

        options_layout.addStretch()

        controls_layout.addLayout(options_layout)

        layout.addWidget(controls_group)

        # Trigger configuration group
        trigger_group = QGroupBox("Torque Triggers")
        trigger_layout = QVBoxLayout(trigger_group)

        # Trigger info
        trigger_info = QLabel(
            "Configure per-joint torque overrides for hybrid playback.\n"
            "When position crosses threshold, switches to constant torque."
        )
        trigger_info.setStyleSheet("color: #757575; font-size: 11px;")
        trigger_info.setWordWrap(True)
        trigger_layout.addWidget(trigger_info)

        # Trigger list
        self._trigger_list = QListWidget()
        self._trigger_list.setMaximumHeight(100)
        trigger_layout.addWidget(self._trigger_list)

        # Trigger buttons
        trigger_btn_layout = QHBoxLayout()

        self._add_trigger_btn = QPushButton("Add...")
        self._add_trigger_btn.clicked.connect(self.configure_triggers_requested)
        trigger_btn_layout.addWidget(self._add_trigger_btn)

        self._remove_trigger_btn = QPushButton("Remove")
        self._remove_trigger_btn.clicked.connect(self._remove_selected_trigger)
        trigger_btn_layout.addWidget(self._remove_trigger_btn)

        self._clear_triggers_btn = QPushButton("Clear All")
        self._clear_triggers_btn.clicked.connect(self._clear_triggers)
        trigger_btn_layout.addWidget(self._clear_triggers_btn)

        trigger_btn_layout.addStretch()

        trigger_layout.addLayout(trigger_btn_layout)

        layout.addWidget(trigger_group)

        # Spacer
        layout.addStretch()

    def _on_recording_changed(self, index: int):
        """Handle recording selection change."""
        if index >= 0:
            self._selected_recording = self._recording_combo.itemData(index)
            if self._selected_recording:
                dur = self._selected_recording.duration_sec
                frames = self._selected_recording.frame_count
                joints = ", ".join(self._selected_recording.joint_names[:3])
                if len(self._selected_recording.joint_names) > 3:
                    joints += "..."
                self._info_label.setText(
                    f"Duration: {dur:.1f}s | Frames: {frames} | Joints: {joints}"
                )
            else:
                self._info_label.setText("No recording selected")
        else:
            self._selected_recording = None
            self._info_label.setText("No recording selected")
        # Refresh trigger list to update which triggers apply
        self._update_trigger_list()

    def _on_play_clicked(self):
        """Handle play button click."""
        if self._playing:
            return
        if self._selected_recording:
            self.play_requested.emit(self._selected_recording)

    def _on_pause_clicked(self):
        """Handle pause button click."""
        self.pause_requested.emit()

    def _on_stop_clicked(self):
        """Handle stop button click."""
        self.stop_requested.emit()

    def _on_speed_changed(self, index: int):
        """Handle speed change."""
        speed = self._speed_combo.itemData(index)
        if speed:
            self.speed_changed.emit(speed)

    def _on_loop_changed(self, state: int):
        """Handle loop checkbox change."""
        self.loop_changed.emit(state == Qt.CheckState.Checked.value)

    def _remove_selected_trigger(self):
        """Remove the selected trigger."""
        row = self._trigger_list.currentRow()
        if 0 <= row < len(self._triggers):
            trigger = self._triggers.pop(row)
            self.trigger_removed.emit(trigger)  # Signal for persistent removal
            self._update_trigger_list()
            self._emit_triggers()

    def _clear_triggers(self):
        """Clear all triggers."""
        for trigger in self._triggers:
            self.trigger_removed.emit(trigger)
        self._triggers.clear()
        self._update_trigger_list()
        self._emit_triggers()

    def _update_trigger_list(self):
        """Update the trigger list display."""
        self._trigger_list.clear()
        selected_recording = self._selected_recording.name if self._selected_recording else ""

        for trigger in self._triggers:
            # Check if trigger applies to current recording
            applies = (not trigger.recording_name or
                       trigger.recording_name == selected_recording)

            state = self._trigger_states.get(trigger.joint_name, "inactive")
            state_text = " [ACTIVE]" if state == "active" else ""

            # Show trigger name and details
            name = trigger.name or "(unnamed)"
            recording_info = f" [{trigger.recording_name}]" if trigger.recording_name else ""
            item = QListWidgetItem(
                f"{name}: {trigger.joint_name} < {trigger.enter_threshold_rad:.3f} rad → "
                f"{trigger.torque_nm:.1f} Nm{recording_info}{state_text}"
            )

            if state == "active":
                item.setForeground(Qt.GlobalColor.yellow)
            elif not applies:
                item.setForeground(Qt.GlobalColor.gray)

            self._trigger_list.addItem(item)

    def _emit_triggers(self):
        """Emit the current trigger configuration."""
        config = PlaybackTriggerConfig(triggers=self._triggers)
        self.triggers_changed.emit(config)

    # === Public methods ===

    def set_recordings(self, recordings: List[RecordingInfo]):
        """Set the list of available recordings."""
        current_name = ""
        if self._selected_recording:
            current_name = self._selected_recording.name

        self._recording_combo.clear()

        for recording in recordings:
            self._recording_combo.addItem(recording.name, recording)

        # Restore selection if possible
        for i in range(self._recording_combo.count()):
            if self._recording_combo.itemData(i).name == current_name:
                self._recording_combo.setCurrentIndex(i)
                break

    def set_playing(self, playing: bool):
        """Set playback state."""
        self._playing = playing
        self._paused = False

        if playing:
            self._play_btn.setText("Playing")
            self._play_btn.setEnabled(False)
            self._pause_btn.setEnabled(True)
            self._stop_btn.setEnabled(True)
            self._recording_combo.setEnabled(False)
        else:
            self._play_btn.setText("Play")
            self._play_btn.setEnabled(True)
            self._pause_btn.setEnabled(False)
            self._pause_btn.setText("Pause")
            self._stop_btn.setEnabled(False)
            self._recording_combo.setEnabled(True)
            self._progress.setValue(0)
            self._progress.setFormat("0.0s / 0.0s")

    def set_paused(self, paused: bool):
        """Set paused state."""
        self._paused = paused
        if paused:
            self._pause_btn.setText("Resume")
        else:
            self._pause_btn.setText("Pause")

    def set_progress(self, current_sec: float, total_sec: float):
        """Update playback progress."""
        if total_sec > 0:
            progress = int((current_sec / total_sec) * 1000)
            self._progress.setValue(progress)
        self._progress.setFormat(f"{current_sec:.1f}s / {total_sec:.1f}s")

    def add_trigger(self, trigger: HysteresisTorqueTrigger):
        """Add a trigger."""
        self._triggers.append(trigger)
        self._update_trigger_list()
        self._emit_triggers()

    def set_triggers(self, triggers: List[HysteresisTorqueTrigger]):
        """Set all triggers."""
        self._triggers = list(triggers)
        self._update_trigger_list()

    def get_triggers(self) -> List[HysteresisTorqueTrigger]:
        """Get current triggers."""
        return self._triggers

    def set_trigger_states(self, states: Dict[str, dict]):
        """Update trigger states from driver."""
        self._trigger_states = {
            name: info.get('state', 'inactive')
            for name, info in states.items()
        }
        self._update_trigger_list()

    def get_selected_recording(self) -> Optional[RecordingInfo]:
        """Get the selected recording."""
        return self._selected_recording
