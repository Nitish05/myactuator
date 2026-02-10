"""
Easy Mode widget for Motor Recording Studio.

Dropdown to pick a recording, circular play/pause button, stop button.
"""

from typing import List, Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QProgressBar, QFrame, QSizePolicy
)

from myactuator_python_driver.config import TriggerStore
from myactuator_python_driver.studio.recording_manager import RecordingInfo

# Unicode symbols
_PLAY = "\u25B6"   # ▶
_PAUSE = "\u275A\u275A"  # ❚❚
_STOP = "\u25A0"   # ■


class EasyModeWidget(QWidget):
    """
    Simple mode with a dropdown to select a recording,
    a circular play/pause button, and a stop button.
    """

    recording_play_requested = pyqtSignal(object)  # RecordingInfo
    pause_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    advanced_mode_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._recordings: List[RecordingInfo] = []
        self._playing = False
        self._paused = False
        self._trigger_store: Optional[TriggerStore] = None
        self._setup_ui()

    def _setup_ui(self):
        """Set up the UI."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 16, 24, 16)
        layout.setSpacing(12)

        # Header row: connection LED + title + advanced button
        header = QHBoxLayout()
        header.setSpacing(12)

        self._connection_led = QLabel()
        self._connection_led.setObjectName("easyConnectionLed")
        self._connection_led.setFixedSize(14, 14)
        self._set_led(False)
        header.addWidget(self._connection_led)

        title = QLabel("Motor Studio")
        title.setObjectName("easyTitle")
        title.setStyleSheet("font-size: 22px; font-weight: bold; color: #e0e0e0;")
        header.addWidget(title)

        header.addStretch()

        advanced_btn = QPushButton("Advanced Mode")
        advanced_btn.setObjectName("advancedModeButton")
        advanced_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        advanced_btn.clicked.connect(self.advanced_mode_requested)
        header.addWidget(advanced_btn)

        layout.addLayout(header)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #424242;")
        layout.addWidget(sep)

        # Spacer above centre content
        layout.addStretch()

        # Recording dropdown
        combo_row = QHBoxLayout()
        combo_row.setSpacing(10)

        combo_label = QLabel("Recording:")
        combo_label.setStyleSheet("font-size: 15px; color: #e0e0e0;")
        combo_row.addWidget(combo_label)

        self._recording_combo = QComboBox()
        self._recording_combo.setMinimumHeight(36)
        self._recording_combo.setMinimumWidth(250)
        self._recording_combo.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self._recording_combo.currentIndexChanged.connect(self._on_selection_changed)
        combo_row.addWidget(self._recording_combo)

        layout.addLayout(combo_row)

        # Info label
        self._info_label = QLabel("No recording selected")
        self._info_label.setStyleSheet("color: #9e9e9e; font-size: 12px;")
        self._info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._info_label)

        layout.addSpacing(12)

        # Play/pause + stop buttons (centred)
        btn_row = QHBoxLayout()
        btn_row.setSpacing(20)
        btn_row.addStretch()

        self._play_btn = QPushButton(_PLAY)
        self._play_btn.setObjectName("easyPlayButton")
        self._play_btn.setFixedSize(80, 80)
        self._play_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._play_btn.setEnabled(False)
        self._play_btn.clicked.connect(self._on_play_clicked)
        btn_row.addWidget(self._play_btn)

        self._stop_btn = QPushButton(_STOP)
        self._stop_btn.setObjectName("easyStopPlaybackButton")
        self._stop_btn.setFixedSize(56, 56)
        self._stop_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop_clicked)
        btn_row.addWidget(self._stop_btn, alignment=Qt.AlignmentFlag.AlignVCenter)

        btn_row.addStretch()
        layout.addLayout(btn_row)

        layout.addSpacing(4)

        # Now-playing area (hidden until playing)
        self._playing_widget = QWidget()
        playing_layout = QVBoxLayout(self._playing_widget)
        playing_layout.setContentsMargins(0, 0, 0, 0)
        playing_layout.setSpacing(8)

        self._now_playing_label = QLabel("")
        self._now_playing_label.setObjectName("nowPlayingLabel")
        self._now_playing_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._now_playing_label.setStyleSheet(
            "font-size: 14px; color: #81c784; font-weight: bold;"
        )
        playing_layout.addWidget(self._now_playing_label)

        self._progress_bar = QProgressBar()
        self._progress_bar.setObjectName("easyProgressBar")
        self._progress_bar.setRange(0, 1000)
        self._progress_bar.setValue(0)
        self._progress_bar.setTextVisible(True)
        self._progress_bar.setFormat("0.0s / 0.0s")
        self._progress_bar.setMinimumHeight(28)
        playing_layout.addWidget(self._progress_bar)

        self._playing_widget.setVisible(False)
        layout.addWidget(self._playing_widget)

        # Spacer below centre content
        layout.addStretch()

        # E-Stop at bottom
        estop_btn = QPushButton("EMERGENCY STOP")
        estop_btn.setObjectName("estopButton")
        estop_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        estop_btn.clicked.connect(self.emergency_stop_requested)
        layout.addWidget(estop_btn)

    # === Internal ===

    def _set_led(self, connected: bool):
        """Update connection LED."""
        color = "#4caf50" if connected else "#616161"
        self._connection_led.setStyleSheet(
            f"background-color: {color}; border-radius: 7px;"
        )

    def _on_selection_changed(self, index: int):
        """Update info label when dropdown selection changes."""
        if index < 0 or index >= len(self._recordings):
            self._info_label.setText("No recording selected")
            self._play_btn.setEnabled(False)
            return

        rec = self._recording_combo.itemData(index)
        if not rec:
            self._info_label.setText("No recording selected")
            self._play_btn.setEnabled(False)
            return

        parts = [f"{rec.duration_sec:.1f}s", f"{rec.frame_count} frames"]
        if rec.joint_names:
            joints = ", ".join(rec.joint_names[:3])
            if len(rec.joint_names) > 3:
                joints += "..."
            parts.append(joints)

        if self._trigger_store:
            triggers = self._trigger_store.get_for_recording(rec.name)
            if triggers:
                parts.append("has trigger")

        self._info_label.setText("  |  ".join(parts))
        self._play_btn.setEnabled(not self._playing)

    def _on_play_clicked(self):
        """Handle play/pause button click."""
        if self._playing:
            # Toggle pause
            self.pause_requested.emit()
        else:
            index = self._recording_combo.currentIndex()
            if index >= 0:
                rec = self._recording_combo.itemData(index)
                if rec:
                    self.recording_play_requested.emit(rec)

    def _on_stop_clicked(self):
        """Handle stop button click."""
        self.stop_requested.emit()

    # === Public API ===

    def set_recordings(self, recordings: List[RecordingInfo],
                       trigger_store: Optional[TriggerStore] = None):
        """Update the recordings dropdown."""
        self._recordings = recordings
        self._trigger_store = trigger_store

        current_name = ""
        if self._recording_combo.currentIndex() >= 0:
            current = self._recording_combo.currentData()
            if current:
                current_name = current.name

        self._recording_combo.blockSignals(True)
        self._recording_combo.clear()
        for rec in recordings:
            self._recording_combo.addItem(rec.name, rec)

        # Restore selection
        restored = False
        if current_name:
            for i in range(self._recording_combo.count()):
                if self._recording_combo.itemData(i).name == current_name:
                    self._recording_combo.setCurrentIndex(i)
                    restored = True
                    break
        self._recording_combo.blockSignals(False)

        if restored:
            self._on_selection_changed(self._recording_combo.currentIndex())
        elif recordings:
            self._on_selection_changed(0)
        else:
            self._on_selection_changed(-1)

    def set_connection_status(self, connected: bool):
        """Update connection LED."""
        self._set_led(connected)

    def set_playing(self, playing: bool, name: str = ""):
        """Toggle between ready and now-playing state."""
        self._playing = playing
        self._paused = False

        if playing:
            self._play_btn.setEnabled(True)
            self._play_btn.setText(_PAUSE)
            self._stop_btn.setEnabled(True)
            self._recording_combo.setEnabled(False)
            self._playing_widget.setVisible(True)
            self._now_playing_label.setText(f"Now Playing: {name}")
            self._progress_bar.setValue(0)
            self._progress_bar.setFormat("0.0s / 0.0s")
        else:
            self._play_btn.setText(_PLAY)
            self._play_btn.setEnabled(self._recording_combo.count() > 0)
            self._stop_btn.setEnabled(False)
            self._recording_combo.setEnabled(True)
            self._playing_widget.setVisible(False)

    def set_paused(self, paused: bool):
        """Update play/pause button to reflect pause state."""
        self._paused = paused
        self._play_btn.setText(_PLAY if paused else _PAUSE)

    def set_progress(self, current_sec: float, total_sec: float):
        """Update playback progress."""
        if total_sec > 0:
            progress = int((current_sec / total_sec) * 1000)
            self._progress_bar.setValue(progress)
        self._progress_bar.setFormat(f"{current_sec:.1f}s / {total_sec:.1f}s")
