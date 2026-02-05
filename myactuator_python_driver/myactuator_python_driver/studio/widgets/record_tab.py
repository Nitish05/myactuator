"""
Record tab widget for Motor Recording Studio.

Recording controls and status display.
"""

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QFrame, QProgressBar, QGroupBox
)


class RecordTab(QWidget):
    """
    Record tab widget for recording motor trajectories.

    Contains:
    - Recording name input
    - Start/Stop recording button
    - Duration and frame count display
    """

    # Signals
    start_recording = pyqtSignal(str)  # recording name
    stop_recording = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._recording = False
        self._record_start_time = 0.0
        self._frame_count = 0
        self._setup_ui()

        # Timer for duration updates
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_duration)

    def _setup_ui(self):
        """Set up the UI components."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        # Recording name group
        name_group = QGroupBox("Recording Name")
        name_layout = QHBoxLayout(name_group)

        self._name_edit = QLineEdit()
        self._name_edit.setPlaceholderText("Auto-generated if empty")
        name_layout.addWidget(self._name_edit)

        layout.addWidget(name_group)

        # Status group
        status_group = QGroupBox("Recording Status")
        status_layout = QVBoxLayout(status_group)

        # Status indicator
        status_row = QHBoxLayout()
        self._status_led = QLabel()
        self._status_led.setFixedSize(16, 16)
        self._status_led.setStyleSheet("""
            QLabel {
                background-color: #616161;
                border-radius: 8px;
            }
        """)
        status_row.addWidget(self._status_led)

        self._status_label = QLabel("Ready")
        self._status_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        status_row.addWidget(self._status_label)
        status_row.addStretch()

        status_layout.addLayout(status_row)

        # Duration and frames
        info_layout = QHBoxLayout()

        duration_layout = QVBoxLayout()
        duration_layout.addWidget(QLabel("Duration"))
        self._duration_label = QLabel("0:00.0")
        self._duration_label.setStyleSheet("font-size: 24px; font-weight: bold;")
        duration_layout.addWidget(self._duration_label)
        info_layout.addLayout(duration_layout)

        frames_layout = QVBoxLayout()
        frames_layout.addWidget(QLabel("Frames"))
        self._frames_label = QLabel("0")
        self._frames_label.setStyleSheet("font-size: 24px; font-weight: bold;")
        frames_layout.addWidget(self._frames_label)
        info_layout.addLayout(frames_layout)

        info_layout.addStretch()

        status_layout.addLayout(info_layout)

        layout.addWidget(status_group)

        # Record button
        self._record_btn = QPushButton("Start Recording")
        self._record_btn.setObjectName("recordButton")
        self._record_btn.setStyleSheet("""
            QPushButton {
                font-weight: bold;
                font-size: 16px;
                padding: 16px;
                min-height: 60px;
            }
        """)
        self._record_btn.clicked.connect(self._on_record_clicked)
        layout.addWidget(self._record_btn)

        # Info text
        info_label = QLabel(
            "Recording will automatically switch to free mode.\n"
            "Move the motors by hand to record the trajectory."
        )
        info_label.setStyleSheet("color: #757575;")
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        # Spacer
        layout.addStretch()

    def _on_record_clicked(self):
        """Handle record button click."""
        if self._recording:
            self.stop_recording.emit()
        else:
            name = self._name_edit.text().strip()
            self.start_recording.emit(name if name else "")

    def _update_duration(self):
        """Update the duration display."""
        if self._recording:
            import time
            elapsed = time.time() - self._record_start_time
            mins = int(elapsed // 60)
            secs = elapsed % 60
            self._duration_label.setText(f"{mins}:{secs:04.1f}")

    # === Public methods ===

    def set_recording(self, recording: bool, name: str = ""):
        """Set recording state."""
        import time
        self._recording = recording

        if recording:
            self._record_start_time = time.time()
            self._frame_count = 0
            self._status_label.setText(f"Recording: {name}")
            self._status_led.setStyleSheet("""
                QLabel {
                    background-color: #f44336;
                    border-radius: 8px;
                }
            """)
            self._record_btn.setText("Stop Recording")
            self._record_btn.setStyleSheet("""
                QPushButton {
                    font-weight: bold;
                    font-size: 16px;
                    padding: 16px;
                    min-height: 60px;
                    background-color: #d32f2f;
                    color: white;
                }
                QPushButton:hover {
                    background-color: #f44336;
                }
            """)
            self._name_edit.setEnabled(False)
            self._timer.start(100)  # Update 10x per second
        else:
            self._timer.stop()
            self._status_label.setText("Ready")
            self._status_led.setStyleSheet("""
                QLabel {
                    background-color: #616161;
                    border-radius: 8px;
                }
            """)
            self._record_btn.setText("Start Recording")
            self._record_btn.setStyleSheet("""
                QPushButton {
                    font-weight: bold;
                    font-size: 16px;
                    padding: 16px;
                    min-height: 60px;
                }
            """)
            self._name_edit.setEnabled(True)
            self._name_edit.clear()

    def set_frame_count(self, count: int):
        """Update the frame count display."""
        self._frame_count = count
        self._frames_label.setText(str(count))

    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._recording
