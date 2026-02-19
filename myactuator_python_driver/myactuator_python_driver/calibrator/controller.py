"""
Calibration controller -- headless orchestration engine.

Coordinates torque application, position tracking, bag recording, and safety
monitoring for automated torque threshold calibration. Reuses RosBridge and
RecordingManager (no new ROS nodes or driver modifications).

All ROS communication goes through RosBridge. Do NOT import rclpy directly.
"""

import math
import time
from datetime import datetime
from typing import Optional

from PySide6.QtCore import QObject, QTimer, Signal
from sensor_msgs.msg import JointState

from .config import CalibrationConfig, CalibrationResult, CalibrationState

# Position must reverse by this much from the extreme before we
# switch to threshold tracking (avoids noise triggering phase 2).
_REVERSAL_THRESHOLD_RAD = math.radians(1.0)


class CalibrationController(QObject):
    """
    State machine controller for automated torque threshold calibration.

    Applies configured torque to a selected joint while sending zero effort
    to all other joints, tracks the maximum position reached, and records
    the entire sequence to a ROS 2 bag.

    Signals:
        state_changed: Emitted on every state transition with CalibrationState.
        max_position_updated: Emitted when max position changes (radians).
        position_updated: Emitted on every joint state with current position (radians).
        error_occurred: Emitted with error message string.
        calibration_complete: Emitted with CalibrationResult on successful stop.
    """

    state_changed = Signal(object)
    max_position_updated = Signal(float)
    position_updated = Signal(float)
    error_occurred = Signal(str)
    calibration_complete = Signal(object)
    reversal_detected = Signal(float)  # extreme position when reversal occurs
    threshold_locked = Signal(float)   # threshold frozen after settle window

    def __init__(self, ros_bridge, recording_manager, parent=None):
        super().__init__(parent)

        self._ros_bridge = ros_bridge
        self._recording_manager = recording_manager

        self._state = CalibrationState.IDLE
        self._config: Optional[CalibrationConfig] = None
        self._extreme_position_rad: Optional[float] = None
        self._threshold_position_rad: Optional[float] = None
        self._reversal_detected: bool = False
        self._threshold_frozen: bool = False
        self._start_time: float = 0.0
        self._effort_timer: Optional[QTimer] = None

        # Connect to RosBridge signals for joint state and connection monitoring
        self._ros_bridge.joint_state_received.connect(self._on_joint_state)
        self._ros_bridge.connection_status_changed.connect(
            self._on_connection_changed
        )

    # -- Properties ----------------------------------------------------------

    @property
    def state(self) -> CalibrationState:
        """Current calibration state."""
        return self._state

    @property
    def is_active(self) -> bool:
        """True if calibration is in progress (RECORDING or STOPPING)."""
        return self._state in (CalibrationState.RECORDING, CalibrationState.STOPPING)

    # -- Public methods ------------------------------------------------------

    def start_calibration(self, config: CalibrationConfig) -> bool:
        """Start a calibration sequence.

        Args:
            config: Calibration configuration specifying joint, torque, etc.

        Returns:
            True if calibration started successfully, False otherwise.
        """
        # Guard: already running
        if self._state != CalibrationState.IDLE:
            self.error_occurred.emit(
                "Cannot start: calibration already in progress"
            )
            return False

        # Guard: driver not connected
        if not self._ros_bridge.is_connected:
            self.error_occurred.emit("Cannot start: driver not connected")
            return False

        self._config = config
        self._extreme_position_rad = None  # Initialize from first joint state
        self._threshold_position_rad = None
        self._reversal_detected = False
        self._threshold_frozen = False
        self._start_time = time.time()

        # Generate recording name if not provided
        recording_name = config.recording_name
        if not recording_name:
            recording_name = (
                f"calib_{config.joint_name}_"
                f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            )
            config.recording_name = recording_name

        # Step 1: Start bag recording FIRST (before torque)
        if not self._recording_manager.start_recording(recording_name):
            self.error_occurred.emit("Failed to start recording")
            return False

        # Step 2: Switch to torque mode
        self._ros_bridge.set_mode("torque")

        # Step 3: Send initial effort command
        self._send_effort_command()

        # Step 4: Start periodic effort refresh at 100ms
        self._effort_timer = QTimer(self)
        self._effort_timer.timeout.connect(self._send_effort_command)
        self._effort_timer.start(100)

        # Step 5: Transition to RECORDING
        self._state = CalibrationState.RECORDING
        self.state_changed.emit(self._state)

        return True

    def stop_calibration(self) -> Optional[CalibrationResult]:
        """Stop calibration gracefully.

        Returns:
            CalibrationResult if stopped from RECORDING state, None otherwise.
        """
        if self._state != CalibrationState.RECORDING:
            return None

        # Transition to STOPPING
        self._state = CalibrationState.STOPPING
        self.state_changed.emit(self._state)

        # Step 1: Stop effort refresh timer
        if self._effort_timer is not None:
            self._effort_timer.stop()
            self._effort_timer.deleteLater()
            self._effort_timer = None

        # Step 2: Zero all efforts and switch to free mode (no torque)
        try:
            msg = JointState()
            msg.name = list(self._ros_bridge.joint_names)
            msg.effort = [0.0] * len(msg.name)
            self._ros_bridge.send_joint_command(msg)
            # Step 3: Switch to free mode so motors stop applying any torque
            self._ros_bridge.set_mode("free")
        except Exception:
            pass  # ROS handle may be destroyed during shutdown

        # Step 4: Stop recording
        self._recording_manager.stop_recording()

        # Step 5: Build result
        # Use threshold if reversal was detected, otherwise fall back to extreme
        extreme = self._extreme_position_rad if self._extreme_position_rad is not None else 0.0
        if self._reversal_detected and self._threshold_position_rad is not None:
            threshold = self._threshold_position_rad
        else:
            threshold = extreme

        result = CalibrationResult(
            recording_name=self._config.recording_name,
            joint_name=self._config.joint_name,
            max_position_rad=threshold,
            extreme_position_rad=extreme,
            torque_nm=self._config.torque_nm,
            duration_sec=time.time() - self._start_time,
        )

        # Step 6: Return to IDLE
        self._state = CalibrationState.IDLE
        self.state_changed.emit(self._state)
        self.calibration_complete.emit(result)

        return result

    def emergency_stop(self):
        """Immediate emergency stop from any state.

        Calls the driver emergency stop first (immediate motor stop), then
        cleans up recording and timers. Transitions to IDLE (not ERROR)
        because e-stop is an intentional user action.
        """
        # 1. Driver emergency stop FIRST (immediate motor safety)
        try:
            self._ros_bridge.emergency_stop()
        except Exception:
            pass  # ROS handle may already be destroyed during shutdown

        # 2. Stop effort refresh timer
        if self._effort_timer is not None:
            self._effort_timer.stop()
            self._effort_timer.deleteLater()
            self._effort_timer = None

        # 3. Stop recording if active
        if self._recording_manager.is_recording:
            self._recording_manager.stop_recording()

        # 4. Transition to IDLE (intentional action, not error)
        self._state = CalibrationState.IDLE
        self.state_changed.emit(self._state)

    # -- Internal methods ----------------------------------------------------

    def _send_effort_command(self):
        """Send per-joint effort command via RosBridge.

        Applies configured torque to the selected joint and zero effort
        to all others. Position and velocity arrays are left empty since
        the driver only reads effort in torque mode.
        """
        if self._config is None:
            return

        msg = JointState()
        msg.name = list(self._ros_bridge.joint_names)
        msg.effort = [
            self._config.torque_nm if name == self._config.joint_name else 0.0
            for name in msg.name
        ]
        try:
            self._ros_bridge.send_joint_command(msg)
        except Exception:
            pass  # ROS handle may be destroyed during shutdown

    def _on_joint_state(self, msg):
        """Handle incoming joint state during calibration.

        Two-phase position tracking:
          Phase 1: Track extreme position in torque direction (arm pressing down).
          Phase 2: After reversal detected (user rotating part), track the peak
                   position going back — this becomes the trigger threshold.

        Also records each frame to the bag.
        """
        if self._state != CalibrationState.RECORDING:
            return

        if self._config is None:
            return

        # Find the selected joint in the message
        try:
            idx = list(msg.name).index(self._config.joint_name)
        except ValueError:
            return  # Selected joint not in this message

        position = msg.position[idx]
        self.position_updated.emit(position)

        positive_torque = self._config.torque_nm >= 0

        if self._extreme_position_rad is None:
            # First reading — initialize extreme
            self._extreme_position_rad = position
            self.max_position_updated.emit(self._extreme_position_rad)

        elif not self._reversal_detected:
            # Phase 1: track extreme in torque direction
            if positive_torque:
                if position > self._extreme_position_rad:
                    self._extreme_position_rad = position
                    self.max_position_updated.emit(self._extreme_position_rad)
                elif position < self._extreme_position_rad - _REVERSAL_THRESHOLD_RAD:
                    self._start_threshold_tracking(position)
            else:
                if position < self._extreme_position_rad:
                    self._extreme_position_rad = position
                    self.max_position_updated.emit(self._extreme_position_rad)
                elif position > self._extreme_position_rad + _REVERSAL_THRESHOLD_RAD:
                    self._start_threshold_tracking(position)

        elif not self._threshold_frozen:
            # Phase 2: track peak in opposite direction until settle window expires
            if positive_torque:
                if position < self._threshold_position_rad:
                    self._threshold_position_rad = position
                    self.max_position_updated.emit(self._threshold_position_rad)
            else:
                if position > self._threshold_position_rad:
                    self._threshold_position_rad = position
                    self.max_position_updated.emit(self._threshold_position_rad)

        # Record frame to bag
        self._recording_manager.record_frame(msg, int(time.time() * 1e9))

    def _start_threshold_tracking(self, position):
        """Begin phase 2: track threshold for the settle window, then freeze."""
        self._reversal_detected = True
        self._threshold_position_rad = position
        self.reversal_detected.emit(self._extreme_position_rad)

        # Freeze threshold after settle window
        settle_ms = int(self._config.settle_time_sec * 1000)
        QTimer.singleShot(settle_ms, self._freeze_threshold)

    def _freeze_threshold(self):
        """Lock threshold after settle window expires."""
        if self._threshold_position_rad is not None and not self._threshold_frozen:
            self._threshold_frozen = True
            self.threshold_locked.emit(self._threshold_position_rad)

    def _on_connection_changed(self, connected: bool):
        """Handle connection status changes from RosBridge.

        If connection is lost during RECORDING, abort to ERROR state.
        """
        if not connected and self._state == CalibrationState.RECORDING:
            # Stop effort timer
            if self._effort_timer is not None:
                self._effort_timer.stop()
                self._effort_timer.deleteLater()
                self._effort_timer = None

            # Stop recording if active
            if self._recording_manager.is_recording:
                self._recording_manager.stop_recording()

            # Transition to ERROR
            self._state = CalibrationState.ERROR
            self.state_changed.emit(self._state)
            self.error_occurred.emit(
                "Connection to driver lost during calibration"
            )
