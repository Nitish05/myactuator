# Phase 1: Safety Infrastructure & Core Controller - Research

**Researched:** 2026-02-09
**Domain:** Calibration state machine, ROS 2 motor control, PyQt6 signal/slot architecture
**Confidence:** HIGH

## Summary

Phase 1 builds a headless CalibrationController that orchestrates torque application, safety monitoring, and ROS 2 bag recording. The controller is a pure logic/orchestration layer that reuses three existing components: RosBridge (ROS 2 communication), RecordingManager (bag recording), and TriggerStore (trigger persistence, used in Phase 3). No new ROS topics, services, or driver modifications are needed -- the existing driver_node already supports per-joint torque control via the torque mode + JointState effort field pattern.

The key architectural insight is that "apply torque to one joint, free the others" maps directly to the existing driver's torque mode: set mode to "torque", then send a JointState command with the configured torque for the selected joint and 0.0 effort for all others. Zero effort in torque mode sends zero current, which effectively releases the motor. This eliminates any need to modify the driver_node.

**Primary recommendation:** Build CalibrationController as a QObject with an enum-based state machine (IDLE, RECORDING, STOPPING, ERROR), driven by pyqtSignals from RosBridge and RecordingManager. Do NOT use Qt's QStateMachine framework (overkill for 4 states and may not be available in PyQt6). Keep the controller headless (no UI) so Phase 2 can wrap it with a PyQt6 window.

## Standard Stack

### Core (already in project -- reuse only)
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| PyQt6 | >=6.4.0 | Signal/slot, QObject base, QTimer | Already used by Motor Studio |
| rclpy | (ROS 2 distro) | ROS 2 Python client | Already used by driver_node and RosBridge |
| rosbag2_py | (ROS 2 distro) | Bag recording | Already used by RecordingManager |
| sensor_msgs | (ROS 2 distro) | JointState message | Already used throughout |

### Supporting (already in project)
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| Python enum | stdlib | CalibrationState enum | State machine states |
| dataclasses | stdlib | CalibrationConfig, CalibrationResult | Configuration and result data |
| threading.Lock | stdlib | Thread-safe state access | Protecting shared state between ROS thread and Qt thread |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| Enum-based state machine | Qt QStateMachine | QStateMachine is overkill for 4 states, adds complexity, may need separate PyQt6 module |
| Enum-based state machine | python-statemachine lib | External dependency for trivial state count, not warranted |
| Direct RosBridge reuse | New ROS node | Would duplicate all ROS setup; RosBridge already handles everything needed |

**Installation:** No new packages needed. All dependencies already satisfied.

## Architecture Patterns

### Recommended Project Structure
```
myactuator_python_driver/
  studio/
    ... (existing Motor Studio files)
  calibrator/
    __init__.py
    controller.py          # CalibrationController (QObject, state machine)
    config.py              # CalibrationConfig dataclass
    main.py                # Entry point (Phase 2 will add UI here)
```

### Pattern 1: Headless Controller as QObject with Enum State Machine

**What:** CalibrationController inherits QObject, uses a Python Enum for states, exposes pyqtSignals for state changes and data updates, methods for start/stop/emergency_stop.

**When to use:** When you need a testable, reusable controller that can be driven by both a GUI (Phase 2) and programmatic tests.

**Example:**
```python
from enum import Enum, auto
from dataclasses import dataclass
from PyQt6.QtCore import QObject, pyqtSignal, QTimer

class CalibrationState(Enum):
    IDLE = auto()        # Ready, not recording
    RECORDING = auto()   # Torque applied, bag recording, tracking max position
    STOPPING = auto()    # Transitioning back to safe state
    ERROR = auto()       # Something went wrong (connection lost, etc.)

@dataclass
class CalibrationConfig:
    joint_name: str           # Which joint to apply torque to
    torque_nm: float          # Torque to apply (signed, direction matters)
    recording_name: str = ""  # Optional recording name
    offset_deg: float = 0.5   # Threshold offset in degrees

@dataclass
class CalibrationResult:
    recording_name: str
    joint_name: str
    max_position_rad: float
    torque_nm: float
    duration_sec: float

class CalibrationController(QObject):
    # Signals
    state_changed = pyqtSignal(object)      # CalibrationState
    max_position_updated = pyqtSignal(float) # Current max position (rad)
    position_updated = pyqtSignal(float)     # Current position (rad)
    error_occurred = pyqtSignal(str)         # Error message
    calibration_complete = pyqtSignal(object) # CalibrationResult

    def __init__(self, ros_bridge, recording_manager, parent=None):
        super().__init__(parent)
        self._ros_bridge = ros_bridge
        self._recording_manager = recording_manager
        self._state = CalibrationState.IDLE
        self._config = None
        self._max_position_rad = 0.0
        # ... wire up signals from ros_bridge and recording_manager
```

### Pattern 2: Per-Joint Torque via Existing Driver Torque Mode

**What:** Send torque to one joint while freeing others using the existing driver's torque mode. No driver modifications needed.

**When to use:** For the calibration sequence where one joint gets torque and all others must be free.

**How it works in the existing driver:**
1. RosBridge.set_mode("torque") -- sets `_torque_mode = True` in driver
2. RosBridge.send_joint_command(msg) -- JointState with effort array
3. Driver control loop reads `_control_efforts[joint_name]` per joint
4. `motor.send_torque(effort)` -- sends torque setpoint to each motor
5. effort=0.0 sends zero torque via `sendTorqueSetpoint(0.0, torque_constant)`, effectively releasing the motor

**Example:**
```python
def _apply_torque(self):
    """Apply torque to selected joint, zero effort to others."""
    msg = JointState()
    msg.name = list(self._ros_bridge.joint_names)
    msg.effort = []
    for name in msg.name:
        if name == self._config.joint_name:
            msg.effort.append(self._config.torque_nm)
        else:
            msg.effort.append(0.0)
    # Position and velocity arrays left empty -- driver only reads effort in torque mode
    self._ros_bridge.send_joint_command(msg)
```

**Critical detail:** The `_joint_ctrl_callback` in driver_node (line 428-429) sets `_control_efforts[name] = msg.effort[i]`. The effort values persist in the driver until overwritten. So the controller only needs to send the effort command once when starting, not continuously. However, for safety, it may be good to send it periodically (e.g., every 100ms via QTimer) to guard against dropped messages.

### Pattern 3: Connection Monitoring with Automatic Abort

**What:** Subscribe to RosBridge.connection_status_changed signal, abort calibration if connection is lost during recording.

**When to use:** Always. This is a safety requirement (CONN-01, CONN-02).

**How it works:** RosBridge already tracks connection via heartbeat -- if no `/joint_states` message for 2 seconds, it emits `connection_status_changed(False)`. The controller connects to this signal and transitions to ERROR state if it fires during RECORDING.

**Example:**
```python
def _on_connection_changed(self, connected: bool):
    if not connected and self._state == CalibrationState.RECORDING:
        self._abort("Connection to driver lost")

def _abort(self, reason: str):
    self._stop_torque()
    self._recording_manager.stop_recording()
    self._state = CalibrationState.ERROR
    self.state_changed.emit(self._state)
    self.error_occurred.emit(reason)
```

### Pattern 4: Max Position Tracking from Joint State Stream

**What:** Track the maximum absolute position reached by the selected joint during torque application, updated from the joint_state_received signal.

**When to use:** During RECORDING state. This is the core measurement for threshold computation in Phase 3.

**Example:**
```python
def _on_joint_state(self, msg):
    """Handle joint state update during calibration."""
    if self._state != CalibrationState.RECORDING:
        return

    # Find selected joint index
    if self._config.joint_name in msg.name:
        idx = list(msg.name).index(self._config.joint_name)
        position = msg.position[idx]
        self.position_updated.emit(position)

        # Track max position (absolute value, since torque can be + or -)
        # Actually: track in the direction of torque application
        if self._config.torque_nm > 0:
            if position > self._max_position_rad:
                self._max_position_rad = position
                self.max_position_updated.emit(self._max_position_rad)
        else:
            if position < self._max_position_rad:
                self._max_position_rad = position
                self.max_position_updated.emit(self._max_position_rad)

    # Record frame to bag
    timestamp_ns = int(time.time() * 1e9)
    self._recording_manager.record_frame(msg, timestamp_ns)
```

### Pattern 5: Emergency Stop Flow

**What:** Emergency stop must immediately cease torque, stop recording, and return to safe state. Must work from any state.

**When to use:** SAFE-01, SAFE-02 requirements.

**Example:**
```python
def emergency_stop(self):
    """Immediate emergency stop from any state."""
    # 1. Call driver emergency stop (stops all motors)
    self._ros_bridge.emergency_stop()

    # 2. Stop recording if active
    if self._recording_manager.is_recording:
        self._recording_manager.stop_recording()

    # 3. Transition to IDLE (not ERROR -- e-stop is intentional)
    self._state = CalibrationState.IDLE
    self.state_changed.emit(self._state)
```

### Anti-Patterns to Avoid

- **Direct motor control from calibrator:** Never bypass RosBridge to talk to motors directly. All commands go through RosBridge -> ROS topics -> driver_node -> motors.
- **Subclassing QThread:** Use QObject worker + signals pattern (already established by RosBridge). The CalibrationController should be a QObject, not a thread.
- **Global mode for per-joint control:** Don't try to set some joints to "free" mode and one to "torque" mode -- the driver has a single global mode. Instead, use torque mode with effort=0 for "free" joints.
- **Modifying driver_node for calibration:** The driver already supports everything needed. Don't add calibration-specific logic to the driver.
- **Blocking the Qt event loop:** All state transitions and commands must be non-blocking. Use signals/slots and QTimer for periodic operations.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| ROS 2 communication | Custom ROS node | Existing RosBridge | Already handles threading, connection monitoring, all needed topics |
| Bag recording | Custom rosbag2 writer | Existing RecordingManager | Already handles bag lifecycle, frame recording, compatibility |
| Trigger persistence | Custom JSON store | Existing TriggerStore | Already handles load/save/CRUD for HysteresisTorqueTrigger |
| Connection detection | Custom heartbeat | RosBridge.connection_status_changed | Already implements 2-second timeout on /joint_states |
| Emergency stop | Custom motor shutdown | RosBridge.emergency_stop() | Already calls /motor_driver/emergency_stop service |
| Mode switching | Custom topic publisher | RosBridge.set_mode() | Already publishes to /motor_driver/set_mode |
| Joint command sending | Custom JointState publisher | RosBridge.send_joint_command() | Already handles timestamping and publishing |

**Key insight:** The entire ROS communication layer is already built. The CalibrationController is purely orchestration logic -- it coordinates existing components via their signal/slot interfaces. It should not contain any ROS, CAN, or motor-specific code.

## Common Pitfalls

### Pitfall 1: Sending Torque Once and Assuming It Persists
**What goes wrong:** Controller sends effort command once at start, but if a ROS message is dropped or the driver restarts, the motor silently loses its torque setpoint.
**Why it happens:** ROS 2 topics are unreliable (best-effort by default). The driver's `/joint_state_ctrl` uses QoS depth 10 but messages can still be lost.
**How to avoid:** Send the effort command periodically (every 100ms via QTimer) during RECORDING state. This is a standard ROS pattern for safety-critical commands.
**Warning signs:** Motor stops applying torque during calibration without any error.

### Pitfall 2: Race Between Emergency Stop and State Transitions
**What goes wrong:** Emergency stop fires while a state transition is in progress, leading to inconsistent state (e.g., recording stopped but state still says RECORDING).
**Why it happens:** Signals can be queued and processed out of order if multiple state changes happen quickly.
**How to avoid:** Make emergency_stop() a synchronous operation that immediately sets state before emitting any signals. Use a lock or check-and-set pattern.
**Warning signs:** State label shows "Recording" after e-stop.

### Pitfall 3: Max Position Tracking Direction Confusion
**What goes wrong:** Max position tracks in the wrong direction when torque is negative (pulling instead of pushing).
**Why it happens:** Using `max()` when torque is negative means we should track `min()` of position.
**How to avoid:** Track in the direction of torque: if torque > 0, track max; if torque < 0, track min. Or track the position that is furthest from the starting position in the direction of applied torque.
**Warning signs:** Max position shows a value less than (or greater than) expected based on physical observation.

### Pitfall 4: Not Initializing Max Position to Starting Position
**What goes wrong:** Max position starts at 0.0 instead of the joint's current position, causing incorrect threshold computation.
**Why it happens:** Controller forgets to read starting position before applying torque.
**How to avoid:** On calibration start, capture the joint's current position from the first joint_state message and initialize max_position_rad to that value.
**Warning signs:** Max position shows 0.0 or a value that doesn't match the joint's actual range.

### Pitfall 5: Recording Frames Before Bag Writer Is Ready
**What goes wrong:** First few joint state messages are lost because RecordingManager.start_recording() hasn't finished opening the bag file yet.
**Why it happens:** start_recording() is synchronous but the JointState callback fires immediately.
**How to avoid:** Start recording BEFORE sending the torque command. The sequence must be: (1) start recording, (2) verify recording started (check is_recording), (3) set torque mode, (4) send effort command.
**Warning signs:** First ~100ms of data missing from recording.

### Pitfall 6: Forgetting to Return Motors to Safe State on Stop
**What goes wrong:** After stopping calibration, motors remain in torque mode with last commanded effort.
**Why it happens:** Controller stops recording but forgets to zero the effort and switch mode.
**How to avoid:** On stop: (1) send zero effort to all joints, (2) switch mode back to "position" or "disabled", (3) THEN stop recording.
**Warning signs:** Motor continues applying torque after calibration stop.

## Code Examples

### Complete Start Calibration Sequence
```python
def start_calibration(self, config: CalibrationConfig) -> bool:
    """Start a calibration sequence. Returns True if started successfully."""
    if self._state != CalibrationState.IDLE:
        self.error_occurred.emit("Cannot start: calibration already in progress")
        return False

    if not self._ros_bridge.is_connected:
        self.error_occurred.emit("Cannot start: driver not connected")
        return False

    self._config = config
    self._max_position_rad = None  # Will be set from first joint state
    self._start_time = time.time()

    # Generate recording name if not provided
    recording_name = config.recording_name
    if not recording_name:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        recording_name = f"calib_{config.joint_name}_{timestamp}"

    # Step 1: Start bag recording FIRST
    if not self._recording_manager.start_recording(recording_name):
        self.error_occurred.emit("Failed to start recording")
        return False

    # Step 2: Switch to torque mode
    self._ros_bridge.set_mode("torque")

    # Step 3: Send per-joint effort command
    self._send_effort_command()

    # Step 4: Start periodic effort refresh timer (100ms)
    self._effort_timer = QTimer(self)
    self._effort_timer.timeout.connect(self._send_effort_command)
    self._effort_timer.start(100)

    # Step 5: Update state
    self._state = CalibrationState.RECORDING
    self.state_changed.emit(self._state)

    return True
```

### Complete Stop Calibration Sequence
```python
def stop_calibration(self) -> Optional[CalibrationResult]:
    """Stop calibration gracefully. Returns result if successful."""
    if self._state != CalibrationState.RECORDING:
        return None

    self._state = CalibrationState.STOPPING
    self.state_changed.emit(self._state)

    # Step 1: Stop effort refresh timer
    if self._effort_timer:
        self._effort_timer.stop()
        self._effort_timer = None

    # Step 2: Zero all efforts
    msg = JointState()
    msg.name = list(self._ros_bridge.joint_names)
    msg.effort = [0.0] * len(msg.name)
    self._ros_bridge.send_joint_command(msg)

    # Step 3: Switch to position mode (safe)
    self._ros_bridge.set_mode("position")

    # Step 4: Stop recording
    recording_name = self._recording_manager.stop_recording()

    # Step 5: Build result
    result = CalibrationResult(
        recording_name=recording_name or "",
        joint_name=self._config.joint_name,
        max_position_rad=self._max_position_rad or 0.0,
        torque_nm=self._config.torque_nm,
        duration_sec=time.time() - self._start_time,
    )

    # Step 6: Return to IDLE
    self._state = CalibrationState.IDLE
    self.state_changed.emit(self._state)
    self.calibration_complete.emit(result)

    return result
```

### RosBridge Reuse Pattern (how the calibrator creates and uses it)
```python
# In calibrator/main.py (Phase 1: headless, Phase 2: with UI)
from myactuator_python_driver.studio.ros_bridge import RosBridge
from myactuator_python_driver.studio.recording_manager import RecordingManager

# RosBridge and RecordingManager are created at app level and shared
ros_bridge = RosBridge()
recording_manager = RecordingManager()

controller = CalibrationController(ros_bridge, recording_manager)

# Wire up joint states for recording
ros_bridge.joint_state_received.connect(controller.on_joint_state)
ros_bridge.connection_status_changed.connect(controller.on_connection_changed)

ros_bridge.start()
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Custom ROS node per app | Reuse RosBridge QObject | Already established in Motor Studio | No new ROS nodes needed |
| rosbag2 Humble API | rosbag2 Humble+Jazzy compat | RecordingManager already handles | Compatible TopicMetadata creation |
| Single global control mode | Per-joint effort in torque mode | Already in driver_node | Enables mixed torque/free per joint |

**Deprecated/outdated:**
- None relevant. All existing components are current and maintained.

## Open Questions

1. **Zero effort vs explicit release for "free" joints**
   - What we know: In torque mode, `send_torque(0.0)` calls `sendTorqueSetpoint(0.0, torque_constant)`. `release()` calls `sendCurrentSetpoint(0.0)`. Both should result in zero holding torque.
   - What's unclear: Whether there's a behavioral difference in the motor firmware between zero torque command and zero current command. In practice they appear identical.
   - Recommendation: Use effort=0.0 in the JointState message (which triggers `send_torque(0.0)` in torque mode). This is the simplest approach and avoids needing a new per-joint release mechanism.

2. **Max position direction for unsigned torque values**
   - What we know: Torque sign determines direction. Positive torque moves one way, negative the other.
   - What's unclear: Whether the user always knows which sign corresponds to which physical direction (it depends on motor `inverted` config).
   - Recommendation: Track the position that moves furthest from the starting position in the direction of motion. Initialize max_position from the first joint state reading. Phase 2's UI can show direction feedback (DISP-03, which is in Phase 2).

3. **Recording name collision handling**
   - What we know: RecordingManager writes to `recordings_dir / name`. If a directory already exists, rosbag2 behavior is undefined.
   - What's unclear: Whether rosbag2_py raises an error or overwrites.
   - Recommendation: Generate unique names with timestamp prefix (already the pattern). Validate before starting.

## Sources

### Primary (HIGH confidence)
- **Existing codebase** -- ros_bridge.py, recording_manager.py, config.py, driver_node.py, motor_wrapper.py (read in full, all patterns verified against source)
- **driver_node.py control loop** (lines 291-411) -- Verified per-joint effort handling in torque mode
- **RosBridge API** -- Verified connection monitoring, mode setting, emergency stop, joint command sending
- **RecordingManager API** -- Verified start/stop recording, frame recording, signal interface

### Secondary (MEDIUM confidence)
- [Qt State Machine Framework (Qt 6)](https://doc.qt.io/qt-6/qstatemachine.html) -- Confirmed QStateMachine exists but is in separate QtStateMachine module
- [PyQt6 threading patterns](https://www.pythonguis.com/tutorials/multithreading-pyqt6-applications-qthreadpool/) -- Confirmed QObject worker + signals is the standard pattern
- [ROS 2 JointState message](https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html) -- Confirmed effort field for per-joint torque

### Tertiary (LOW confidence)
- None. All critical findings verified against existing source code.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH -- all components already exist in the codebase, no new dependencies
- Architecture: HIGH -- patterns directly derived from reading existing Motor Studio code and driver_node
- Pitfalls: HIGH -- identified from actual code analysis (e.g., effort persistence, recording timing, stop sequence)

**Research date:** 2026-02-09
**Valid until:** 2026-03-09 (stable -- no external dependencies changing, all internal code)
