# Architecture Patterns

**Domain:** Torque Threshold Calibration Tool (standalone PyQt6 app for robotics motor control)
**Researched:** 2026-02-09
**Confidence:** HIGH -- based entirely on existing codebase analysis, no external research needed

## Recommended Architecture

The calibration app is a **standalone PyQt6 process** that communicates with the existing MotorDriverNode over ROS 2 topics. It reuses three existing components directly (`RosBridge`, `RecordingManager`, `TriggerStore`) and introduces two new components (`CalibrationController`, `CalibrationWindow`).

### System Context

```
+------------------+       ROS 2 Topics        +------------------+
|  Calibration App |  <------ /joint_states --- |  MotorDriverNode |
|  (new process)   |  --- /joint_effort_ctrl -> |  (existing)      |
|                  |  --- ~/set_mode ---------> |                  |
|                  |  --- ~/set_enabled -------> |                  |
+------------------+                            +------------------+
       |
       |  filesystem
       v
+------------------+
|  recordings/     |
|  triggers.json   |
+------------------+
```

### Internal Component Architecture

```
CalibrationWindow (QMainWindow)
    |
    |-- owns --> CalibrationController (QObject)
    |                |
    |                |-- owns --> RosBridge (reused, unmodified)
    |                |                |-- background QThread with ROS node
    |                |                |-- subscribes: /joint_states
    |                |                |-- publishes:  /joint_effort_ctrl (new usage)
    |                |                |               ~/set_mode
    |                |
    |                |-- owns --> RecordingManager (reused, unmodified)
    |                |                |-- rosbag2_py writer
    |                |                |-- recordings directory access
    |                |
    |                |-- owns --> TriggerStore (reused, unmodified)
    |                                |-- triggers.json read/write
    |
    |-- contains --> UI widgets (joint selector, torque input, etc.)
```

### Component Boundaries

| Component | Responsibility | Communicates With | New or Reused |
|-----------|---------------|-------------------|---------------|
| `CalibrationWindow` | UI layout, user input, display live data | CalibrationController (method calls + Qt signals) | **New** |
| `CalibrationController` | Calibration state machine, orchestrates ROS commands and recording | RosBridge, RecordingManager, TriggerStore, CalibrationWindow (signals) | **New** |
| `RosBridge` | ROS 2 communication in background thread | MotorDriverNode (ROS topics), CalibrationController (Qt signals) | **Reused unmodified** |
| `RecordingManager` | ROS bag write/read, recording metadata | Filesystem (recordings/), CalibrationController (method calls + signals) | **Reused unmodified** |
| `TriggerStore` | Persistent trigger JSON storage | Filesystem (triggers.json), CalibrationController (method calls) | **Reused unmodified** |
| `MotorDriverNode` | Motor control loop, CAN communication | CAN bus (motors), ROS topics (Calibration App) | **Existing, no changes needed** |

## Component Detail

### CalibrationController (New -- Core Logic)

The controller is the only new substantial component. It encapsulates a calibration state machine and keeps all ROS/recording/trigger logic out of the window class.

**State Machine:**

```
IDLE --> PREPARING --> CALIBRATING --> CALCULATING --> IDLE
  ^                        |                |
  |                        v                |
  +------ STOPPING <-------+               |
  +<----------------------------------------+
```

**States:**
- `IDLE`: Waiting for user to configure and start. RosBridge connected, joint states updating.
- `PREPARING`: User clicked Start. Set mode to torque, begin recording, wait one control loop cycle for mode confirmation.
- `CALIBRATING`: Torque is being applied. Publishing effort commands each cycle. Tracking max position of target joint. Recording bag frames.
- `CALCULATING`: User clicked Stop. Stop recording, calculate threshold from max position + offset, create trigger, save to TriggerStore.
- `STOPPING`: Cancellation path. Stop torque (set all efforts to 0), stop recording, return to IDLE.

**Key responsibilities:**
- Subscribe to `joint_state_received` signal from RosBridge to track live positions and max position
- Publish effort commands: target torque on calibration joint, 0.0 on all others
- Feed incoming JointState messages to RecordingManager for bag recording
- On completion: compute threshold, construct `HysteresisTorqueTrigger`, save via TriggerStore

**Signals emitted:**
- `state_changed(str)` -- for window to update UI enable/disable states
- `position_updated(float, float)` -- (current_position, max_position) for live display
- `calibration_complete(HysteresisTorqueTrigger)` -- result for display/confirmation
- `error(str)` -- for error display

### CalibrationWindow (New -- UI Shell)

Thin UI layer. No ROS or recording logic. Receives signals from controller, calls controller methods.

**Layout:**

```
+--------------------------------------------------+
|  Torque Threshold Calibrator          [X]         |
+--------------------------------------------------+
|  Connection: Connected / Disconnected             |
+--------------------------------------------------+
|                                                   |
|  Joint:    [  joint1  v ]                         |
|  Torque:   [  1.00  ] Nm                          |
|  Offset:   [  0.50  ] deg                         |
|  Name:     [ auto-generated           ]           |
|                                                   |
+--------------------------------------------------+
|                                                   |
|  Current Position:    0.4523 rad                  |
|  Max Position:        0.8912 rad     <-- live     |
|  Threshold (preview): 0.9000 rad                  |
|                                                   |
+--------------------------------------------------+
|                                                   |
|  [ START CALIBRATION ]  /  [ STOP ]               |
|                                                   |
+--------------------------------------------------+
|  Status: Idle / Recording... / Complete           |
+--------------------------------------------------+
```

**No tabs, no docks, no menus.** Single-purpose window. This is a tool, not an application. Keep it minimal.

### RosBridge (Reused Unmodified)

The existing `RosBridge` class provides everything the calibration app needs:

| Capability | RosBridge Method | Calibration Use |
|------------|-----------------|-----------------|
| Subscribe to joint states | `joint_state_received` signal | Track position, feed to RecordingManager |
| Detect available joints | `joint_names` property | Populate joint selector |
| Connection status | `connection_status_changed` signal | Enable/disable Start button |
| Set mode | `set_mode("torque")` | Enter torque mode before calibration |
| Send joint effort | See analysis below | Apply torque to target joint |

**Critical gap: `RosBridge` has no `send_effort()` method.** It has `send_joint_command()` which publishes to `/joint_state_ctrl` (position control), but no method for `/joint_effort_ctrl`. However, `RosBridge` does not need modification because:

The driver node's `_effort_ctrl_callback` listens on `/joint_effort_ctrl` for `Float64MultiArray`. The calibration app can publish to this topic through the RosBridge's internal ROS node. Two approaches:

1. **Add a `send_effort()` method to RosBridge** -- clean but modifies a shared component.
2. **Extend RosBridge via subclass or add the method** -- the method is trivial (3 lines) and follows the exact same pattern as existing methods.

**Recommendation: Add `send_effort(efforts: list)` to `RosBridgeWorker` and `RosBridge`.** This is a small, backwards-compatible addition (not a modification). The existing Motor Studio ControlPanel already has torque mode but uses mode-based control (set mode to torque, then JointState effort field), not the Float64MultiArray effort topic. Adding `send_effort()` makes the RosBridge complete for all control modes and benefits both apps.

Alternatively, the calibration app can **use torque mode via the existing mechanism**: set mode to "torque" and then publish JointState messages with effort values via `send_joint_command()`. Looking at the driver's `_joint_ctrl_callback`:

```python
if i < len(msg.effort):
    self._control_efforts[name] = msg.effort[i]
```

The driver stores efforts from JointState messages. And the control loop, when in torque mode, applies `self._control_efforts[joint_name]`. So the existing `send_joint_command()` already works: send a JointState with the target effort for the calibration joint and 0.0 for others, with mode set to "torque".

**Final answer: RosBridge works as-is.** Use `set_mode("torque")` + `send_joint_command(msg)` with effort values. No modification needed.

### RecordingManager (Reused Unmodified)

The calibration app uses RecordingManager in a subset of its capability:

| Method | Use |
|--------|-----|
| `start_recording(name)` | Begin recording when calibration starts |
| `record_frame(msg, timestamp_ns)` | Feed each JointState during calibration |
| `stop_recording()` | End recording when user stops |
| `recordings_dir` | Path for TriggerStore initialization |

The RecordingManager's playback features are not used. The calibration app calls `record_frame()` directly from its `_on_joint_state()` handler, same pattern as `MainWindow._on_joint_state()`.

### TriggerStore (Reused Unmodified)

After calibration completes:

```python
trigger = HysteresisTorqueTrigger.create_falling(
    name=user_provided_name,
    joint_name=selected_joint,
    threshold_rad=max_position + offset_rad,
    torque_nm=selected_torque,
    recording_name=recording_name,
)
trigger_store.add(trigger)  # Persists to triggers.json
```

The trigger is then immediately available in Motor Studio's playback system because both apps read/write the same `triggers.json` file via `TriggerStore`.

## Data Flow

### During Calibration (CALIBRATING state)

```
MotorDriverNode                    CalibrationController              CalibrationWindow
     |                                    |                                 |
     |  /joint_states (500 Hz)            |                                 |
     | ---------------------------------> |                                 |
     |                                    |  extract position for target    |
     |                                    |  joint, update max_position     |
     |                                    |                                 |
     |                                    |  position_updated signal        |
     |                                    | ------------------------------> |
     |                                    |                                 |  update labels
     |                                    |                                 |
     |                                    |  record_frame(msg, ts)          |
     |                                    |  --> RecordingManager           |
     |                                    |      (writes to rosbag)         |
     |                                    |                                 |
     |  /joint_state_ctrl (effort)        |                                 |
     | <--------------------------------- |                                 |
     |  (torque on target, 0.0 on rest)   |                                 |
     |                                    |                                 |
     |  control loop applies torque       |                                 |
     |  to physical motor via CAN         |                                 |
```

### Effort Command Construction

The CalibrationController builds a JointState message each time it receives a joint_state update:

```python
msg = JointState()
msg.name = all_joint_names          # e.g., ["joint1", "joint2", "joint3"]
msg.position = [0.0, 0.0, 0.0]     # ignored in torque mode
msg.velocity = [0.0, 0.0, 0.0]     # ignored in torque mode
msg.effort = [0.0, 5.0, 0.0]       # 5.0 Nm on joint2, 0.0 (free) on others
```

The driver's `_joint_ctrl_callback` stores these efforts. The control loop in torque mode applies them:
- joint1: `send_torque(0.0)` = free
- joint2: `send_torque(5.0)` = applies calibration torque
- joint3: `send_torque(0.0)` = free

**Note:** The effort command must be sent continuously (not just once) because the driver reads `_control_efforts` each cycle. If the calibration app only sends once, it works because the dict retains the value. But sending continuously (on each joint_state update) is safer and matches the existing pattern.

### After Calibration (CALCULATING state)

```
CalibrationController:
  1. Stop recording --> RecordingManager.stop_recording() --> saves bag
  2. Set mode to position --> RosBridge.set_mode("position") --> motors hold
  3. Calculate threshold:
       threshold_rad = max_position_rad + deg_to_rad(offset_deg)
  4. Create trigger:
       HysteresisTorqueTrigger.create_falling(...)
  5. Save: TriggerStore.add(trigger) --> writes triggers.json
  6. Emit calibration_complete(trigger) signal

CalibrationWindow:
  7. Display result: "Trigger 'Grip Close' saved. Threshold: 0.90 rad"
  8. Re-enable Start button for next calibration
```

## Patterns to Follow

### Pattern 1: Controller/View Separation (from existing MainWindow)

**What:** All logic in a controller/manager object, window just wires signals and updates UI.
**When:** Always for this app.
**Why:** MainWindow already follows this with RecordingManager and RosBridge. The calibration app should do the same.

```python
class CalibrationController(QObject):
    state_changed = pyqtSignal(str)
    position_updated = pyqtSignal(float, float)  # current, max
    calibration_complete = pyqtSignal(object)     # HysteresisTorqueTrigger
    error = pyqtSignal(str)

    def __init__(self, ros_bridge: RosBridge, recording_manager: RecordingManager,
                 trigger_store: TriggerStore):
        ...

    def start_calibration(self, joint_name: str, torque_nm: float,
                          offset_deg: float, name: str): ...
    def stop_calibration(self): ...
    def cancel(self): ...
```

### Pattern 2: RosBridge Ownership (from existing studio/main.py)

**What:** The app entry point creates QApplication, the window creates and owns RosBridge.
**When:** Every standalone app that needs ROS communication.
**Why:** RosBridge manages rclpy lifecycle. Only one RosBridge per process. Window owns it so cleanup happens on close.

```python
# main.py for calibrator
def main():
    app = QApplication(sys.argv)
    # ... theme setup (copy from studio/main.py) ...
    window = CalibrationWindow()
    window.show()
    sys.exit(app.exec())

# CalibrationWindow.__init__()
self._ros_bridge = RosBridge(self)
self._recording_manager = RecordingManager(self)
self._trigger_store = TriggerStore(self._recording_manager.recordings_dir)
self._controller = CalibrationController(
    self._ros_bridge, self._recording_manager, self._trigger_store)
self._ros_bridge.start()
```

### Pattern 3: Frame Recording via Joint State Signal (from MainWindow._on_joint_state)

**What:** Recording frames are fed from the joint_state_received signal handler, not from a separate subscription.
**When:** Any recording during live operation.
**Why:** Ensures the recorded data matches exactly what the app sees. Single source of truth.

```python
def _on_joint_state(self, msg):
    # Update live display
    self._controller.on_joint_state(msg)

    # Controller internally:
    #   if self._state == CALIBRATING:
    #       timestamp_ns = int(time.time() * 1e9)
    #       self._recording_manager.record_frame(msg, timestamp_ns)
    #       self._update_max_position(msg)
```

### Pattern 4: Mode Restore on Exit (from MainWindow._stop_recording)

**What:** Restore motor mode when calibration ends or app closes.
**When:** Always.
**Why:** Leaving motors in torque mode with no controller publishing is dangerous. Motors would hold last effort indefinitely (or until timeout).

```python
def stop_calibration(self):
    # 1. Set efforts to zero FIRST (safety)
    self._send_zero_efforts()
    # 2. Switch to position mode (motors hold current position)
    self._ros_bridge.set_mode("position")
    # 3. Stop recording
    self._recording_manager.stop_recording()
    # 4. Calculate and save
    ...

def closeEvent(self, event):
    # Safety: ensure motors aren't left in torque mode
    if self._controller.state != IDLE:
        self._controller.cancel()
    self._ros_bridge.stop()
    event.accept()
```

## Anti-Patterns to Avoid

### Anti-Pattern 1: Direct ROS Publishing from UI Thread

**What:** Creating ROS publishers in the CalibrationWindow and publishing from button handlers.
**Why bad:** ROS node lives in RosBridge's background thread. Cross-thread ROS access is undefined behavior.
**Instead:** All ROS communication goes through RosBridge methods (which are thread-safe by design -- they call worker methods that publish from the correct thread).

### Anti-Pattern 2: Subclassing RosBridge for Calibration

**What:** Creating `CalibrationRosBridge(RosBridge)` that adds effort publishing.
**Why bad:** RosBridge already provides everything needed via `set_mode("torque")` + `send_joint_command()`. A subclass adds complexity, creates maintenance burden, and diverges from the single RosBridge used everywhere.
**Instead:** Use the existing `send_joint_command()` with effort field populated, after setting mode to "torque".

### Anti-Pattern 3: Polling for Max Position After Stop

**What:** Reading back the recording after stopping to find the max position.
**Why bad:** Requires deserializing the entire bag. Slow, complex, and the data was already available in real-time.
**Instead:** Track max position incrementally during calibration. One float comparison per frame (~500 Hz) is essentially free.

### Anti-Pattern 4: Shared TriggerStore Instance Between Processes

**What:** Trying to share a TriggerStore object between Motor Studio and Calibration App.
**Why bad:** They are separate processes. Object sharing is impossible.
**Instead:** Both processes create their own TriggerStore pointing to the same `triggers.json` file. TriggerStore reads the file on construction, so Motor Studio will see new triggers when it refreshes (which happens on tab switch or explicit refresh). No IPC needed.

### Anti-Pattern 5: Creating a New ROS Node Name

**What:** Using a unique node name like `'calibration_tool'` for the RosBridge.
**Why bad (mild):** Not truly bad, but using `'motor_studio'` (the existing default in RosBridge) means the calibration app and Motor Studio cannot run simultaneously. Since they share the same driver node and the same recordings directory, running both at once is already problematic.
**Instead:** Use a distinct node name `'torque_calibrator'` to allow coexistence if needed. This requires either: (a) passing the node name to RosBridge constructor, or (b) a one-line change in RosBridgeWorker. Given the calibration app is standalone, a small RosBridge enhancement to accept a configurable node name is worthwhile.

## Key Architectural Decision: RosBridge Reuse Strategy

**Decision:** Reuse RosBridge as-is, with one optional enhancement (configurable node name).

**Rationale:**

The calibration app's ROS communication needs are a strict subset of what RosBridge already provides:

| Need | RosBridge Capability | Gap? |
|------|---------------------|------|
| Subscribe to /joint_states | `joint_state_received` signal | No |
| Know joint names | `joint_names` property | No |
| Connection detection | `connection_status_changed` signal | No |
| Set torque mode | `set_mode("torque")` | No |
| Send effort commands | `send_joint_command(msg)` with effort field | No |
| Set position mode (restore) | `set_mode("position")` | No |
| Emergency stop | `emergency_stop()` | No |

The one thing missing is a per-joint effort convenience method, but that is a CalibrationController responsibility (build the JointState message), not a RosBridge responsibility. RosBridge is a transport layer.

**Alternative considered:** Creating a lightweight `CalibrationBridge` that only subscribes to joint states and publishes effort commands. Rejected because:
- Would duplicate the QThread + rclpy lifecycle management (~80 lines)
- Would lose connection detection, mode control, and emergency stop
- No performance benefit (RosBridge's unused subscribers have zero overhead)
- Violates DRY for no gain

## Suggested Build Order

Based on dependency analysis:

```
Phase 1: CalibrationController (core logic, state machine)
    Depends on: RosBridge, RecordingManager, TriggerStore (all existing)
    Can be tested with mock signals

Phase 2: CalibrationWindow (UI shell)
    Depends on: CalibrationController
    Wires signals to widgets

Phase 3: Entry point + integration
    Depends on: CalibrationWindow
    main.py, setup.py entry_points, launch file
```

**Why this order:**

1. **Controller first** because it contains all the logic and can be validated independently. The state machine transitions, max position tracking, and trigger creation are testable without UI.

2. **Window second** because it is purely a signal/slot wiring exercise once the controller exists. Every widget maps to a controller signal or method.

3. **Entry point last** because it is boilerplate (copy from `studio/main.py`, change window class).

**Within Phase 1, sub-ordering:**

```
1a. Effort command construction (how to build JointState for torque mode)
1b. Max position tracking (extract position from JointState, track max)
1c. State machine (IDLE -> PREPARING -> CALIBRATING -> CALCULATING -> IDLE)
1d. Threshold calculation + trigger creation
1e. Recording integration (feed frames to RecordingManager)
```

Steps 1a and 1b are independent and can be built in any order. 1c depends on understanding 1a and 1b. 1d is pure computation (no dependencies). 1e plugs into the state machine from 1c.

## File Placement

```
myactuator_python_driver/
  myactuator_python_driver/
    calibrator/                    # New package directory
      __init__.py
      main.py                     # Entry point (QApplication, theme, window)
      calibration_window.py       # CalibrationWindow (QMainWindow)
      calibration_controller.py   # CalibrationController (QObject, state machine)
```

Not under `studio/` because the calibrator is a separate application, not a Motor Studio component. It imports from `studio.ros_bridge`, `studio.recording_manager`, and `config` but is not part of the studio package hierarchy.

**setup.py entry point:**

```python
'console_scripts': [
    ...
    'calibrator = myactuator_python_driver.calibrator.main:main',
],
```

**Launch:**

```bash
ros2 run myactuator_python_driver calibrator
```

## Thread Model

```
Main Thread (Qt Event Loop)
    |
    +-- CalibrationWindow (all UI updates)
    |
    +-- CalibrationController (signal handlers, state transitions)
    |       |
    |       +-- RecordingManager.record_frame() (called from signal handler,
    |           writes to bag -- fast, <1ms per frame)
    |
Background Thread (QThread, managed by RosBridge)
    |
    +-- RosBridgeWorker (ROS node, executor spin)
            |
            +-- joint_state callback --> emit signal to main thread
            +-- publishers (called from main thread via worker methods)
```

All Qt signal/slot connections between RosBridgeWorker (background thread) and CalibrationController (main thread) are automatically queued by Qt, ensuring thread safety without manual locking.

## Sources

All findings based on direct analysis of the existing codebase:

- `ros_bridge.py` -- RosBridge interface and threading model
- `main_window.py` -- Existing signal wiring patterns, recording flow
- `recording_manager.py` -- Recording API and frame capture
- `driver_node.py` -- Mode control, effort handling, trigger evaluation
- `config.py` -- HysteresisTorqueTrigger, TriggerStore, PlaybackTriggerConfig
- `trigger_dialog.py` -- Existing trigger creation UI patterns
- `studio/main.py` -- Application entry point pattern

---

*Architecture analysis: 2026-02-09*
