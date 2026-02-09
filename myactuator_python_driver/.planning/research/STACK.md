# Technology Stack

**Project:** Torque Threshold Calibration Tool
**Researched:** 2026-02-09

## Recommended Stack

**Summary:** Reuse 90% of existing infrastructure. The calibration app is a thin orchestrator on top of RosBridge, RecordingManager, and TriggerStore -- zero new dependencies needed.

### Core Framework (all existing)

| Technology | Version | Purpose | Why |
|------------|---------|---------|-----|
| PyQt6 | >=6.4.0 | GUI framework | Already used by Motor Recording Studio; shared dark theme, styling patterns, widget patterns all transfer directly |
| rclpy | ROS 2 (Jazzy/Humble) | ROS 2 communication | Driver node already exposes all needed topics/services; no custom message types needed |
| rosbag2_py | ROS 2 distro-matched | Bag recording | RecordingManager already handles Jazzy/Humble API differences |
| Python 3 | 3.10+ | Runtime | Matches existing workspace |

### Existing Components to Reuse (CRITICAL)

| Component | Location | What It Provides | Reuse Strategy |
|-----------|----------|-----------------|----------------|
| `RosBridge` | `studio/ros_bridge.py` | Qt-threadsafe ROS 2 pub/sub/srv, mode control, joint state streaming, trigger config publishing | **Use directly as-is.** Instantiate a new RosBridge in the calibration window. Provides `set_mode()`, `send_joint_command()`, `set_trigger_config()`, `joint_state_received` signal, `connection_status_changed` signal. |
| `RecordingManager` | `studio/recording_manager.py` | Bag writing, bag reading, recording listing, frame recording | **Use directly as-is.** Call `start_recording()`, `record_frame()`, `stop_recording()`. The calibration app feeds frames from RosBridge's `joint_state_received` signal. |
| `TriggerStore` | `config.py` | Persistent JSON storage of `HysteresisTorqueTrigger` objects | **Use directly as-is.** After calibration discovers the threshold, call `TriggerStore.add(trigger)`. |
| `HysteresisTorqueTrigger` | `config.py` | Trigger data model with `create_falling()` factory method | **Use directly.** The calibration result is exactly one `HysteresisTorqueTrigger` created via `create_falling(name, joint_name, threshold_rad, torque_nm)`. |
| `PlaybackTriggerConfig` | `config.py` | Serialization wrapper for triggers | **Use directly** when sending triggers to driver via RosBridge. |
| `JointStateModel` | `studio/models/joint_state_model.py` | Qt table model for joint state display | **Use directly** if the calibration app needs a joint monitor. |
| Dark theme / Fusion palette | `studio/main.py` | Consistent look with Motor Studio | **Copy the theme setup pattern** (15 lines) into the calibration app's entry point. |

### New Components to Build

| Component | Purpose | Complexity | Notes |
|-----------|---------|------------|-------|
| `CalibrationWindow` (QMainWindow) | Main calibration UI: joint selector, torque input, start/stop, results display | Medium | Single-window app, no tabs needed. Simpler than MainWindow. |
| `CalibrationOrchestrator` (QObject) | State machine coordinating the calibration sequence: set modes, apply torque, record, track max position, create trigger | Medium | This is the core new logic. ~150-200 lines. |
| `MaxPositionTracker` | Tracks maximum position for the calibrated joint during recording | Low | Simple class: receives JointState, tracks `max(position)` for a named joint. ~20 lines. |
| Entry point `calibrate_tool` | Console script launching the calibration app | Low | Same pattern as `motor_studio` entry point in setup.py. |

### Database

No database needed. Existing `TriggerStore` uses JSON file storage (`recordings/triggers.json`). This is sufficient -- the calibration tool writes to the same store that Motor Studio reads.

### Infrastructure

No new infrastructure. The calibration tool connects to the already-running driver node via ROS 2 topics. No new topics, services, or message types are required.

## ROS 2 Communication Pattern

The calibration workflow uses exclusively existing topics and modes:

### Topics Used (all existing)

| Topic | Direction | Type | Purpose in Calibration |
|-------|-----------|------|----------------------|
| `/joint_states` | Subscribe | `sensor_msgs/JointState` | Read real-time position during calibration to track max |
| `/joint_state_ctrl` | Publish | `sensor_msgs/JointState` | Not used directly (torque mode bypasses position control) |
| `/motor_driver/set_mode` | Publish | `std_msgs/String` | Set "free" for non-calibrated joints, "torque" for calibrated joint |
| `/motor_driver/mode` | Subscribe | `std_msgs/String` | Confirm mode changes |
| `/motor_driver/set_enabled` | Publish | `std_msgs/Bool` | Enable/disable motors |
| `/motor_driver/playback_triggers` | Publish | `std_msgs/String` | Not needed during calibration (triggers are post-calibration output) |

### Calibration Sequence (ROS 2 operations)

1. **Connect:** RosBridge starts, subscribes to `/joint_states`, confirms connection
2. **Set free mode:** `RosBridge.set_mode("free")` -- all joints free to move
3. **Start recording:** `RecordingManager.start_recording("calibration_...")`
4. **Apply torque:** Publish `JointState` with effort for the selected joint via `/joint_state_ctrl` while driver is in torque mode -- OR switch to torque mode and use `_control_efforts` on the driver
5. **Track max position:** `MaxPositionTracker` monitors `joint_state_received` signal
6. **Stop recording:** `RecordingManager.stop_recording()`
7. **Compute threshold:** `max_position + offset`
8. **Create trigger:** `HysteresisTorqueTrigger.create_falling(...)` + `TriggerStore.add()`

### Key Driver Mode Insight

The driver node's mode system is per-robot, not per-joint. But the calibration needs mixed-mode behavior (one joint in torque, others free). Two approaches:

**Approach A (Recommended): Use existing torque mode with selective effort commands.**
Set mode to "torque" via `set_mode("torque")`. Then publish `JointState` on `/joint_state_ctrl` with effort values: target torque for the calibrated joint, 0.0 Nm for all others. The driver's `_effort_ctrl_callback` applies per-joint efforts. Zero effort on a joint makes it effectively free (zero current = no holding force).

This works because:
- The `send_torque(0.0)` call in the driver sends zero current, which releases the motor (same as `release()`)
- The driver already supports per-joint effort via `_control_efforts` dict
- No driver modifications needed

**Approach B (Fallback): Direct effort publishing.**
Use the `/joint_effort_ctrl` topic (`Float64MultiArray`) which sets `_torque_mode = True` and applies per-joint effort values. Same zero-effort-for-free behavior.

Approach A is cleaner because it uses the same `JointState` message pattern as the rest of the Studio infrastructure.

## Alternatives Considered

| Category | Recommended | Alternative | Why Not |
|----------|-------------|-------------|---------|
| GUI Framework | PyQt6 (existing) | Separate CLI tool | GUI provides real-time position feedback during calibration, which is essential for verifying the threshold makes sense |
| App Architecture | Standalone window (new QMainWindow) | New tab in Motor Studio | Standalone is cleaner: calibration is a one-time setup task, not a daily workflow. Adding a tab would bloat Motor Studio. Can be launched independently. |
| ROS Communication | Reuse RosBridge | Direct rclpy node | RosBridge already solves Qt/ROS thread safety. Duplicating this would be error-prone and wasteful. |
| Position Tracking | Simple max tracker on `joint_state_received` | Post-process bag after recording | Real-time tracking gives immediate feedback to the user. Post-processing adds delay and complexity. |
| Trigger Storage | Existing TriggerStore (JSON) | New storage format | TriggerStore is what Motor Studio reads. Using the same store means zero integration work. |
| Per-joint mixed mode | Torque mode + zero effort | New per-joint mode in driver | Driver modification would be a larger change. Zero-effort-as-free is already how the driver works. |

## New Dependencies

**None.** Every dependency is already in `setup.py`:

```
install_requires=[
    'setuptools',
    'rich',
    'PyQt6>=6.4.0',
    'qt-material>=2.14',
]
```

The only change needed is adding a new `console_scripts` entry point in `setup.py`:

```python
entry_points={
    'console_scripts': [
        # ... existing entries ...
        'calibrate_tool = myactuator_python_driver.studio.calibrate:main',
    ],
},
```

## File Organization

New files within the existing `studio/` package:

```
myactuator_python_driver/studio/
    calibrate.py              # Entry point (main function, QApplication setup)
    calibration_window.py     # CalibrationWindow QMainWindow
    calibration_orchestrator.py  # State machine for calibration sequence
```

This mirrors the existing pattern: `main.py` (entry) + `main_window.py` (window) for Motor Studio.

## Installation

No new packages to install. Build as usual:

```bash
colcon build --packages-select myactuator_python_driver
source install/setup.bash
ros2 run myactuator_python_driver calibrate_tool
```

## Confidence Assessment

| Area | Confidence | Notes |
|------|------------|-------|
| Reuse of RosBridge | HIGH | Read the source; all needed methods exist (`set_mode`, `joint_state_received`, `send_joint_command`) |
| Reuse of RecordingManager | HIGH | Read the source; `start_recording`/`record_frame`/`stop_recording` API matches calibration needs exactly |
| Reuse of TriggerStore | HIGH | Read the source; `add()` method is all that is needed |
| Zero-effort-as-free strategy | HIGH | Confirmed in `motor_wrapper.py`: `release()` calls `sendCurrentSetpoint(0.0)`, and `send_torque(0.0)` does the same via `sendTorqueSetpoint` |
| No new topics/services needed | HIGH | All communication paths exist; calibration is an orchestration of existing primitives |
| No new dependencies | HIGH | Verified `setup.py`; everything needed is already listed |

## Sources

- `/home/nitish/work/myactuator/myactuator_python_driver/myactuator_python_driver/studio/ros_bridge.py` (RosBridge API)
- `/home/nitish/work/myactuator/myactuator_python_driver/myactuator_python_driver/studio/recording_manager.py` (RecordingManager API)
- `/home/nitish/work/myactuator/myactuator_python_driver/myactuator_python_driver/config.py` (TriggerStore, HysteresisTorqueTrigger, PlaybackTriggerConfig)
- `/home/nitish/work/myactuator/myactuator_python_driver/myactuator_python_driver/driver_node.py` (MotorDriverNode mode system, topic/service definitions)
- `/home/nitish/work/myactuator/myactuator_python_driver/myactuator_python_driver/motor_wrapper.py` (send_torque, release, zero-current behavior)
- `/home/nitish/work/myactuator/myactuator_python_driver/myactuator_python_driver/studio/main.py` (app bootstrap pattern)
- `/home/nitish/work/myactuator/myactuator_python_driver/myactuator_python_driver/studio/main_window.py` (MainWindow composition pattern)
- `/home/nitish/work/myactuator/myactuator_python_driver/setup.py` (entry points, dependencies)
