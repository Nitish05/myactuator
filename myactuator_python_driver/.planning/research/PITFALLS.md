# Domain Pitfalls

**Domain:** Torque threshold calibration tool for ROS 2 motor control
**Researched:** 2026-02-09
**Confidence:** HIGH (based on direct codebase analysis, not training data)

## Critical Pitfalls

Mistakes that cause hardware damage, safety incidents, or fundamental architectural rework.

### Pitfall 1: Motor Jumps to Position Zero on Mode Transition

**What goes wrong:** When switching from free mode to torque mode (the core calibration workflow), the driver captures current positions as setpoints. But if the calibration app also publishes a torque command before the position capture completes, the non-selected joints may briefly receive stale position commands, causing them to jerk toward their last commanded position (possibly zero).

**Why it happens:** The driver's `_set_mode_callback` (line 520-599 in `driver_node.py`) calls `_capture_positions_locked()` when transitioning from free/admittance to torque mode. This reads current motor positions sequentially over CAN. If the calibration app publishes effort commands to `/joint_effort_ctrl` during this window, the `_effort_ctrl_callback` sets `_torque_mode = True` (line 434) globally -- all joints enter torque mode simultaneously, not just the selected one.

**Consequences:**
- Physical hardware damage: a joint snapping to position zero under full torque
- Operator injury if working near the robot during calibration
- Mechanical stress on gear reducers from sudden high-torque motion

**Prevention:**
1. Never use `/joint_effort_ctrl` topic for the calibration app. That topic sets ALL joints to torque mode globally.
2. Instead, use the `~/set_mode` topic to switch to "free" mode for the entire robot, then use the per-joint torque trigger mechanism (`~/playback_triggers`) to apply torque to only the selected joint. This is exactly what the playback system does -- it switches individual joints between position and torque control.
3. Alternative: Send a "torque" mode command, then immediately publish effort commands with zero torque for non-selected joints and the desired torque for the selected joint. But this is fragile -- a missed message means uncontrolled joints.

**Detection:** During development, test mode transitions with the robot in a safe configuration (e.g., joints not at zero, so a jump to zero would be visible). Monitor `/joint_states` for sudden position changes during mode switches.

**Phase:** Must be addressed in the architecture/design phase (Phase 1). The choice of how to coordinate with the driver determines the entire control flow of the app.

---

### Pitfall 2: Two GUI Apps Publishing Conflicting Mode Commands

**What goes wrong:** Motor Studio and the Calibration App both publish to `/motor_driver/set_mode`. If both are running, one app can override the other's mode. The operator starts calibration (sets torque on one joint), then Motor Studio's playback or mode selector sends "position" mode, and the torqued joint suddenly enters position control -- jerking to the last commanded position.

**Why it happens:** ROS 2 pub/sub is broadcast. The driver has a single `_set_mode_callback` subscriber on `~/set_mode` (line 127-130 in `driver_node.py`). There is no concept of "who" sent the mode command. Both `RosBridge` (line 112-113 in `ros_bridge.py`) and `RecorderTUI` (line 154 in `recorder_tui.py`) publish to the same `set_mode` topic.

**Consequences:**
- Unexpected motor behavior during active torque calibration
- Loss of operator trust in the calibration tool's behavior
- Potential hardware damage if mode switch happens during torque application

**Prevention:**
1. Display a clear warning in the Calibration App: "Close Motor Studio before running calibration" or detect if Motor Studio is already publishing and refuse to start.
2. Detect conflicting publishers by subscribing to `/motor_driver/mode` and checking if the mode unexpectedly changes during calibration. If it does, immediately stop calibration and alert the operator.
3. Consider using a ROS 2 service call for mode changes instead of a topic. Services are request/response, making the caller explicit. But this requires driver changes, so it is a longer-term fix.
4. At minimum: the calibration app should own the mode lifecycle during calibration. On start, set mode; on any unexpected mode change, abort and emergency-stop.

**Detection:** Subscribe to `~/mode` topic and compare received mode against expected mode. If they diverge during an active calibration, the app has lost control of the driver.

**Phase:** Must be addressed in Phase 1 (architecture). The conflict detection and safety-abort mechanism should be designed before writing any motor control code.

---

### Pitfall 3: No Torque Limit Enforcement in the Calibration App

**What goes wrong:** The user enters an arbitrary torque value (e.g., 50 Nm) in the calibration UI, which exceeds the motor's rated torque (e.g., X4 series is rated for 1 Nm, X8 series for 9 Nm). The motor draws excessive current, overheats, or the gear reducer fails under sustained load.

**Why it happens:** The existing `TriggerDialog` (line 127-133 in `trigger_dialog.py`) allows torque from -50 to +50 Nm with no motor-specific validation. The `MotorWrapper.send_torque()` passes torque directly to `ActuatorInterface.sendTorqueSetpoint()` with no clamping. The motor firmware has its own current limit, but it may not protect the gear reducer from sustained over-torque.

**Consequences:**
- Motor overheating (motor thermal protection may or may not activate in time)
- Gear reducer damage from sustained torque exceeding rated specifications
- Encoder damage from mechanical overload
- Potential fire hazard from sustained overcurrent

**Prevention:**
1. Load the motor's rated torque from `actuator_constants.hpp` values (already available in `config.py` as `MOTOR_TORQUE_CONSTANTS`). Each motor model has a `rated_torque` value.
2. Set the torque input spinbox maximum to the motor's rated torque (or a configurable percentage of it, like 80%).
3. Display the motor's rated torque next to the input field so the operator can make an informed choice.
4. Add a timeout: if calibration has been applying torque for more than N seconds (e.g., 30s), automatically stop. This prevents sustained thermal loading if the operator walks away.

**Detection:** Monitor motor temperature via `/motor_status` topic (temperature is published). If temperature rises above 60C during calibration, auto-stop and warn.

**Phase:** Phase 1 (UI design and safety constraints). The torque input limits should be defined at design time, not bolted on later.

---

### Pitfall 4: Recording While Applying Torque Creates a Misleading Recording

**What goes wrong:** The calibration app records a ROS bag while applying torque to one joint. The recorded `/joint_states` shows the joint moving under torque -- but the position trajectory is NOT the same as what a normal "free" recording would capture. If this recording is later played back in Motor Studio, the driver will try to follow these positions, which include the torque-induced motion. The playback system expects recordings made in free mode (operator moving joints by hand).

**Why it happens:** In the normal workflow (RecorderTUI line 284), recording is done in "free" mode -- the operator moves joints, and the positions are captured. Playback replays those positions. The trigger system then overrides specific joints at specific positions. But the calibration recording captures the joint reaching its torque-limited position -- a fundamentally different kind of trajectory.

**Consequences:**
- The auto-created trigger's enter_threshold_rad is based on the max position under torque. But if this recording is played back, the position trajectory already includes the torque motion. The trigger may fire at the wrong time, or the position/torque interaction may cause oscillation.
- Operator confusion: "I recorded a calibration, why does playback behave differently?"

**Prevention:**
1. Clearly separate calibration recordings from normal recordings. Use a naming convention (e.g., prefix with `calibration_`) or store in a separate directory.
2. The calibration recording should NOT be directly playable in Motor Studio. It should be marked as a "calibration capture" in metadata.
3. Better yet: the calibration app should pair the auto-created trigger with the OPERATOR'S recording (the one made in free mode), not with the calibration recording itself. The calibration recording is just a measurement tool to determine the threshold.
4. Clarify the workflow: (1) operator records in free mode via Motor Studio, (2) operator runs calibration to discover the torque threshold, (3) calibration app creates a trigger paired with the free-mode recording.

**Detection:** If the calibration app auto-creates a trigger with `recording_name` set to the calibration recording name, the trigger will only activate during playback of that specific recording -- which is useless because that recording already includes the torque motion.

**Phase:** Phase 1 (workflow design). This is a fundamental design question about what the calibration output is used for.

---

### Pitfall 5: CAN Bus Contention from Simultaneous Read and Command

**What goes wrong:** The driver's control loop reads motor state (`getMotorStatus2()`) and sends commands (`sendTorqueSetpoint()` or `sendPositionAbsoluteSetpoint()`) for each motor sequentially at 500 Hz. If the calibration app also makes CAN requests (e.g., direct motor queries), the CAN bus becomes saturated and commands are dropped or delayed.

**Why it happens:** All motors share a single CAN bus at 1 Mbps. Each command/response cycle is 8+8 bytes = 128 bits, taking ~128 microseconds on wire. With 8 motors and both read and command per motor, that is 16 CAN transactions per loop iteration = ~2ms. At 500 Hz (2ms period), the bus is at 100% utilization with just the driver. Any additional traffic causes missed frames.

**Consequences:**
- CAN timeout errors logged by the driver (throttled to every 5 seconds, so easily missed)
- Motor holding stale position/torque commands, leading to delayed or jerky motion
- The calibration app's "live position" display may be stale by 10-100ms

**Prevention:**
1. The calibration app must NOT create its own CAN driver or ActuatorInterface. It must communicate exclusively through ROS 2 topics, using the existing driver node as the sole CAN bus owner.
2. The existing `RosBridge` already provides this: subscribe to `/joint_states` for live positions, publish to `~/set_mode` for mode changes, publish trigger config for per-joint torque.
3. Do NOT add "direct motor query" functionality to the calibration app. All motor data comes through the driver node.

**Detection:** If the calibration app imports `myactuator_rmd_py` or creates `CanDriver` instances, that is a design violation.

**Phase:** Phase 1 (architecture). The calibration app should be a pure ROS 2 client with no direct hardware access.

## Moderate Pitfalls

### Pitfall 6: Position Capture Race Condition During Free-to-Torque Transition

**What goes wrong:** The driver captures current positions when exiting free mode (`_capture_positions_locked()`, line 718-727 in `driver_node.py`). This reads each motor sequentially via CAN. If a motor is still in motion (being released from free mode), the captured position may not reflect the actual resting position.

**Prevention:**
1. Before switching from free to torque mode, wait for joint velocities to drop below a threshold (e.g., 0.01 rad/s) for all joints.
2. Subscribe to `/joint_states` and check velocity fields before issuing the mode change.
3. Add a small delay (100-200ms) between the mode change request and the first torque command to allow positions to stabilize.

**Phase:** Phase 2 (calibration control flow). This is an implementation detail of the calibration sequence.

---

### Pitfall 7: Max Position Tracking Misses the Actual Maximum

**What goes wrong:** The calibration app tracks "max position reached" during torque application. If the joint state update rate (500 Hz) is faster than the GUI update rate (typically 20 Hz for Qt), the GUI shows an old max. More critically, if the max position is reached between two `/joint_states` samples, the actual peak is missed.

**Prevention:**
1. Track max position in the ROS callback (or signal handler), not in the GUI timer. Every `/joint_states` message should update the max.
2. Use a thread-safe variable (e.g., `threading.Lock` protected float) for the max position.
3. For the threshold calculation, consider using a rolling average of the top N% of positions rather than the single absolute maximum. This is more robust to noise.

**Phase:** Phase 2 (implementation). The max tracking logic must be in the data path, not the display path.

---

### Pitfall 8: Hysteresis Offset Too Small or Inverted

**What goes wrong:** The auto-calculated trigger uses `max_position + offset` as the enter threshold and `max_position + offset - hysteresis` as the exit threshold. If the offset is too small (e.g., 0.5 degrees = 0.0087 rad), the trigger may fire prematurely during playback due to position noise. If the hysteresis gap is too small, the trigger oscillates between active and inactive.

**Why it happens:** The default offset of 0.5 degrees is based on user input, but position noise in the motor feedback can be 0.1-0.3 degrees. A threshold within the noise floor means random trigger activations.

**Prevention:**
1. Set minimum offset to at least 2x the expected position noise (typically 1-2 degrees for RMD motors, or 0.02-0.035 rad).
2. Set minimum hysteresis to at least 3x the position noise.
3. During calibration, measure the position noise (standard deviation over a 1-second window) and use it to set a sensible minimum offset.
4. Display the offset in both degrees and radians in the UI, since users think in degrees but the system uses radians.

**Phase:** Phase 2 (implementation). The offset/hysteresis defaults and validation belong in the threshold calculation logic.

---

### Pitfall 9: Emergency Stop Not Accessible During Calibration

**What goes wrong:** The calibration app is a standalone PyQt6 window. During active torque application, if something goes wrong (unexpected motor behavior, collision, operator needs to intervene), the only e-stop is ESC in Motor Studio or the RecorderTUI -- but those might not be running.

**Prevention:**
1. Include a prominent EMERGENCY STOP button in the calibration app (same pattern as `control_panel.py` line 137-157: large, red, always visible).
2. Bind the ESC key to emergency stop in the calibration app.
3. On e-stop: call the `/motor_driver/emergency_stop` service AND set mode to "disabled" AND stop recording AND zero out all effort commands.
4. E-stop must be a service call (not just a topic publish) because service calls are reliable (acknowledged), while topic publishes are fire-and-forget.

**Detection:** If the calibration app's UI does not have a visually prominent e-stop button, it fails the safety review.

**Phase:** Phase 1 (UI design). The e-stop button should be the first thing designed, not the last.

---

### Pitfall 10: RosBridge Creates Duplicate rclpy.init() Call

**What goes wrong:** The calibration app reuses `RosBridge`, which calls `rclpy.init()` in `start_ros()` (line 69-70 in `ros_bridge.py`). If the calibration app or its test harness also calls `rclpy.init()`, the second call throws `rclpy.exceptions.RCLError: context has already been initialized`. This crashes the app.

**Why it happens:** `rclpy.init()` is process-global. The `RosBridgeWorker` guards with `if not rclpy.ok()`, but this check is racy -- another thread might init between the check and the call.

**Prevention:**
1. Initialize `rclpy` exactly once at the application entry point (in `main()` of the calibration app), before creating any ROS objects.
2. Modify the app's `RosBridge` usage to skip `rclpy.init()` if already initialized, or let the app's main function handle it.
3. Use a try/except around `rclpy.init()` to catch double-init gracefully.

**Phase:** Phase 1 (app scaffolding). This is the first thing that must work correctly.

## Minor Pitfalls

### Pitfall 11: Calibration App and Motor Studio Share TriggerStore File

**What goes wrong:** Both apps read/write `triggers.json` via `TriggerStore`. If both are open, writes from one overwrite the other's changes without conflict detection.

**Prevention:** TriggerStore should reload from disk before writing (compare mtime or use file locking). The calibration app should call `TriggerStore.load()` before `TriggerStore.add()`.

**Phase:** Phase 2 (implementation). A reload-before-write pattern is sufficient.

---

### Pitfall 12: Falling Trigger Direction Assumption May Not Hold

**What goes wrong:** The calibration app assumes all triggers are "falling" (activate when position drops below threshold). This matches grip/close motions. But if calibrating a joint that opens under torque (torque pushes joint upward), the threshold should be a "rising" trigger.

**Prevention:**
1. Auto-detect direction: compare the joint's position before and after torque application. If position increased, create a rising trigger. If decreased, create a falling trigger.
2. At minimum, display the direction and let the user confirm or override.

**Phase:** Phase 2 (implementation). The PROJECT.md currently scopes this as "only falling triggers," which is acceptable for MVP but should be reconsidered.

---

### Pitfall 13: Bag Recording Starts Before Mode Change Takes Effect

**What goes wrong:** The calibration sequence is: (1) set free mode, (2) set torque on selected joint, (3) start recording. If recording starts before the mode change propagates through the ROS system, the first few frames capture the joint at rest (not yet under torque). This slightly shifts the max position window.

**Prevention:** After sending the mode change, wait for the `~/mode` topic to confirm the change before starting the recording. The `RosBridge.mode_changed` signal provides this confirmation.

**Phase:** Phase 2 (implementation). A simple "wait for mode confirmation" guard.

---

### Pitfall 14: Units Confusion Between Degrees and Radians in UI

**What goes wrong:** The driver uses radians internally, the motor protocol uses degrees (0.01 degree resolution), and operators typically think in degrees. The offset parameter (default 0.5 degrees) is specified in degrees but must be converted to radians for the threshold calculation. If the conversion is missed, the offset is 0.5 rad (~28.6 degrees) instead of 0.5 degrees (~0.0087 rad).

**Prevention:**
1. Label all input fields with explicit units: "Offset (degrees):" or "Threshold (rad):"
2. Perform all internal calculations in radians (consistent with driver and `HysteresisTorqueTrigger` fields).
3. Convert user-facing values at the UI boundary, nowhere else.
4. Add a unit test that verifies the threshold calculation produces values in the expected range.

**Phase:** Phase 2 (implementation). A simple but easily missed bug.

## Phase-Specific Warnings

| Phase Topic | Likely Pitfall | Mitigation |
|-------------|---------------|------------|
| Architecture (Phase 1) | Mode conflict with Motor Studio (#2) | Design conflict detection and safety-abort into the core architecture |
| Architecture (Phase 1) | Direct CAN access (#5) | Enforce "ROS topics only" rule from day one |
| Architecture (Phase 1) | Recording purpose confusion (#4) | Decide workflow before writing code: calibration produces triggers, not playable recordings |
| UI Design (Phase 1) | No e-stop (#9) | E-stop button is the first widget to add, not the last |
| UI Design (Phase 1) | Torque limits (#3) | Wire torque input to motor specs at design time |
| Control Flow (Phase 2) | Mode jump on transition (#1) | Use per-joint trigger mechanism, not global torque mode |
| Control Flow (Phase 2) | Position settle before torque (#6) | Wait for velocity < threshold before applying torque |
| Threshold Calc (Phase 2) | Max tracking in wrong thread (#7) | Track max in ROS callback, display in GUI |
| Threshold Calc (Phase 2) | Offset/hysteresis too small (#8) | Set minimums based on motor noise characteristics |
| Integration (Phase 3) | rclpy double init (#10) | Initialize once at entry point |
| Integration (Phase 3) | TriggerStore file conflict (#11) | Reload before write |

## Sources

All findings are based on direct analysis of the following files in the repository:

- `myactuator_python_driver/myactuator_python_driver/driver_node.py` -- driver control loop, mode transitions, trigger evaluation
- `myactuator_python_driver/myactuator_python_driver/motor_wrapper.py` -- CAN communication, unit conversion
- `myactuator_python_driver/myactuator_python_driver/config.py` -- HysteresisTorqueTrigger, TriggerStore, motor constants
- `myactuator_python_driver/myactuator_python_driver/recorder_tui.py` -- recording/playback patterns
- `myactuator_python_driver/myactuator_python_driver/studio/ros_bridge.py` -- ROS 2 communication layer
- `myactuator_python_driver/myactuator_python_driver/studio/recording_manager.py` -- bag recording/playback
- `myactuator_python_driver/myactuator_python_driver/studio/dialogs/trigger_dialog.py` -- existing trigger creation UI
- `myactuator_python_driver/myactuator_python_driver/studio/widgets/control_panel.py` -- e-stop pattern
- `myactuator_rmd/include/myactuator_rmd/actuator_constants.hpp` -- motor rated torque values
- `.planning/codebase/ARCHITECTURE.md` -- system architecture documentation
- `.planning/codebase/CONCERNS.md` -- known tech debt and fragile areas

Confidence: HIGH. All pitfalls are derived from specific code paths and data flows in the existing codebase, with line-number references. No external sources or training-data-only claims.

---

*Pitfalls analysis: 2026-02-09*
