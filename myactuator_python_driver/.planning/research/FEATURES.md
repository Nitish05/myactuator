# Feature Landscape

**Domain:** Torque threshold calibration tool for robotics motor control
**Researched:** 2026-02-09
**Overall confidence:** HIGH (based on deep analysis of existing codebase, domain patterns from motor tuning/calibration tools, and the specific workflow described in PROJECT.md)

## Table Stakes

Features the user expects. Missing any of these makes the tool useless for its stated purpose.

| Feature | Why Expected | Complexity | Notes |
|---------|--------------|------------|-------|
| Joint selector dropdown | User must choose which joint to calibrate. Multi-joint robots always need joint selection. | Low | Populate from `RosBridge.joint_names` on connection. Disable during recording. |
| Torque input (Nm) | The entire point is applying a known torque to discover the threshold. | Low | QDoubleSpinBox with reasonable range. Existing `TriggerDialog` uses -50 to +50 Nm range. |
| Offset input (degrees) | Threshold = max_position + offset. Without configurable offset, the trigger fires at the exact extreme of travel, leaving zero margin. | Low | Default 0.5 degrees (converted to rad internally). Small positive value adds safety margin above max position. |
| Record button with start/stop toggle | User must control when torque application begins and ends. Manual stop is explicitly in scope. | Low | Toggle button that changes state. Reuse `RecordingManager.start_recording()` / `stop_recording()`. |
| Apply torque to selected joint during recording | Core behavior: the tool must actually command torque on the selected joint while recording. Without this, it is just a recorder. | Medium | Set selected joint to torque mode via `RosBridge.set_mode("torque")` then publish effort command. Other joints set to free mode. Requires per-joint mode control -- current driver only supports one global mode, so need to use the existing per-joint torque trigger approach or send torque commands directly via `send_joint_command`. |
| Set other joints to free mode during recording | If other joints are held in position mode while one joint is torqued, the robot fights itself. Other joints must be free to move. | Medium | The driver node has global mode control. Setting mode to "free" makes all joints free, then send torque command for the selected joint only via `/joint_effort_ctrl` or per-joint command. Alternatively, switch to torque mode globally but only send non-zero torque to the selected joint. |
| Track max position of torqued joint during recording | The threshold is derived from max position. Without tracking, the tool cannot calculate the result. | Low | Subscribe to `/joint_states`, track `max(abs(position))` for the selected joint. Reset on recording start. |
| Display live joint position during recording | User must see what is happening to know when to stop. Blind torque application with no feedback is dangerous. | Low | Label or readout showing current position of selected joint, updated at ~20Hz from `joint_state_received` signal. |
| Display max-position-so-far during recording | User needs to see the peak value accumulating to judge convergence (has the joint stopped moving further?). | Low | Second label showing running max, updated alongside live position. |
| Auto-create HysteresisTorqueTrigger on stop | The output of the calibration is a trigger. If the user has to manually copy numbers and create the trigger elsewhere, the tool has failed its purpose. | Medium | On stop: compute `threshold = max_position + offset_rad`, create `HysteresisTorqueTrigger.create_falling(...)` with the recording name, save to `TriggerStore`. |
| Pair trigger with the recording | Triggers are recording-specific. The trigger created by calibrating joint X with torque Y only makes sense for the recording that was captured during that calibration run. | Low | Set `recording_name` field on the created trigger to match the bag name. Already supported by `HysteresisTorqueTrigger.recording_name`. |
| Emergency stop | Applying torque to a robot is inherently dangerous. E-stop must be accessible at all times, one click or keypress away. | Low | Reuse `RosBridge.emergency_stop()`. Bind to Escape key. Prominent red button, always visible. |
| Connection status indicator | User must know if the driver is connected before attempting calibration. Silently failing is unacceptable when motors are involved. | Low | Reuse `RosBridge.connection_status_changed` signal. Show connected/disconnected in status bar or header. |
| Recording name input | The recording needs a name so the trigger can be paired with it and it can be identified in Motor Studio later. | Low | QLineEdit with auto-generated default (timestamp-based). Editable before starting. |

## Differentiators

Features that improve the experience but are not strictly required for the tool to fulfill its purpose.

| Feature | Value Proposition | Complexity | Notes |
|---------|-------------------|------------|-------|
| Pre-recording position display (all joints) | Shows user the starting position of all joints before calibration. Helps verify the robot is in the expected configuration. | Low | Table or list of joint names with current positions, updated live before recording starts. Reuse `JointMonitor` widget or simplified version. |
| Computed threshold preview before saving | After stopping, show the user what trigger will be created (threshold value, hysteresis gap, torque) and let them confirm or discard. | Low | Confirmation dialog or inline preview with "Save Trigger" / "Discard" buttons. Prevents accidental bad triggers. |
| Calibration history log | Show a list of completed calibrations in the current session with their parameters and results (joint, torque, max position, threshold). | Medium | QListWidget or QTableWidget. Useful when calibrating multiple joints in sequence. Persists only for session. |
| Position chart/graph during recording | Visual plot of joint position over time shows trajectory shape, whether the joint has settled, and any oscillation. Much more informative than a number. | Medium | Use pyqtgraph or matplotlib embedded widget. Plot position of selected joint over time. Optional: also plot the threshold line once recording stops. |
| Torque direction indicator | Show whether the applied torque will move the joint in the positive or negative direction. Prevents applying torque the wrong way. | Low | Text label or arrow showing expected motion direction based on torque sign. Simple sign check. |
| Multiple calibration runs comparison | Allow the user to run calibration multiple times and compare results (max positions, consistency). Helps validate that the threshold is repeatable. | High | Requires storing intermediate results, comparison UI. Over-engineering for an initial release but valuable later. |
| Auto-name trigger from joint and recording | Generate a descriptive trigger name like "elbow_grasp_trigger" from joint name and recording name, instead of requiring manual naming. | Low | String formatting: `f"{joint_name}_calibration_{recording_name}"`. User can override. |
| Return-to-start after calibration | After stopping recording, optionally command the robot back to its starting position. Convenient when doing multiple sequential calibrations. | Medium | Capture positions at recording start, command them on stop. Needs position mode switch after torque mode. Already done in recorder TUI. |
| Torque ramp option | Instead of applying full torque instantly, ramp from 0 to target over a configurable duration. Reduces mechanical shock. | Medium | Timer-based ramp from 0 to target Nm. Good safety practice but adds complexity to the control loop. |
| Existing trigger display | Show any triggers already saved for the selected recording in TriggerStore. Helps user understand what they already have. | Low | Query `TriggerStore.get_for_recording(name)` and display in a read-only list. |

## Anti-Features

Features to explicitly NOT build. Including these would add complexity without value or would conflict with the tool's focused purpose.

| Anti-Feature | Why Avoid | What to Do Instead |
|--------------|-----------|-------------------|
| Playback functionality | Motor Studio already has full playback with trigger support. Duplicating it creates maintenance burden and user confusion about which tool to use. | Show a "Open in Motor Studio" hint or button after calibration. |
| Trigger editing | The existing `TriggerDialog` in Motor Studio handles editing. The calibrator creates triggers; Motor Studio manages them. | After saving, the trigger is in `TriggerStore` and visible in Motor Studio's playback tab. |
| Multi-joint simultaneous calibration | Torquing multiple joints at once makes it impossible to attribute position changes to a specific joint. The math breaks down and the safety risk multiplies. | Always calibrate one joint at a time. The tool enforces this. |
| Auto-stop based on stall detection | Stall detection requires monitoring velocity convergence, setting thresholds, handling oscillation. The user is better equipped to judge when the joint has settled by watching the live position. Premature auto-stop produces incorrect thresholds. | User stops manually. Show max position and live position so they can judge convergence. |
| Rising direction triggers | The existing workflow and `TriggerDialog` use falling triggers only (position drops below threshold). Rising triggers are a different use case not served by this calibration approach. | Always create falling triggers. If rising is needed later, that is a Motor Studio enhancement. |
| PID tuning | This is a calibration tool for trigger thresholds, not a servo tuning tool. PID parameters are set in the motor firmware. Mixing concerns creates a confusing tool. | Use the MyActuator RMD configurator or firmware tools for PID tuning. |
| Motor configuration | CAN interface, motor IDs, torque constants are configured in the setup TUI and saved in `DriverConfig`. The calibrator should not duplicate this. | Require the driver node to be running with correct configuration before launching the calibrator. |
| Direct CAN communication | The calibrator should not talk to motors directly. It communicates through the ROS 2 driver node via topics and services. Direct CAN access would conflict with the driver node. | All motor commands go through `RosBridge` which publishes to ROS topics consumed by the driver node. |
| Bag playback verification | After calibration, the user might want to verify the trigger works. That is Motor Studio's job. The calibrator should not try to play back recordings. | Display text: "Trigger saved. Use Motor Studio to play back the recording with the trigger." |

## Feature Dependencies

```
Connection to driver  -->  Joint selector populated
                      -->  Live position display
                      -->  Emergency stop functional

Joint selected  ------>  Torque can be applied
                ------>  Max position tracking starts on correct joint

Recording started --->  Torque applied to selected joint
                  --->  Other joints set to free
                  --->  Max position tracking active
                  --->  Live position display active
                  --->  ROS bag recording active

Recording stopped --->  Threshold computed (max_pos + offset)
                  --->  Trigger auto-created
                  --->  Trigger saved to TriggerStore
                  --->  Motors returned to safe state (position mode)

E-stop (anytime) ---->  Torque stops
                  --->  Recording stops
                  --->  Motors safe
```

## MVP Recommendation

Prioritize (Phase 1 -- all table stakes):

1. **Connection + joint selector + status** -- cannot do anything without these
2. **Torque input + offset input + recording name** -- the three user inputs
3. **Record/stop toggle with torque application** -- core behavior
4. **Live position + max position display** -- feedback during recording
5. **Auto-create trigger on stop + save to TriggerStore** -- the output
6. **Emergency stop** -- safety, non-negotiable

Defer to Phase 2 (differentiators):

- **Position chart/graph** -- high value but adds pyqtgraph dependency; can ship without it
- **Computed threshold preview before saving** -- nice but not blocking; auto-save is fine for v1
- **Return-to-start after calibration** -- convenience; user can manually reset
- **Torque ramp** -- safety improvement but adds control complexity
- **Calibration history log** -- session-only convenience; not needed for single calibrations

Do not build:

- Everything in the anti-features list, permanently

## Critical Implementation Note

The main architectural challenge is **per-joint mode control**. The current driver node (`driver_node.py`) has a single global mode (position, torque, free, etc.) applied to all joints. The calibrator needs:

- Joint A: torque mode (selected joint, receiving calibration torque)
- Joints B, C, ...: free mode (can move freely)

Two approaches exist in the codebase:

1. **Hybrid trigger approach**: The playback trigger system already achieves per-joint torque override during position mode. The calibrator could configure a trigger, but triggers evaluate based on recorded position which does not apply here since we are not playing back.

2. **Direct approach (recommended)**: Set global mode to "torque", then publish to `/joint_effort_ctrl` with the desired torque for the selected joint and 0.0 Nm for all others. When effort is 0.0 Nm, the motor effectively releases (same as free mode). This works because `send_torque(0.0)` commands zero current, which releases the motor.

Option 2 is simpler and does not require driver modifications. The calibrator publishes a `Float64MultiArray` on `/joint_effort_ctrl` at the control loop rate with the calibration torque for the selected joint and zeros for everything else.

## Sources

- Codebase analysis: `config.py` (HysteresisTorqueTrigger, TriggerStore), `driver_node.py` (mode control, trigger evaluation), `ros_bridge.py` (Qt-ROS bridge pattern), `recording_manager.py` (bag recording), `trigger_dialog.py` (existing trigger creation UI), `motor_wrapper.py` (motor control API), `control_panel.py` (mode switching), `playback_tab.py` (trigger configuration UI)
- PROJECT.md requirements and constraints
- Domain knowledge: robotics calibration tool patterns, motor tuning GUI conventions, safety practices for torque-controlled actuator systems (MEDIUM confidence -- training data, not verified against external sources)
