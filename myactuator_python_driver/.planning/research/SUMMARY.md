# Project Research Summary

**Project:** Torque Threshold Calibration Tool
**Domain:** Robotics motor control tooling - PyQt6 application for ROS 2 motor calibration
**Researched:** 2026-02-09
**Confidence:** HIGH

## Executive Summary

The torque threshold calibration tool is a standalone PyQt6 application that discovers the maximum position a joint reaches under applied torque, enabling automatic trigger creation for playback automation. Research reveals this is fundamentally an orchestration problem, not a development problem — 90% of required infrastructure already exists in the codebase. The calibration app reuses RosBridge (ROS communication), RecordingManager (bag recording), and TriggerStore (persistent storage) unmodified, introducing only two new components: CalibrationController (state machine) and CalibrationWindow (UI shell).

The recommended approach is a thin application layer (~300-400 lines of new code) that coordinates existing components through a simple state machine: set motors to appropriate modes, apply torque to one joint while leaving others free, track maximum position during recording, compute threshold from max + offset, and auto-create a HysteresisTorqueTrigger. No new dependencies, no new ROS topics, no driver modifications. The calibration output (trigger + recording) integrates seamlessly with Motor Studio's existing playback system because both apps share TriggerStore.

Key risks center on motor safety during torque application. Critical mitigations include: (1) using per-joint effort commands with zero-effort-as-free for non-selected joints to avoid mode transition jumps, (2) detecting conflicting mode commands from other apps (Motor Studio) and aborting calibration if control is lost, (3) enforcing motor-specific torque limits from actuator constants, and (4) prominent emergency stop accessible at all times. The architecture must handle these safety concerns from day one — they cannot be bolted on later.

## Key Findings

### Recommended Stack

Reuse 90% of existing infrastructure. The calibration app is a thin orchestrator on top of RosBridge, RecordingManager, and TriggerStore. Zero new dependencies needed beyond what's already in setup.py.

**Core technologies (all existing):**
- PyQt6 (>=6.4.0): GUI framework — already used by Motor Recording Studio; shared dark theme, styling patterns, widget patterns all transfer directly
- rclpy (ROS 2): ROS 2 communication — driver node already exposes all needed topics/services; no custom message types needed
- rosbag2_py (ROS 2 distro-matched): Bag recording — RecordingManager already handles Jazzy/Humble API differences
- Python 3.10+: Runtime — matches existing workspace

**Existing components to reuse (CRITICAL):**
- **RosBridge** (studio/ros_bridge.py): Qt-threadsafe ROS 2 pub/sub/srv, mode control, joint state streaming. Use directly as-is. Provides set_mode(), send_joint_command(), joint_state_received signal, connection_status_changed signal.
- **RecordingManager** (studio/recording_manager.py): Bag writing/reading. Use directly as-is. Call start_recording(), record_frame(), stop_recording().
- **TriggerStore** (config.py): Persistent JSON storage of HysteresisTorqueTrigger objects. Use directly as-is. Call TriggerStore.add(trigger) after calibration.
- **HysteresisTorqueTrigger** (config.py): Trigger data model with create_falling() factory method. Calibration result is exactly one HysteresisTorqueTrigger.

**New components to build (minimal):**
- CalibrationWindow (QMainWindow): Main calibration UI — joint selector, torque input, start/stop, results display. Medium complexity, ~150 lines.
- CalibrationController (QObject): State machine coordinating calibration sequence — set modes, apply torque, record, track max position, create trigger. Medium complexity, ~150-200 lines.
- MaxPositionTracker: Tracks maximum position for calibrated joint during recording. Low complexity, ~20 lines.
- Entry point: Console script launching the calibration app. Low complexity, boilerplate.

**ROS 2 communication pattern:**
The calibration workflow uses exclusively existing topics. No new topics, services, or message types required. All communication via /joint_states (subscribe), /joint_state_ctrl (publish effort), /motor_driver/set_mode (publish), /motor_driver/mode (subscribe for confirmation).

**Critical insight on per-joint mode control:**
Use torque mode with selective effort commands. Set mode to "torque", then publish JointState with target torque for calibrated joint and 0.0 Nm for all others. Zero effort releases the motor (same as free mode). The driver's _effort_ctrl_callback applies per-joint efforts from the _control_efforts dict. No driver modifications needed.

### Expected Features

**Must have (table stakes):**
- Joint selector dropdown populated from RosBridge.joint_names
- Torque input (Nm) with motor-specific limits from actuator constants
- Offset input (degrees) for threshold margin above max position
- Record button with start/stop toggle controlling torque application
- Apply torque to selected joint during recording while setting other joints to free mode
- Track max position of torqued joint during recording
- Display live joint position and max-position-so-far during recording
- Auto-create HysteresisTorqueTrigger on stop with threshold = max_position + offset
- Pair trigger with the recording (set recording_name field)
- Emergency stop (ESC key + prominent red button) using RosBridge.emergency_stop()
- Connection status indicator from RosBridge.connection_status_changed signal
- Recording name input with auto-generated timestamp default

**Should have (differentiators):**
- Pre-recording position display (all joints) to verify starting configuration
- Computed threshold preview before saving with confirm/discard options
- Calibration history log (session-only) for multiple sequential calibrations
- Position chart/graph during recording showing trajectory shape and convergence
- Torque direction indicator showing expected motion based on sign
- Auto-name trigger from joint and recording (e.g., "elbow_grasp_trigger")
- Return-to-start after calibration (capture positions, command them on stop)
- Torque ramp option (0 to target over duration) to reduce mechanical shock

**Defer (v2+):**
- Multiple calibration runs comparison for repeatability validation
- Position chart/graph (adds pyqtgraph dependency, can ship without it in v1)

**Anti-features (do NOT build):**
- Playback functionality — Motor Studio already has full playback with trigger support
- Trigger editing — TriggerDialog in Motor Studio handles editing
- Multi-joint simultaneous calibration — impossible to attribute position changes, safety risk multiplies
- Auto-stop based on stall detection — user is better equipped to judge convergence
- Rising direction triggers — existing workflow uses falling only; different use case
- PID tuning — different tool, use MyActuator RMD configurator
- Motor configuration — handled by setup TUI and DriverConfig
- Direct CAN communication — all communication through driver node via ROS topics
- Bag playback verification — Motor Studio's job

**Critical implementation note:**
The main architectural challenge is per-joint mode control. Current driver has single global mode applied to all joints. Calibrator needs joint A in torque mode, joints B/C/... in free mode. Solution: set global mode to "torque", then publish to /joint_state_ctrl with desired torque for selected joint and 0.0 Nm for all others. When effort is 0.0 Nm, motor effectively releases (same as free mode). Works because send_torque(0.0) commands zero current.

### Architecture Approach

The calibration app is a standalone PyQt6 process that communicates with the existing MotorDriverNode over ROS 2 topics. It reuses three existing components directly (RosBridge, RecordingManager, TriggerStore) and introduces two new components (CalibrationController, CalibrationWindow).

**System context:**
```
Calibration App (new process) <---> MotorDriverNode (existing)
    via /joint_states, /joint_effort_ctrl, ~/set_mode, ~/set_enabled
                |
                v filesystem
        recordings/ + triggers.json
```

**Internal component architecture:**
```
CalibrationWindow (QMainWindow) — UI layout, user input, display
    |
    +-- owns --> CalibrationController (QObject) — state machine, orchestration
                     |
                     +-- owns --> RosBridge (reused) — ROS communication in background thread
                     +-- owns --> RecordingManager (reused) — bag recording
                     +-- owns --> TriggerStore (reused) — persistent trigger storage
```

**Major components:**
1. CalibrationController (new): State machine (IDLE → PREPARING → CALIBRATING → CALCULATING → IDLE) coordinating ROS commands, recording, and trigger creation. Subscribes to joint_state_received, publishes effort commands, feeds frames to RecordingManager, computes threshold on completion.
2. CalibrationWindow (new): Thin UI layer. No ROS or recording logic. Single-purpose window (no tabs, no docks). Receives signals from controller, calls controller methods.
3. RosBridge (reused unmodified): Provides joint_state_received signal, joint_names property, connection_status_changed, set_mode(), send_joint_command(). No modifications needed.
4. RecordingManager (reused unmodified): start_recording(), record_frame(), stop_recording(). Calibration app calls record_frame() directly from joint state handler.
5. TriggerStore (reused unmodified): add() method persists triggers to triggers.json. Shared file between Calibration App and Motor Studio.

**Key patterns to follow:**
- Controller/view separation: all logic in controller, window just wires signals (from MainWindow pattern)
- RosBridge ownership: app entry point creates QApplication, window creates and owns RosBridge (from studio/main.py)
- Frame recording via joint state signal: recording frames fed from signal handler, not separate subscription (from MainWindow._on_joint_state)
- Mode restore on exit: restore motor mode when calibration ends or app closes to avoid leaving motors in dangerous state

**Data flow during calibration:**
MotorDriverNode publishes /joint_states (500 Hz) → CalibrationController extracts position for target joint, updates max_position, emits position_updated signal → CalibrationWindow updates labels. Controller calls record_frame(msg, ts) → RecordingManager writes to rosbag. Controller publishes JointState with effort → MotorDriverNode control loop applies torque to physical motor via CAN.

**Effort command construction:**
CalibrationController builds JointState with all joint names, positions (ignored in torque mode), velocities (ignored), and effort array with target torque for selected joint and 0.0 for others. Driver's _joint_ctrl_callback stores efforts in _control_efforts dict, control loop applies them. Joint with 0.0 Nm is effectively free.

**After calibration:**
Stop recording → set mode to position → calculate threshold (max_position + offset) → create HysteresisTorqueTrigger.create_falling() → TriggerStore.add(trigger) → emit calibration_complete signal → window displays result.

**File placement:**
```
myactuator_python_driver/calibrator/  # New package directory
    __init__.py
    main.py                           # Entry point (QApplication, theme, window)
    calibration_window.py             # CalibrationWindow (QMainWindow)
    calibration_controller.py         # CalibrationController (state machine)
```
Not under studio/ because calibrator is separate application. Imports from studio.ros_bridge, studio.recording_manager, and config.

### Critical Pitfalls

**Pitfall 1: Motor Jumps to Position Zero on Mode Transition**
When switching from free to torque mode, driver captures current positions as setpoints. If calibration app publishes torque command before position capture completes, non-selected joints may jerk toward last commanded position. **Prevention:** Never use /joint_effort_ctrl topic (sets ALL joints to torque globally). Instead, use ~/set_mode to switch to "free", then use per-joint torque trigger mechanism, OR send "torque" mode command then immediately publish effort with zero torque for non-selected joints. Test mode transitions with robot in safe configuration. Monitor /joint_states for sudden position changes. **Phase:** Architecture/design (Phase 1).

**Pitfall 2: Two GUI Apps Publishing Conflicting Mode Commands**
Motor Studio and Calibration App both publish to ~/set_mode. If both running, one overrides other's mode. Operator starts calibration (torque on one joint), Motor Studio sends "position" mode, torqued joint jerks to last commanded position. **Prevention:** Display warning "Close Motor Studio before calibration" or detect conflicting publishers. Subscribe to ~/mode and check if mode unexpectedly changes during calibration — if it does, immediately stop and emergency-stop. Calibrator should own mode lifecycle during calibration. **Phase:** Architecture (Phase 1).

**Pitfall 3: No Torque Limit Enforcement in Calibration App**
User enters arbitrary torque exceeding motor's rated torque (X4 rated 1 Nm, X8 rated 9 Nm). Motor draws excessive current, overheats, or gear reducer fails. **Prevention:** Load motor's rated torque from actuator_constants.hpp (available in config.py as MOTOR_TORQUE_CONSTANTS). Set spinbox maximum to rated torque (or 80%). Display rated torque next to input. Add timeout: auto-stop after 30s to prevent sustained thermal loading. Monitor temperature via /motor_status, auto-stop if >60C. **Phase:** UI design and safety constraints (Phase 1).

**Pitfall 4: Recording While Applying Torque Creates Misleading Recording**
Calibration records bag while applying torque. Recorded /joint_states shows joint moving under torque — NOT same as "free" recording. If played back in Motor Studio, driver tries to follow torque-induced positions. Playback expects free-mode recordings (operator moving by hand). **Prevention:** Clearly separate calibration recordings from normal recordings (prefix with "calibration_" or separate directory). Calibration recording should NOT be directly playable — mark as "calibration capture" in metadata. Better: pair auto-created trigger with operator's free-mode recording, not calibration recording. Clarify workflow: (1) record in free mode via Motor Studio, (2) run calibration to discover threshold, (3) calibration creates trigger paired with free-mode recording. **Phase:** Workflow design (Phase 1).

**Pitfall 5: CAN Bus Contention from Simultaneous Read and Command**
Driver's control loop reads motor state and sends commands for each motor sequentially at 500 Hz. With 8 motors and both read+command per motor, that's 16 CAN transactions per 2ms loop = ~100% bus utilization. If calibration app makes direct CAN requests, bus saturates and commands are dropped. **Prevention:** Calibration app must NOT create its own CAN driver or ActuatorInterface. Communicate exclusively through ROS 2 topics. Existing RosBridge provides this. Do NOT add "direct motor query" functionality. All motor data comes through driver node. If calibration app imports myactuator_rmd_py or creates CanDriver, that's a design violation. **Phase:** Architecture (Phase 1).

**Other significant pitfalls:**
- Position capture race condition during free-to-torque transition: wait for joint velocities <0.01 rad/s before mode change
- Max position tracking misses actual maximum: track in ROS callback, not GUI timer; use thread-safe variable
- Hysteresis offset too small or inverted: minimum offset 2x position noise (~1-2 degrees); minimum hysteresis 3x noise
- Emergency stop not accessible: prominent red button, bind ESC key, call /motor_driver/emergency_stop service
- RosBridge creates duplicate rclpy.init() call: initialize rclpy exactly once at app entry point

## Implications for Roadmap

Based on research, suggested phase structure:

### Phase 1: Safety Infrastructure & Core Controller
**Rationale:** Safety concerns must be designed in from the start, not bolted on. The state machine is the heart of the app — everything else depends on it. This phase establishes the control flow and safety guardrails before any UI work.

**Delivers:**
- CalibrationController with state machine (IDLE → PREPARING → CALIBRATING → CALCULATING)
- Per-joint effort command construction (target torque on selected, 0.0 on others)
- Max position tracking in ROS callback (thread-safe)
- Emergency stop integration (service call + mode restore + zero efforts)
- Mode conflict detection (subscribe to ~/mode, abort if unexpected change)
- Motor-specific torque limits from actuator constants
- Threshold calculation (max_position + offset) and trigger creation logic

**Addresses features:**
- Apply torque to selected joint (table stakes)
- Set other joints to free mode (table stakes)
- Track max position (table stakes)
- Auto-create trigger (table stakes)
- Emergency stop (table stakes)

**Avoids pitfalls:**
- Pitfall 1 (mode transition jumps): per-joint effort with zero-as-free
- Pitfall 2 (mode conflicts): mode monitoring and abort
- Pitfall 3 (torque limits): enforced at controller level
- Pitfall 5 (CAN contention): ROS-only communication enforced

**Research flag:** NO RESEARCH NEEDED. All patterns exist in codebase. Primarily integration work.

### Phase 2: UI Shell & Integration
**Rationale:** Once the controller works, the UI is straightforward signal wiring. This phase makes the calibration accessible to users and integrates with existing infrastructure.

**Delivers:**
- CalibrationWindow with all required inputs (joint selector, torque, offset, recording name)
- Live position and max position displays
- Start/stop toggle button
- Connection status indicator
- Emergency stop button (large, red, always visible)
- Recording integration via RecordingManager
- Trigger persistence via TriggerStore
- Entry point (main.py with QApplication, dark theme)
- Console script in setup.py

**Addresses features:**
- Joint selector dropdown (table stakes)
- Torque input (table stakes)
- Offset input (table stakes)
- Recording name input (table stakes)
- Display live position (table stakes)
- Display max position (table stakes)
- Connection status (table stakes)

**Uses stack:**
- PyQt6 for GUI (existing)
- RosBridge for ROS communication (reused)
- RecordingManager for bag recording (reused)
- TriggerStore for persistence (reused)
- Dark theme from studio/main.py (copy pattern)

**Implements architecture:**
- CalibrationWindow (thin UI layer)
- Component ownership (window owns RosBridge, RecordingManager, TriggerStore, Controller)
- Signal/slot wiring between controller and window

**Avoids pitfalls:**
- Pitfall 9 (no e-stop): prominent button, ESC binding
- Pitfall 10 (rclpy double init): initialize once in main.py entry point
- Pitfall 14 (units confusion): explicit labels, conversions at UI boundary

**Research flag:** NO RESEARCH NEEDED. UI follows existing patterns from MainWindow and TriggerDialog.

### Phase 3: Polish & Safety Enhancements
**Rationale:** Basic calibration works from Phase 1+2. This phase adds user-facing polish and additional safety mechanisms to improve the calibration experience and reduce operator error.

**Delivers:**
- Computed threshold preview with confirm/discard before saving
- Pre-recording position display (all joints) for configuration verification
- Calibration history log (session-only, QTableWidget)
- Torque direction indicator (positive/negative motion)
- Auto-name trigger from joint + recording
- Return-to-start after calibration (capture positions, command on stop)
- Recording settle check (wait for velocity <0.01 rad/s before torque)
- Position noise measurement and offset/hysteresis validation
- Temperature monitoring with auto-stop at 60C
- TriggerStore reload before write to avoid file conflicts with Motor Studio

**Addresses features:**
- Computed threshold preview (differentiator)
- Pre-recording position display (differentiator)
- Calibration history log (differentiator)
- Torque direction indicator (differentiator)
- Auto-name trigger (differentiator)
- Return-to-start (differentiator)

**Avoids pitfalls:**
- Pitfall 6 (position capture race): velocity check before mode change
- Pitfall 7 (max tracking misses peak): already handled in Phase 1, validation here
- Pitfall 8 (offset too small): noise-based validation
- Pitfall 11 (TriggerStore conflicts): reload before write
- Pitfall 13 (bag starts before mode change): wait for mode confirmation

**Research flag:** NO RESEARCH NEEDED. Straightforward enhancements building on Phase 1+2 foundation.

### Phase 4: Advanced Features (Optional)
**Rationale:** These features are nice-to-have but not essential for MVP. Can be deferred or implemented based on user feedback after Phase 1-3 deployed.

**Delivers:**
- Position chart/graph during recording (pyqtgraph or matplotlib)
- Torque ramp option (0 to target over configurable duration)
- Multiple calibration runs comparison for repeatability
- Existing trigger display for selected recording
- Rising trigger auto-detection based on position change direction

**Addresses features:**
- Position chart (deferred differentiator)
- Torque ramp (deferred differentiator)
- Multiple runs comparison (deferred differentiator)
- Existing trigger display (deferred differentiator)

**Avoids pitfalls:**
- Pitfall 12 (falling trigger assumption): auto-detect direction

**Research flag:** LIGHT RESEARCH for position chart (pyqtgraph API, integration with Qt signal updates). Otherwise standard patterns.

### Phase Ordering Rationale

**Why Phase 1 first:**
Safety infrastructure cannot be an afterthought. The controller's state machine is the core logic — UI and polish are useless without a working calibration sequence. Per-joint effort commands, torque limits, emergency stop, and mode conflict detection must be designed correctly from the start. Fixing these later risks architectural rework or, worse, shipping with safety gaps.

**Why Phase 2 second:**
UI is signal wiring once controller exists. Every widget maps to a controller signal or method. This phase makes the tool usable but adds no new behavior — it exposes what Phase 1 built. Integration with RecordingManager and TriggerStore is straightforward because both are well-tested components with stable APIs.

**Why Phase 3 third:**
Polish and enhancements improve UX and add defensive checks (velocity settle, temperature monitoring, noise validation) but aren't required for basic calibration to work. These can be implemented incrementally without redesign. If time is tight, Phase 1+2 alone delivers a functional tool.

**Why Phase 4 optional:**
Advanced features like position graphing and torque ramping are nice improvements but require additional complexity (pyqtgraph dependency, ramp control loop). Deferring these allows earlier deployment and user feedback before investing in polish. Rising trigger auto-detection is interesting but PROJECT.md explicitly scopes this as "falling triggers only" for MVP.

**Dependency relationships:**
- Phase 2 requires Phase 1 (UI wires controller signals)
- Phase 3 builds on Phase 1+2 (enhancements to working tool)
- Phase 4 independent of Phase 3 (can be implemented in any order or skipped)

**Groupings based on architecture:**
- Phase 1 = backend (controller, state machine, ROS communication)
- Phase 2 = frontend (UI, integration, entry point)
- Phase 3 = middleware enhancements (safety checks, validation, polish)
- Phase 4 = advanced frontend (visualization, complex features)

**How this avoids pitfalls:**
All critical pitfalls (1-5) are addressed in Phase 1 architecture. Moderate pitfalls (6-10) are split: some in Phase 1 (e-stop, rclpy init), others in Phase 2 (units confusion), remainder in Phase 3 (settle checks, noise validation). Minor pitfalls (11-14) are handled in Phase 3 polish. This ensures safety-critical items are never deferred.

### Research Flags

**Phases NOT needing research-phase:**
- **Phase 1:** All patterns exist in codebase. Controller follows MainWindow pattern. RosBridge, RecordingManager, TriggerStore APIs are stable and well-understood from existing code analysis.
- **Phase 2:** UI follows existing patterns from MainWindow (window structure), TriggerDialog (input widgets), control_panel (e-stop button). PyQt6 usage is standard, no niche APIs.
- **Phase 3:** Enhancements are straightforward extensions. Velocity monitoring uses existing /joint_states data. Temperature monitoring uses /motor_status. TriggerStore file handling is simple reload logic.

**Phases possibly needing light research:**
- **Phase 4 (optional):** Position chart integration with pyqtgraph or matplotlib. Would benefit from quick API research to understand signal-to-plot update pattern, but not deep domain research. 30-minute investigation, not full research-phase.

**Overall:** This project requires ZERO research-phase invocations during roadmap planning. All necessary patterns, APIs, and architectural decisions are already understood from initial research. The work is primarily integration and orchestration of existing components.

## Confidence Assessment

| Area | Confidence | Notes |
|------|------------|-------|
| Stack | HIGH | All dependencies already in setup.py. Verified no new packages needed. Component reuse confirmed by reading source. |
| Features | HIGH | Feature breakdown based on domain knowledge (motor calibration) validated against existing codebase patterns (TriggerDialog, RecorderTUI). User expectations inferred from PROJECT.md requirements. |
| Architecture | HIGH | All findings from direct codebase analysis. Component APIs verified by reading source files. State machine design follows established Qt/ROS patterns. No external sources needed. |
| Pitfalls | HIGH | All pitfalls derived from specific code paths with line numbers. Safety concerns validated against driver_node.py control loop. No training-data-only claims. |

**Overall confidence:** HIGH

All research based on direct analysis of existing codebase. No external documentation consulted because the calibration tool is purely internal to this system — it orchestrates existing components with no new external integrations. The primary research question was "what can be reused?" and the answer is "almost everything."

### Gaps to Address

**Minimal gaps identified:**

1. **Workflow clarity needed:** PROJECT.md doesn't explicitly state whether the calibration recording itself should be playable in Motor Studio, or whether the trigger should be paired with a separate free-mode recording. Recommendation: treat calibration recording as measurement tool, not playback target. Trigger pairs with free-mode recording made separately. This should be validated with user/stakeholder before Phase 1.

2. **Torque ramp vs instant application:** Instant torque application is simpler (Phase 1+2) but torque ramp is better safety practice (Phase 3 or 4). Decision deferred to implementation — instant is acceptable for MVP if motor-specific torque limits are enforced.

3. **Rising vs falling trigger direction:** PROJECT.md scopes "falling triggers only" but calibration of joints that extend under torque (not retract) would need rising triggers. This is acknowledged as out-of-scope for MVP. Phase 4 could add auto-detection.

4. **Simultaneous app usage:** Research identified conflict between Motor Studio and Calibration App if both running. Mitigation chosen: display warning and/or detect mode conflicts. Stronger solution (service-based mode control instead of topic-based) would require driver changes — out of scope for this project but good tech debt item.

**None of these gaps block Phase 1 implementation.** All are design decisions or nice-to-have enhancements, not missing information. The core calibration workflow is fully specified and all required APIs are confirmed to exist.

## Sources

### Primary (HIGH confidence)

All findings based on direct analysis of the existing codebase at /home/nitish/work/myactuator:

**Core component APIs:**
- myactuator_python_driver/myactuator_python_driver/studio/ros_bridge.py — RosBridge interface, threading model, signal definitions
- myactuator_python_driver/myactuator_python_driver/studio/recording_manager.py — RecordingManager API, bag recording/playback
- myactuator_python_driver/myactuator_python_driver/config.py — TriggerStore, HysteresisTorqueTrigger, PlaybackTriggerConfig, motor constants

**Driver and control patterns:**
- myactuator_python_driver/myactuator_python_driver/driver_node.py — Mode control, effort handling, trigger evaluation, control loop structure
- myactuator_python_driver/myactuator_python_driver/motor_wrapper.py — CAN communication, unit conversion, send_torque/release behavior

**Existing UI patterns:**
- myactuator_python_driver/myactuator_python_driver/studio/main.py — Application entry point, QApplication setup, dark theme
- myactuator_python_driver/myactuator_python_driver/studio/main_window.py — Window composition, signal wiring, recording flow
- myactuator_python_driver/myactuator_python_driver/studio/dialogs/trigger_dialog.py — Trigger creation UI patterns, input validation
- myactuator_python_driver/myactuator_python_driver/studio/widgets/control_panel.py — Emergency stop button pattern

**Additional context:**
- myactuator_python_driver/myactuator_python_driver/recorder_tui.py — Recording/playback TUI patterns, mode switching
- myactuator_python_driver/setup.py — Entry points, dependencies
- myactuator_rmd/include/myactuator_rmd/actuator_constants.hpp — Motor rated torque values
- .planning/codebase/ARCHITECTURE.md — System architecture documentation
- .planning/codebase/CONCERNS.md — Known tech debt and fragile areas

**Research outputs:**
- .planning/research/STACK.md — Technology stack recommendations
- .planning/research/FEATURES.md — Feature landscape and MVP definition
- .planning/research/ARCHITECTURE.md — Component architecture and patterns
- .planning/research/PITFALLS.md — Domain-specific risks and mitigations

### Secondary (MEDIUM confidence)

None. All research findings derived from primary source code analysis.

### Tertiary (LOW confidence)

None. This project required no external sources or training-data-based inference. The calibration tool is purely internal to this system.

---

*Research completed: 2026-02-09*
*Ready for roadmap: yes*
