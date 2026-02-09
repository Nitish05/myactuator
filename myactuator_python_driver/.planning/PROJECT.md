# Torque Threshold Calibrator

## What This Is

A standalone PyQt6 desktop application that automates the discovery of torque trigger thresholds for motor recordings. Instead of manually moving a joint and capturing a position, the user selects a joint and torque value, hits record, and the app applies the torque while recording — then automatically calculates the threshold from the max position reached and creates a trigger paired with that recording.

## Core Value

Automatically discover the correct trigger threshold for a recording by applying torque and measuring the result — eliminating manual guesswork from the trigger configuration process.

## Requirements

### Validated

- ✓ HysteresisTorqueTrigger data model with enter/exit thresholds, hysteresis, and recording pairing — existing in `config.py`
- ✓ TriggerStore for persistent JSON storage of triggers — existing in `config.py`
- ✓ ROS 2 driver supports per-joint torque triggers during playback via `~/playback_triggers` topic — existing in `driver_node.py`
- ✓ RecordingManager handles ROS 2 bag recording/playback — existing in `recording_manager.py`
- ✓ Motor Studio plays back recordings with trigger support — existing in `main_window.py`
- ✓ RosBridge provides ROS 2 communication layer with joint state, mode control, and trigger config — existing in `ros_bridge.py`

### Active

- [ ] Standalone PyQt6 app with joint selector, torque input, offset input, and record/stop controls
- [ ] Apply selected torque to chosen joint while setting all other joints to free mode during recording
- [ ] Record ROS 2 bag simultaneously while torque is applied
- [ ] Track max position of the torqued joint during the recording
- [ ] Calculate threshold as max position + configurable offset (default 0.5°)
- [ ] Auto-create a falling HysteresisTorqueTrigger paired with the recording in TriggerStore
- [ ] User stops recording manually
- [ ] Display live joint position and max-position-so-far during recording

### Out of Scope

- Playback functionality — handled by existing Motor Studio
- Trigger editing — handled by existing TriggerDialog
- Multi-joint simultaneous torque calibration — one joint at a time
- Auto-stop based on stall detection — user stops manually
- Rising direction triggers — only falling triggers (matching existing TriggerDialog behavior)

## Context

This app fills a gap in the Motor Studio workflow. Currently, configuring a torque trigger requires the user to:
1. Physically move the robot joint to the desired threshold position
2. Capture that position in the TriggerDialog
3. Guess the correct torque and threshold values

This is error-prone because the "right" threshold depends on where the joint actually ends up under the applied torque — something you can only know by applying the torque first.

The new app inverts the process: apply the torque, observe the result, and derive the threshold automatically.

The app reuses the existing infrastructure:
- `RosBridge` for ROS 2 communication (joint states, mode control)
- `RecordingManager` for ROS 2 bag recording
- `TriggerStore` for persistent trigger storage
- `HysteresisTorqueTrigger` data model

The trigger created by this app is directly usable by Motor Studio's playback system — the driver evaluates triggers against recorded positions and switches between position control and torque control per-joint.

## Constraints

- **Tech stack**: PyQt6, must integrate with existing ROS 2 driver node and infrastructure
- **ROS 2 dependency**: Requires the motor driver node to be running for joint states and motor control
- **CAN bus**: Requires active CAN connection to physical motors
- **Single joint**: Only one joint receives torque at a time; others must be in free mode
- **Trigger direction**: Always falling (activates when position drops below threshold), matching existing convention

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Standalone app, not a Motor Studio tab | Keeps Motor Studio focused on recording/playback; calibration is a separate workflow | — Pending |
| Falling triggers only | Matches existing TriggerDialog behavior and the primary use case (grip/close motions) | — Pending |
| Configurable offset (default 0.5°) | Different joints/tasks may need different margins above max position | — Pending |
| Reuse RosBridge and RecordingManager | Avoid duplicating ROS 2 communication and bag recording logic | — Pending |

---
*Last updated: 2026-02-09 after initialization*
