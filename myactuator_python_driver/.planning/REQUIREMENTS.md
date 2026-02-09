# Requirements: Torque Threshold Calibrator

**Defined:** 2026-02-09
**Core Value:** Automatically discover trigger thresholds by applying torque and measuring the result

## v1 Requirements

Requirements for initial release. Each maps to roadmap phases.

### Connection & Safety

- [ ] **CONN-01**: App displays connection status to the ROS 2 driver node (connected/disconnected)
- [ ] **CONN-02**: App prevents calibration actions when driver node is not connected
- [ ] **SAFE-01**: User can trigger emergency stop at any time via Escape key or prominent red button
- [ ] **SAFE-02**: Emergency stop immediately stops torque application, stops recording, and returns motors to safe state

### Inputs & Configuration

- [ ] **INPT-01**: User can select which joint to calibrate from a dropdown populated by connected motor joints
- [ ] **INPT-02**: User can set the torque value in Nm via a numeric input
- [ ] **INPT-03**: User can set the threshold offset in degrees via a numeric input (default 0.5 degrees)
- [ ] **INPT-04**: User can set or edit the recording name before starting (auto-generated timestamp default)
- [ ] **INPT-05**: Joint selector, torque, offset, and recording name inputs are disabled during active recording

### Calibration Core

- [ ] **CALB-01**: User can start calibration by pressing a Record button
- [ ] **CALB-02**: On record start, the selected joint receives the configured torque command
- [ ] **CALB-03**: On record start, all non-selected joints are set to free mode (zero torque)
- [ ] **CALB-04**: A ROS 2 bag recording starts simultaneously with torque application
- [ ] **CALB-05**: User can stop calibration by pressing the Stop button (manual stop)
- [ ] **CALB-06**: On stop, torque application ceases and motors return to a safe state

### Feedback & Display

- [ ] **DISP-01**: App displays live position of the selected joint during recording (updated at ~20Hz)
- [ ] **DISP-02**: App displays the running max position of the selected joint during recording
- [ ] **DISP-03**: App shows torque direction indicator (which direction the joint will move based on torque sign)

### Trigger Output

- [ ] **TRIG-01**: On recording stop, app computes threshold as max position + configurable offset (converted to radians)
- [ ] **TRIG-02**: App shows a preview of the computed trigger (threshold, torque, joint, recording name) before saving
- [ ] **TRIG-03**: User can confirm (save) or discard the computed trigger from the preview
- [ ] **TRIG-04**: On save, a falling HysteresisTorqueTrigger is created and stored in TriggerStore
- [ ] **TRIG-05**: The created trigger is paired with the recording name (recording_name field set)
- [ ] **TRIG-06**: Trigger name is auto-generated from joint name and recording name (user can override)

### App Infrastructure

- [ ] **INFR-01**: App is a standalone PyQt6 application with its own entry point (not a Motor Studio tab)
- [ ] **INFR-02**: App reuses existing RosBridge for ROS 2 communication
- [ ] **INFR-03**: App reuses existing RecordingManager for ROS 2 bag recording
- [ ] **INFR-04**: App reuses existing TriggerStore for persistent trigger storage
- [ ] **INFR-05**: App is registered as a console_scripts entry point in setup.py

## v2 Requirements

Deferred to future release. Tracked but not in current roadmap.

### Enhanced Feedback

- **DISP-10**: Position chart/graph showing joint trajectory over time during recording
- **DISP-11**: Pre-recording position display showing all joints before calibration starts
- **DISP-12**: Calibration history log for the current session

### Convenience

- **CONV-01**: Return-to-start position after calibration stops
- **CONV-02**: Torque ramp option (gradual increase from 0 to target Nm)
- **CONV-03**: Display existing triggers for the selected recording from TriggerStore

### Advanced

- **ADVN-01**: Multiple calibration runs comparison for repeatability validation

## Out of Scope

| Feature | Reason |
|---------|--------|
| Playback functionality | Handled by Motor Studio — no duplication |
| Trigger editing/management | Handled by Motor Studio's TriggerDialog |
| Multi-joint simultaneous calibration | Cannot attribute position changes to specific joint; safety risk |
| Auto-stop based on stall detection | User judges convergence better by watching live position |
| Rising direction triggers | Only falling triggers match existing workflow |
| PID/servo tuning | Different tool entirely — use motor firmware configurator |
| Motor/CAN configuration | Handled by setup TUI and DriverConfig |
| Direct CAN bus communication | All commands go through ROS 2 driver node |

## Traceability

Which phases cover which requirements. Updated during roadmap creation.

| Requirement | Phase | Status |
|-------------|-------|--------|
| CONN-01 | Phase ? | Pending |
| CONN-02 | Phase ? | Pending |
| SAFE-01 | Phase ? | Pending |
| SAFE-02 | Phase ? | Pending |
| INPT-01 | Phase ? | Pending |
| INPT-02 | Phase ? | Pending |
| INPT-03 | Phase ? | Pending |
| INPT-04 | Phase ? | Pending |
| INPT-05 | Phase ? | Pending |
| CALB-01 | Phase ? | Pending |
| CALB-02 | Phase ? | Pending |
| CALB-03 | Phase ? | Pending |
| CALB-04 | Phase ? | Pending |
| CALB-05 | Phase ? | Pending |
| CALB-06 | Phase ? | Pending |
| DISP-01 | Phase ? | Pending |
| DISP-02 | Phase ? | Pending |
| DISP-03 | Phase ? | Pending |
| TRIG-01 | Phase ? | Pending |
| TRIG-02 | Phase ? | Pending |
| TRIG-03 | Phase ? | Pending |
| TRIG-04 | Phase ? | Pending |
| TRIG-05 | Phase ? | Pending |
| TRIG-06 | Phase ? | Pending |
| INFR-01 | Phase ? | Pending |
| INFR-02 | Phase ? | Pending |
| INFR-03 | Phase ? | Pending |
| INFR-04 | Phase ? | Pending |
| INFR-05 | Phase ? | Pending |

**Coverage:**
- v1 requirements: 29 total
- Mapped to phases: 0
- Unmapped: 29 ⚠️

---
*Requirements defined: 2026-02-09*
*Last updated: 2026-02-09 after initial definition*
