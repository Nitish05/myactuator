# Roadmap: Torque Threshold Calibrator

## Overview

The Torque Threshold Calibrator delivers automated trigger discovery in three phases. Phase 1 establishes safety infrastructure and core calibration logic (controller, state machine, per-joint torque commands). Phase 2 exposes this through a PyQt6 UI integrated with existing ROS infrastructure (RosBridge, RecordingManager, TriggerStore). Phase 3 adds trigger management, preview, and output persistence. The tool reuses 90% of existing components, introducing only orchestration logic to coordinate torque application, position tracking, and automatic HysteresisTorqueTrigger creation.

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 2.2): Urgent insertions (marked with INSERTED)

Decimal phases appear between their surrounding integers in numeric order.

- [ ] **Phase 1: Safety Infrastructure & Core Controller** - Build calibration state machine with safety guardrails
- [ ] **Phase 2: UI Shell & Basic Calibration** - Expose controller through PyQt6 interface with ROS integration
- [ ] **Phase 3: Trigger Management & Output** - Implement trigger preview, persistence, and management

## Phase Details

### Phase 1: Safety Infrastructure & Core Controller
**Goal**: User can run headless calibration sequence with safety protections and per-joint torque control
**Depends on**: Nothing (first phase)
**Requirements**: CONN-01, CONN-02, SAFE-01, SAFE-02, CALB-02, CALB-03, CALB-04, CALB-06, INFR-02, INFR-03, INFR-04
**Success Criteria** (what must be TRUE):
  1. CalibrationController applies torque to selected joint while setting other joints to free mode (zero effort)
  2. Emergency stop immediately ceases torque application and returns motors to safe state
  3. Controller detects when driver node connection is lost and aborts calibration
  4. Max position tracking accurately captures the highest position reached during torque application
  5. ROS 2 bag recording runs simultaneously with torque application
**Plans**: TBD

Plans:
- [ ] 01-01: TBD during planning

### Phase 2: UI Shell & Basic Calibration
**Goal**: User can launch the calibration app, configure parameters, and execute calibrations through a graphical interface
**Depends on**: Phase 1
**Requirements**: INPT-01, INPT-02, INPT-03, INPT-04, INPT-05, CALB-01, CALB-05, DISP-01, DISP-02, DISP-03, INFR-01, INFR-05
**Success Criteria** (what must be TRUE):
  1. User can select which joint to calibrate from a dropdown populated by connected motors
  2. User can set torque value (Nm) and threshold offset (degrees) with validation against motor limits
  3. User can start calibration with Record button and stop manually with Stop button
  4. App displays live joint position and running max position during active calibration at ~20Hz
  5. App prevents calibration when driver node is not connected via visible status indicator
**Plans**: TBD

Plans:
- [ ] 02-01: TBD during planning

### Phase 3: Trigger Management & Output
**Goal**: User can review computed trigger parameters before saving and triggers persist to TriggerStore for use in Motor Studio
**Depends on**: Phase 2
**Requirements**: TRIG-01, TRIG-02, TRIG-03, TRIG-04, TRIG-05, TRIG-06
**Success Criteria** (what must be TRUE):
  1. On recording stop, app computes threshold as max position plus configurable offset
  2. User sees preview of computed trigger (threshold, torque, joint, recording name) before confirming
  3. User can confirm to save or discard the computed trigger from preview
  4. Saved trigger is stored as HysteresisTorqueTrigger in TriggerStore and usable in Motor Studio playback
  5. Trigger is automatically paired with the recording name and has auto-generated name from joint and recording
**Plans**: TBD

Plans:
- [ ] 03-01: TBD during planning

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Safety Infrastructure & Core Controller | 0/? | Not started | - |
| 2. UI Shell & Basic Calibration | 0/? | Not started | - |
| 3. Trigger Management & Output | 0/? | Not started | - |
