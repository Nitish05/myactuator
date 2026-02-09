---
phase: 01-safety-infrastructure-core-controller
plan: 02
subsystem: calibration
tags: [cli, argparse, qcoreapplication, entry-point, setup-py, hardware-verification]

# Dependency graph
requires:
  - 01-01 (CalibrationController, CalibrationConfig, CalibrationResult)
provides:
  - Headless CLI entry point (calibrator_cli) for running calibration from terminal
  - setup.py console_scripts registration
affects:
  - phase-02 (GUI entry point replaces/supplements CLI)

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "QCoreApplication for headless Qt entry point (no display server)"
    - "Two-phase position tracking: extreme in torque direction, then threshold after reversal"
    - "Settle time window (default 3s) freezes threshold after reversal to capture dwelling position"
    - "SIGINT handling: first Ctrl+C graceful stop, second emergency stop"
    - "Qt signal timer workaround (200ms) for SIGINT in C++ event loop"
    - "Free mode on stop (not position mode) to release motor torque"

key-files:
  created:
    - myactuator_python_driver/myactuator_python_driver/calibrator/main.py
  modified:
    - myactuator_python_driver/setup.py
    - myactuator_python_driver/myactuator_python_driver/calibrator/config.py
    - myactuator_python_driver/myactuator_python_driver/calibrator/controller.py

key-decisions:
  - "Free mode on stop instead of position mode -- position mode PID applies holding torque"
  - "Two-phase tracking with settle window captures dwelling position, not full comeback peak"
  - "ROS calls wrapped in try/except for clean shutdown when handles are destroyed"
  - "SIGINT prints scheduled on Qt event loop to avoid reentrant stdout writes"
  - "1-degree reversal threshold filters noise before entering phase 2"

patterns-established:
  - "Wrap all ROS bridge calls in try/except for shutdown resilience"
  - "Use QTimer.singleShot(0, fn) for SIGINT handler actions, never print directly"

# Metrics
duration: ~15min (including hardware verification and bug fixes)
completed: 2026-02-09
---

# Phase 1 Plan 2: Headless CLI & Hardware Verification Summary

**CLI entry point (`calibrator_cli`) with two-phase threshold tracking, settle window, and hardware-verified calibration flow**

## Performance

- **Duration:** ~15 min (including 3 hardware test iterations)
- **Started:** 2026-02-09T16:20:00Z
- **Completed:** 2026-02-09T16:55:00Z
- **Tasks:** 2 (1 auto, 1 human-verify checkpoint)
- **Files created:** 1
- **Files modified:** 3

## Accomplishments
- Headless CLI entry point wires RosBridge + RecordingManager + CalibrationController with QCoreApplication
- argparse CLI: `--joint`, `--torque`, `--name`, `--offset`, `--settle-time`
- setup.py registers `calibrator_cli` console_scripts entry point
- Two-phase position tracking: extreme in torque direction → reversal detection → settle window → threshold locked
- Hardware verified: torque application, position tracking, Ctrl+C graceful stop, connection loss detection all working

## Task Commits

1. **Task 1: Create headless entry point** - `f22dd71` (feat)
2. **Task 2: Hardware verification fixes** - `ab4608f` (fix)
   - Two-phase threshold tracking, settle window, free mode, shutdown resilience

## Files Created/Modified
- `myactuator_python_driver/myactuator_python_driver/calibrator/main.py` - CLI entry point (210 lines): QCoreApplication, argparse, signal handling, RosBridge wiring
- `myactuator_python_driver/setup.py` - Added `calibrator_cli` console_scripts entry
- `myactuator_python_driver/myactuator_python_driver/calibrator/config.py` - Added settle_time_sec, extreme_position_rad
- `myactuator_python_driver/myactuator_python_driver/calibrator/controller.py` - Two-phase tracking, settle freeze, shutdown resilience

## Decisions Made
- Free mode on stop (not position mode) because position hold PID applies torque that user perceives as "torque still applied"
- Two-phase tracking with settle window: arm presses down (phase 1), user rotates part and joint bounces at bottom (phase 2 captures this), full comeback is ignored
- 1-degree reversal threshold prevents noise from prematurely triggering phase 2
- 3-second default settle window captures dwelling position before user rotates part back

## Deviations from Plan
- Plan specified position mode on stop; changed to free mode after hardware testing revealed position hold applies unwanted torque
- Added two-phase threshold tracking with settle window (not in original plan) based on actual sanding drum calibration workflow
- Added settle_time_sec to CalibrationConfig and --settle-time to CLI

## Issues Encountered
- **InvalidHandle crash on Ctrl+C**: ROS handles destroyed during Qt shutdown; fixed with try/except wrappers
- **Reentrant stdout write**: SIGINT handler called print() while another print() was in progress; fixed by scheduling all prints on Qt event loop
- **Position mode torque**: Switching to position mode after calibration applied PID holding torque; fixed by using free mode

## Hardware Verification Results
- Torque application on selected joint: PASSED
- Other joints free during calibration: PASSED
- Position tracking and reversal detection: PASSED
- Graceful stop (Ctrl+C): PASSED
- Connection loss detection: PASSED
- Emergency stop: PASSED (with shutdown resilience fix)

## Self-Check: PASSED

All files verified on disk. Commits f22dd71 and ab4608f verified in git log.

---
*Phase: 01-safety-infrastructure-core-controller*
*Completed: 2026-02-09*
