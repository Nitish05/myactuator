# Codebase Concerns

**Analysis Date:** 2026-02-09

## Tech Debt

**Silent Exception Handling in Motor Wrapper:**
- Issue: Bare `except:` or `except Exception:` blocks swallow all errors without logging
- Files: `myactuator_python_driver/motor_wrapper.py` (lines 139-141, 157-158, 183-184, 202-203, 220-221, 238-239, 315-316)
- Impact: Motor communication failures are silently masked. When `send_position()`, `send_velocity()`, `send_torque()`, etc. fail, the caller only gets the last cached state without knowing the command failed. This makes debugging difficult and can lead to unexpected motor behavior.
- Fix approach: Log errors with meaningful messages indicating which operation failed and why. Preserve error information for diagnostics while still returning a valid state.

**Unclosed File Handles in Recording Operations:**
- Issue: `recorder_tui.py` line 135 opens `/dev/null` but may not be properly closed if exception occurs before line 140
- Files: `myactuator_python_driver/recorder_tui.py` (lines 132-140)
- Impact: Resource leak of file descriptor. Under heavy use or repeated failures, file descriptor table can become exhausted.
- Fix approach: Use context manager: `with open(os.devnull, 'w') as f: sys.stderr = f`

**Bare Socket Error Handling in CAN Scanner:**
- Issue: `can_utils.py` line 71 catches `OSError` but doesn't log or provide context about the socket failure
- Files: `myactuator_python_driver/can_utils.py` (lines 71-72, 85-86)
- Impact: Silent socket failures during motor scanning provide no feedback about why a scan failed. User gets empty results without knowing if hardware is disconnected or misconfigured.
- Fix approach: Log the actual error with socket details. Distinguish between timeout vs. connection errors.

**ROS 2 Bag Reader Not Closed:**
- Issue: `recorder_tui.py` line 223 and line 239 create `SequentialReader` instances but don't explicitly close them
- Files: `myactuator_python_driver/recorder_tui.py` (lines 223-245)
- Impact: Bag file handles remain open, potentially blocking deletion or modification of recordings. Multiple sequential calls leak resources.
- Fix approach: Call `reader.close()` in a try-finally block. Consider implementing context manager pattern.

**Multiple Recorder Instances Not Synchronized:**
- Issue: Both `recorder_tui.py` and `studio/recording_manager.py` can open the same bag directory concurrently
- Files: `myactuator_python_driver/recorder_tui.py`, `myactuator_python_driver/studio/recording_manager.py`
- Impact: Concurrent write operations to the same bag can corrupt metadata or lose frames. No file-level locking prevents simultaneous writes.
- Fix approach: Implement file-based locking (fcntl on Unix) or use bag writer semaphore to prevent concurrent access.

## Known Bugs

**Trigger State Deserialization May Fail Silently:**
- Symptoms: Trigger states from driver not displayed in TUI, but no error message shown
- Files: `myactuator_python_driver/recorder_tui.py` (lines 193-200)
- Trigger: Malformed JSON in trigger_states message or network interruption
- Workaround: Check trigger states in driver node logs separately

**Position Offset Direction Application Order:**
- Symptoms: When both `position_offset` and `inverted` are enabled, offset may apply in wrong direction relative to physical motor
- Files: `myactuator_python_driver/motor_wrapper.py` (lines 124-125, 172-173)
- Trigger: Use `position_offset` > 0 with `inverted=True`. Offset is applied after direction inversion, which may not match user expectation.
- Workaround: Apply negative offset to compensate for direction inversion

**ROS Bag Playback Frame Count Inaccurate:**
- Symptoms: Playback progress bar doesn't match actual number of frames played
- Files: `myactuator_python_driver/recorder_tui.py` (lines 231-232)
- Trigger: When bag contains other topics besides `/joint_states`, frame count includes all topics
- Workaround: Check bag metadata separately with `rosbag2 info`

## Security Considerations

**Subprocess Command Injection via Interface Name:**
- Risk: `can_utils.py` passes user-provided interface name to `subprocess.run()` without validation
- Files: `myactuator_python_driver/can_utils.py` (lines 163, 179, 200)
- Current mitigation: Interface names come from `/proc` output or internal config, not direct user input. Subprocess called with `capture_output=True` and timeout.
- Recommendations: Validate interface name against allowed characters (alphanumeric, underscore, hyphen). Use `shlex.quote()` if any user input involved.

**Unencrypted Motor Configuration Storage:**
- Risk: Motor configuration in YAML files may be readable by other users on shared system
- Files: `myactuator_python_driver/config.py` (lines 200-300 region for config loading)
- Current mitigation: Config directory typically in user home directory with default permissions
- Recommendations: Set umask to 0o077 when creating config files. Document that configs should not contain sensitive calibration data if they need to be shared.

**JSON Deserialization Without Validation:**
- Risk: `driver_node.py` line 706 and `recorder_tui.py` line 198 deserialize JSON without schema validation
- Files: `myactuator_python_driver/driver_node.py` (lines 706-715), `myactuator_python_driver/recorder_tui.py` (lines 193-200)
- Current mitigation: Source is internal ROS topic (trusted), not external input
- Recommendations: Add schema validation or type checking. Catch `json.JSONDecodeError` explicitly.

## Performance Bottlenecks

**Polling-Based State Updates:**
- Problem: Driver node calls `motor.get_state()` in control loop for every motor in sequence, blocking on each CAN read
- Files: `myactuator_python_driver/driver_node.py` (line 292 region in main loop)
- Cause: Synchronous CAN requests with timeout per motor. On busy CAN bus, reading 8 motors sequentially can easily exceed 100ms loop time.
- Improvement path: Implement multi-motor request batching. Use CAN frame queuing to pipeline reads. Consider async I/O for driver communication.

**Trigger Checking in Tight Loop:**
- Problem: Hysteresis trigger state evaluation happens every control cycle for every joint, even when no playback active
- Files: `myactuator_python_driver/driver_node.py` (lines 543-580 in effort control)
- Cause: No early exit for inactive triggers. Floating point comparisons happen even when trigger disabled.
- Improvement path: Cache trigger state, skip evaluation if no active triggers. Use integer comparisons for thresholds.

**Bag Reader Metadata Recalculation:**
- Problem: Each time TUI updates recording list, it opens and scans entire bag to count frames
- Files: `myactuator_python_driver/recorder_tui.py` (lines 220-245)
- Cause: Bag metadata parsed from scratch. Frame count requires iterating all messages.
- Improvement path: Cache recording metadata with mtime check. Store frame count in separate JSON file. Lazy-load metadata only when needed.

**Curses Screen Refresh on Every State Change:**
- Problem: TUI redraws entire screen every time lock is released, even if state unchanged
- Files: `myactuator_python_driver/recorder_tui.py` (line 800+ in rendering code)
- Cause: No dirty-bit tracking. Full redraw on every `_run_tui()` loop iteration.
- Improvement path: Implement dirty region tracking. Only update changed fields. Use curses window dirty bit mechanism.

## Fragile Areas

**Motor State Caching Without Timestamps:**
- Files: `myactuator_python_driver/motor_wrapper.py` (line 97, 130-137)
- Why fragile: `_last_state` is cached but has no timestamp. If motor communication fails for extended period, stale state from minutes ago is returned. Control algorithms may not detect the age of the data.
- Safe modification: Add `timestamp` field to `MotorState`. Update on every successful read. Implement age check in `get_state()` to fail fast if data older than configured threshold.
- Test coverage: No tests verify age checking or cache invalidation on timeout.

**Global Driver Instance Lifecycle:**
- Files: `myactuator_python_driver/driver_node.py` (lines 56-99 init, 279-300 main loop)
- Why fragile: CAN driver created once in `_init_hardware()`. If hardware connection fails, entire node continues running but all motor commands silently fail. No mechanism to reinitialize or handle hot-plug reconnection.
- Safe modification: Add hardware health check callback. Implement graceful degradation mode. Allow runtime reconnection attempts.
- Test coverage: No tests for hardware failure recovery or reconnection scenarios.

**Concurrent TUI and Studio Access:**
- Files: `myactuator_python_driver/recorder_tui.py`, `myactuator_python_driver/studio/main.py`
- Why fragile: Both programs access same recordings directory and may be launched simultaneously. No mutual exclusion for recording operations. Bag writer state not validated.
- Safe modification: Implement process-level lockfile. Check if recording already in progress before starting new one. Validate bag writer state before every write.
- Test coverage: No tests for concurrent access scenarios.

**Trigger Configuration Persistence Without Validation:**
- Files: `myactuator_python_driver/config.py` (lines 50-75), `myactuator_python_driver/driver_node.py` (lines 612-650)
- Why fragile: Trigger thresholds loaded from YAML without range validation. Invalid thresholds can cause immediate crashes or infinite loops in trigger state machine.
- Safe modification: Add `__post_init__` validation. Clamp thresholds to motor limits. Add unit consistency checks (rad vs deg confusion).
- Test coverage: No tests for invalid trigger configurations.

## Scaling Limits

**Single-Threaded Motor Communication:**
- Current capacity: ~10-15 motors on 1Mbps CAN bus with 100ms control loop
- Limit: Where CAN read/write time exceeds loop period. With 8 motors and 10-15ms timeout per motor, ~80-120ms loop time.
- Scaling path: Implement request/response queuing. Use CAN address filtering to pipeline multiple motor reads. Consider slower loop rate (50Hz) for large deployments.

**Memory Accumulation in Bag Reader:**
- Current capacity: Limited by available RAM when reading large bags (>100MB)
- Limit: `rosbag2_py.SequentialReader` buffers frames in memory without streaming. Very long recordings cause OOM errors.
- Scaling path: Implement chunked reading with fixed buffer size. Process frames one at a time. Use memory-mapped I/O for metadata.

**Motor Recording File Growth:**
- Current capacity: No quota enforcement. Recordings directory can fill disk
- Limit: MCAP format with lossless compression still creates ~50-100KB per second of recording
- Scaling path: Implement recording quota system. Add automatic cleanup of old recordings. Compress archived bags with zstd.

## Dependencies at Risk

**rosbag2_py API Fragility:**
- Risk: Recent ROS 2 versions (Jazzy, Humble) changed rosbag2 API multiple times. Code uses `rosbag2_py.SequentialReader` and `SequentialWriter` which have had signature changes.
- Impact: Bag operations fail silently with obscure errors on version mismatches. Recording and playback completely broken on new ROS versions.
- Migration plan: Pin rosbag2 version in package.xml. Implement version detection. Create adapter layer for API differences. Consider using rosbag2 CLI instead of Python bindings for recording.

**pybind11 C++ Bindings Version Lock:**
- Risk: Python bindings built with specific pybind11 version. Mismatch causes import failures or segfaults.
- Impact: Package installation fails on systems with different pybind11 version. Binary wheels incompatible across Python versions.
- Migration plan: Build against stable pybind11 release. Publish binary wheels for common platforms. Consider moving to ctypes/cffi for better compatibility.

**ROS 2 Framework Migration Path Unclear:**
- Risk: codebase targets Humble but ROS 2 roadmap shows end-of-life in 2027. Code uses deprecated APIs (`destroy_node()` patterns).
- Impact: Future ROS 2 versions may remove APIs used in shutdown path. Unknown upgrade effort for Jazzy -> next LTS.
- Migration plan: Target latest LTS (Jazzy). Test with nav2 and modern ecosystem. Remove deprecated patterns.

## Missing Critical Features

**No Motor Communication Timeout Recovery:**
- Problem: When CAN bus stalls or motor stops responding, driver hangs in timeout. No exponential backoff or circuit breaker pattern.
- Blocks: Cannot implement reliable production deployment. Can't detect hardware failures in time.
- Recommendation: Implement adaptive timeout that increases after repeated failures. Add circuit breaker to skip motors that are consistently unresponsive.

**No Persistent Motor Error Logging:**
- Problem: Motor error codes (overheat, stall, overload) are read but not stored. Operator can't see error history after the fact.
- Blocks: Can't diagnose field failures or identify problematic motor units.
- Recommendation: Log error codes to CSV or database. Implement error count threshold for alerts. Add motor health dashboard.

**No Configuration Validation at Load Time:**
- Problem: Invalid configuration silently falls back to defaults. User doesn't know their config was ignored.
- Blocks: Difficult to catch configuration errors during development.
- Recommendation: Validate entire config at startup. Report specific errors. Fail fast with helpful error messages.

**No Soft Limits or Safe Movement Zones:**
- Problem: Can send motors to positions outside mechanical range. Leads to mechanical damage.
- Blocks: Can't implement collaborative robot use cases where safety is critical.
- Recommendation: Add min/max position limits per joint. Validate setpoints before sending. Implement velocity ramp limiting.

## Test Coverage Gaps

**Hardware Integration Tests Not Runnable:**
- What's not tested: Actual CAN communication, motor response, real error conditions
- Files: `myactuator_rmd/test/` directory
- Risk: Code may fail on real hardware even if unit tests pass. Motor initialization edge cases untested.
- Priority: High - Real hardware is only integration point

**Recording Playback Race Conditions Not Tested:**
- What's not tested: Concurrent recording + playback, trigger state transitions during playback, bag corruption recovery
- Files: `myactuator_python_driver/recorder_tui.py`, `myactuator_python_driver/studio/recording_manager.py`
- Risk: Race conditions between threads only manifest under load or timing-sensitive scenarios.
- Priority: High - Affects reliability of recording feature

**Configuration Invalid Input Not Tested:**
- What's not tested: Trigger thresholds outside valid ranges, invalid joint names, missing required fields
- Files: `myactuator_python_driver/config.py`
- Risk: Malformed configs cause crashes at runtime rather than clear errors at load time.
- Priority: Medium - Affects user experience

**Error Handling in Motor Wrapper Not Tested:**
- What's not tested: Timeout behavior, CAN errors, partial responses, what happens to cached state after failures
- Files: `myactuator_python_driver/motor_wrapper.py`
- Risk: Silent failures in motor communication make debugging impossible. Users see "no motion" with no error indication.
- Priority: High - Core functionality

---

*Concerns audit: 2026-02-09*
