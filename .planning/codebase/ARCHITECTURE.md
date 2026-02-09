# Architecture

**Analysis Date:** 2026-02-09

## Pattern Overview

**Overall:** Layered multi-language architecture with C++ low-level CAN protocol handling and Python ROS 2 integration layer.

**Key Characteristics:**
- Hardware abstraction through three distinct layers: CAN/protocol, driver interface, and ROS 2 node
- Language separation: C++17 for performance-critical CAN communication, Python for ROS 2 integration
- Per-motor interface design allowing independent or coordinated control of multiple motors
- Event-driven control loop with multiple operation modes (position, velocity, torque, free, force-controlled)

## Layers

**CAN/Socket Layer (`can` namespace):**
- Purpose: Linux SocketCAN interface abstraction for hardware communication
- Location: `myactuator_rmd/include/myactuator_rmd/can/` and `myactuator_rmd/src/can/`
- Contains: Frame handling (`Frame`), node abstraction (`Node`), utilities for CAN operations
- Depends on: Linux SocketCAN API
- Used by: Driver layer (`CanNode`, `CanDriver`)

**Protocol Layer (`protocol` namespace):**
- Purpose: RMD X-series motor protocol encoding/decoding (V3.8)
- Location: `myactuator_rmd/include/myactuator_rmd/protocol/` and `myactuator_rmd/src/protocol/`
- Contains: Message base class, request/response types, command type enums
- Depends on: CAN frame layer
- Used by: Driver layer (`CanNode`) to construct and parse motor commands
- Key abstractions: `Message` (base for all CAN payloads), `SingleMotorMessage` (command/response wrapper)

**Driver Layer:**
- Purpose: Command-response protocol execution and motor addressing
- Location: `myactuator_rmd/include/myactuator_rmd/driver/`
- Contains:
  - `CanNode<SEND_OFFSET, RECV_OFFSET>` - Template base class managing send/receive CAN IDs
  - `CanDriver` - Concrete CAN driver using offsets 0x140/0x240
  - `Driver` abstract interface for command execution
- Depends on: Protocol and CAN layers
- Used by: Actuator interface layer

**Actuator Interface Layer:**
- Purpose: Per-motor high-level API for control and state queries
- Location: `myactuator_rmd/include/myactuator_rmd/actuator_interface.hpp` and `myactuator_rmd/src/actuator_interface.cpp`
- Contains: `ActuatorInterface` class with methods for position/velocity/torque/current commands, status reads
- Depends on: Driver layer
- Used by: Python bindings and ROS 2 wrapper
- Key methods: `sendPositionAbsoluteSetpoint()`, `sendVelocitySetpoint()`, `sendTorqueSetpoint()`, `getMotorStatus2()`

**Python Bindings Layer:**
- Purpose: C++ to Python interface via pybind11
- Location: `myactuator_rmd/bindings/myactuator_rmd.cpp`
- Exposes: `ActuatorInterface`, `CanDriver`, `can.Node`, `can.Frame`, all state structures
- Depends on: C++ layers
- Used by: ROS 2 Python driver

**ROS 2 Wrapper Layer:**
- Purpose: Unit conversion and high-level motor state abstraction
- Location: `myactuator_python_driver/myactuator_python_driver/motor_wrapper.py`
- Contains: `MotorWrapper` class with ROS units (radians, rad/s, Nm instead of degrees, dps, current)
- Key features:
  - Position offset and direction inversion support
  - Unit conversion (degrees ↔ radians, current → torque)
  - Caching last known state on communication errors
  - Raw CAN interface for V4 force-controlled position commands (0xA9)
- Depends on: Python bindings
- Used by: ROS 2 driver node

**ROS 2 Driver Node Layer:**
- Purpose: ROS 2 topic/service management and high-level control modes
- Location: `myactuator_python_driver/myactuator_python_driver/driver_node.py`
- Contains: `MotorDriverNode` managing multiple motors with coordinated control
- Key features:
  - Multiple control modes: position, velocity, torque, free (hand-movable), admittance control, force-controlled position
  - Hybrid playback with per-joint torque override triggers (hysteresis-based)
  - Threading with reentrant callback group for concurrent topic processing
  - State management for mode transitions and position capture
- Depends on: Motor wrapper, configuration, ROS 2 libraries
- Used by: ROS 2 ecosystem (launch files, external controllers)

**Configuration & Utilities:**
- Location: `myactuator_python_driver/myactuator_python_driver/config.py`
- Contains: `DriverConfig`, `MotorConfig` dataclasses with YAML serialization, `HysteresisTorqueTrigger`, `PlaybackTriggerConfig`
- Purpose: Centralized configuration management for motor IDs, torque constants, position offsets, control modes

**GUI Layer (Motor Studio):**
- Purpose: Qt-based monitoring and control interface
- Location: `myactuator_python_driver/myactuator_python_driver/studio/`
- Contains: Main window, widgets (monitor, record, playback, control), models, dialogs
- Depends on: ROS 2 driver node (via ROS bridge)
- No control of core driver logic (observes via topics)

## Data Flow

**Motor State Read Path (Polling):**

1. ROS 2 driver control loop timer (configurable Hz) triggers
2. `MotorDriverNode._control_loop()` acquires lock and iterates over motors
3. For each motor: `MotorWrapper.get_state()` calls
4. Wrapper calls `ActuatorInterface.getMotorStatus2()` (C++ bindings)
5. ActuatorInterface sends 0x9C command via `CanDriver.sendRecv()`
6. CanDriver constructs CAN frame via protocol layer, sends via `CanNode`
7. `CanNode` uses Linux SocketCAN to transmit on interface (0x140 + motor_id)
8. Motor responds on 0x240 + motor_id
9. CanNode receives, protocol layer parses MotorStatus2 struct
10. Feedback flows back: ActuatorInterface → MotorWrapper → MotorState dataclass
11. Driver node publishes to `/joint_states` topic (JointState message)

**Motor Control Command Path (Position Mode Example):**

1. External controller publishes to `/joint_state_ctrl` topic with target position
2. `_joint_ctrl_callback()` receives, stores position in `_control_positions` dict (locked)
3. Control loop iterates, checks mode flag `_torque_mode`, `_free_mode`, etc.
4. Mode is position (default): calls `MotorWrapper.send_position(position_rad)`
5. Wrapper converts radians to degrees, applies offset/inversion
6. Calls `ActuatorInterface.sendPositionAbsoluteSetpoint(position_deg, max_speed_deg)`
7. ActuatorInterface constructs 0xA4 command via protocol layer
8. CanDriver sends via CanNode, receives feedback response (immediate motor state)
9. Feedback parsed to MotorState, returned through call stack
10. New state used in next iteration of control loop
11. Published to `/joint_states`

**Trigger-Based Playback Mode:**

1. Playback data (recorded positions) flows in via `/joint_state_ctrl`
2. Control loop checks `_active_triggers` dict for this joint
3. `_evaluate_trigger()` examines current recorded position against hysteresis thresholds
4. On threshold crossing, switches `_trigger_states` from "inactive" → "active" or vice versa
5. When active: sends recorded torque command instead of recorded position
6. When inactive: returns to position tracking
7. Trigger state changes logged and published to `~/trigger_states` for TUI feedback

**State Management:**

State is held in three places:
- `MotorWrapper._last_state`: Cache of last successful read (returns on error)
- `MotorDriverNode._control_positions/velocities/efforts`: Current command targets
- `MotorDriverNode._active_triggers`: Playback trigger configuration
- `MotorDriverNode._trigger_states`: Current state ("active"/"inactive") of each trigger

All access protected by `_lock` (threading.Lock) to prevent race conditions during mode transitions.

## Key Abstractions

**ActuatorInterface (Motor Command API):**
- Purpose: Encapsulate all communication with a single motor
- Examples: `myactuator_rmd/include/myactuator_rmd/actuator_interface.hpp`
- Pattern: High-level methods (sendPositionAbsoluteSetpoint, getMotorStatus2) abstract protocol details
- Constructor takes `Driver&` (dependency injection) enabling mock testing

**CanDriver (Hardware Abstraction):**
- Purpose: Manage SocketCAN interface and route messages to motors by ID
- Location: `myactuator_rmd/include/myactuator_rmd/driver/can_driver.hpp`
- Pattern: Inherits from `CanNode<0x140, 0x240>` template (send/recv offsets)
- Multiple ActuatorInterface instances reference same CanDriver instance

**MotorWrapper (ROS Unit Bridge):**
- Purpose: Convert between ROS units (radians, rad/s, Nm) and motor units (degrees, dps, current)
- Location: `myactuator_python_driver/myactuator_python_driver/motor_wrapper.py`
- Pattern: Dataclass `MotorState` carries converted values; methods apply offset/inversion
- Supports motor-specific configuration (torque_constant, position_offset, inverted)

**MotorState (Immutable State Value):**
- Purpose: Carry immutable motor telemetry snapshot
- Definition in `myactuator_python_driver/myactuator_python_driver/motor_wrapper.py` (dataclass)
- Contains: position_rad, velocity_rad_s, effort_nm, temperature_c, voltage_v, current_a, error_code
- Used throughout: type safety and clear data flow

**DriverConfig (Configuration Container):**
- Purpose: Hold all driver-level settings and motor configurations
- Location: `myactuator_python_driver/myactuator_python_driver/config.py`
- Pattern: Dataclass with YAML serialization, default path ~/.myactuator/config.yaml
- Contains: can_interface, publish_rate, control_mode, list of MotorConfig objects
- Each MotorConfig specifies: can_id, joint_name, torque_constant, position_offset, inverted, max_velocity

**HysteresisTorqueTrigger (Playback Trigger):**
- Purpose: Define when to switch from position to torque control during playback
- Location: `myactuator_python_driver/myactuator_python_driver/config.py`
- Pattern: Hysteresis prevents oscillation at threshold; separate enter/exit thresholds
- Supports rising/falling direction; tracks enable/disabled state per trigger

## Entry Points

**C++ Library (Command-line/Standalone):**
- Location: `myactuator_rmd/include/myactuator_rmd/myactuator_rmd.hpp`
- Triggers: Direct instantiation of CanDriver and ActuatorInterface
- Responsibilities: Motor control via C++ API (no ROS 2 required)
- Example: Tests in `myactuator_rmd/test/actuator_test.cpp`

**ROS 2 Driver Node:**
- Location: `myactuator_python_driver/myactuator_python_driver/driver_node.py`
- Triggers: `ros2 run myactuator_python_driver driver_node` (via entry_points in setup.py)
- Responsibilities:
  - Load config from file or ROS parameters
  - Initialize CAN driver and motors
  - Publish joint states and motor status at configurable rate
  - Handle incoming control commands and service calls
- Topics:
  - Pub: `/joint_states`, `/motor_status`, `~/mode`, `~/trigger_states`
  - Sub: `/joint_state_ctrl`, `/joint_effort_ctrl`, `~/set_mode`, `~/set_enabled`, `~/playback_triggers`
- Services: `~/set_zero`, `~/emergency_stop`, `~/enable_torque`, `~/set_free`

**Setup/Configuration TUI:**
- Location: `myactuator_python_driver/myactuator_python_driver/setup_tui.py`
- Triggers: `ros2 run myactuator_python_driver setup_tui`
- Responsibilities: Interactive wizard to detect motors on CAN bus, configure motor IDs/names, save config

**Recording/Playback TUI:**
- Location: `myactuator_python_driver/myactuator_python_driver/recorder_tui.py`
- Triggers: `ros2 run myactuator_python_driver recorder_tui`
- Responsibilities: Record joint state trajectories, playback with per-joint torque triggers

**Motor Studio GUI:**
- Location: `myactuator_python_driver/myactuator_python_driver/studio/main.py`
- Triggers: `ros2 run myactuator_python_driver studio` (via launch file or direct)
- Responsibilities: Monitor live motor states, configure triggers, browse/manage recordings, playback control

**URDF Description Package:**
- Location: `TandP_URDF_description/`
- Purpose: Robot description for visualization in RViz and Gazebo simulation
- Entry points: Launch files for display.launch.py and gazebo.launch.py

## Error Handling

**Strategy:** Graceful degradation with fallback to last known state

**Patterns:**

1. **CAN Communication Errors:**
   - Location: `myactuator_python_driver/myactuator_python_driver/motor_wrapper.py` lines 139-141
   - On exception during `get_state()`: return cached `_last_state` rather than throwing
   - Driver node logs warning with throttling (5-second minimum between logs)
   - Joint state published with last known position/velocity/effort

2. **Hardware Initialization Failures:**
   - Location: `myactuator_python_driver/myactuator_python_driver/driver_node.py` lines 99-101
   - If CAN driver fails to initialize: continue in simulation mode
   - Motor objects are empty, zeros published for all joints
   - Allows testing without physical hardware

3. **Python Bindings Not Available:**
   - Location: `myactuator_python_driver/myactuator_python_driver/motor_wrapper.py` lines 12-25
   - Check `RMD_AVAILABLE` flag before instantiating motors
   - Enables pure-Python testing/simulation

4. **Mode Transition Errors:**
   - Location: `myactuator_python_driver/myactuator_python_driver/driver_node.py` lines 477-486
   - When entering torque mode from free mode: capture current positions first
   - Prevents abrupt position jumps on mode change

5. **Timeout Protection:**
   - Location: `myactuator_rmd/include/myactuator_rmd/actuator_interface.hpp` lines 357-365
   - `setTimeout()` sets communication interruption protection time
   - Motor automatically triggers brake if no command for N milliseconds
   - MotorWrapper sets default 100ms timeout (may fail on some firmware)

## Cross-Cutting Concerns

**Logging:**
- C++ layer: Uses standard logging patterns, some debug output via io.hpp
- Python driver: Uses ROS 2 logger (`self.get_logger().info/warn/error`)
- TUI applications: Print to stderr for user feedback
- Driver node logs mode changes, motor init, trigger state changes, errors with throttling

**Validation:**

1. **Motor ID Range:** [1, 32] enforced by protocol (CAN ID + offset)
2. **Torque Constant:** Required per motor type, from `actuator_constants.hpp`
3. **Position Offset:** Applied consistently in wrapper (add when sending, subtract on reading)
4. **Hysteresis Trigger:** Validated in `HysteresisTorqueTrigger.__post_init__()` (exit must be on opposite side of enter)
5. **Control Mode:** String-based mode validation in `_set_mode_callback()` with fallback to position
6. **Position/Velocity Bounds:** No hard bounds enforced; motor firmware handles saturation

**Authentication:**
- None (assumes trusted network; motors accessible only on local CAN bus)

**Thread Safety:**

1. **Lock Usage:** `MotorDriverNode._lock` (threading.Lock) protects:
   - Motor command state: `_control_positions`, `_control_velocities`, `_control_efforts`
   - Mode flags: `_torque_mode`, `_free_mode`, `_admittance_mode`, etc.
   - Trigger configuration: `_active_triggers`, `_trigger_states`
   - Direct motor access during shutdown

2. **Callback Execution:** All callbacks (topic, service, timer) run in `ReentrantCallbackGroup`
   - Allows concurrent execution of independent callbacks
   - Lock acquired for brief periods during state updates
   - No blocking I/O inside locked regions

3. **Motor Access:** Each motor (MotorWrapper) does not need locks
   - CanDriver is thread-safe internally (sendRecv blocks until response)
   - Only called from control loop or mode transition code (lock-protected)

**Unit Conversions:**

Established patterns in `MotorWrapper`:
- `deg_to_rad(deg: float) -> float`: degrees * π / 180
- `rad_to_deg(rad: float) -> float`: radians * 180 / π
- Current to torque: `current_a * torque_constant` (Nm)
- Torque to current: `torque_nm / torque_constant` (implicit in sendTorqueSetpoint)
- Motor position units: degrees (0.01 degree resolution in protocol)
- Motor velocity units: dps (degree per second, 1 dps resolution)
- Motor current units: 0.01 A resolution in feedback

---

*Architecture analysis: 2026-02-09*
