# Codebase Structure

**Analysis Date:** 2026-02-09

## Directory Layout

```
myactuator/
├── myactuator_rmd/                    # C++17 low-level motor SDK
│   ├── include/myactuator_rmd/
│   │   ├── actuator_interface.hpp     # Main motor API
│   │   ├── actuator_constants.hpp     # Motor specs (torque constants)
│   │   ├── can/                       # SocketCAN abstraction layer
│   │   ├── driver/                    # CAN driver and addressing
│   │   ├── protocol/                  # RMD V3.8 message encoding
│   │   ├── actuator_state/            # State structs (MotorStatus1/2/3, Gains)
│   │   ├── exceptions.hpp             # Custom exception types
│   │   └── version.hpp                # Version constants
│   ├── src/
│   │   ├── actuator_interface.cpp     # Motor API implementation
│   │   ├── can/                       # SocketCAN node implementation
│   │   └── protocol/                  # Protocol encoding/decoding
│   ├── bindings/
│   │   └── myactuator_rmd.cpp         # pybind11 Python bindings
│   ├── test/                          # GTest unit tests
│   │   ├── actuator_test.cpp          # API tests with mock
│   │   ├── mock/                      # Mock actuator fixtures
│   │   ├── can/                       # SocketCAN tests
│   │   └── protocol/                  # Protocol encoding tests
│   ├── cmake/                         # CMake config templates
│   ├── CMakeLists.txt                 # Build configuration
│   ├── setup.py                       # Python package metadata
│   └── package.xml                    # ROS 2 package manifest
│
├── myactuator_python_driver/          # ROS 2 driver node
│   ├── myactuator_python_driver/
│   │   ├── driver_node.py             # Main ROS 2 node (MotorDriverNode)
│   │   ├── motor_wrapper.py           # Motor unit wrapper (MotorWrapper)
│   │   ├── config.py                  # Configuration classes
│   │   ├── setup_tui.py               # Setup wizard TUI
│   │   ├── recorder_tui.py            # Recording/playback TUI
│   │   ├── can_utils.py               # CAN utilities (setup helpers)
│   │   ├── __init__.py                # Package initialization
│   │   └── studio/                    # Motor Studio GUI
│   │       ├── main.py                # Qt application entry
│   │       ├── main_window.py         # Main window widget
│   │       ├── recording_manager.py   # Recording data management
│   │       ├── ros_bridge.py          # ROS 2 subscription interface
│   │       ├── models/                # Data models (joint_state, recording)
│   │       ├── widgets/               # Qt custom widgets (monitor, record, playback, control)
│   │       ├── dialogs/               # Dialog windows (trigger config)
│   │       └── resources/             # CSS styles
│   ├── launch/
│   │   ├── driver.launch.py           # Standalone driver launch
│   │   └── driver_with_studio.launch.py # Driver + GUI launch
│   ├── config/
│   │   └── driver_config.yaml         # Default config template
│   ├── recordings/                    # Recorded trajectory storage
│   │   ├── test1/, test2/, test3/     # Individual recordings with db3+metadata
│   │   └── triggers.json              # Persistent trigger configuration
│   ├── package.xml                    # ROS 2 package manifest
│   └── setup.py                       # Package entry points
│
├── TandP_URDF_description/            # Robot URDF for visualization
│   ├── urdf/                          # XACRO robot description files
│   ├── meshes/                        # STL model files
│   ├── launch/                        # Display and Gazebo launch files
│   └── config/                        # RViz configuration
│
├── recordings/                        # User recording storage (top-level)
│   ├── test1/, test2/, test3/         # Rosbag2 recordings
│   └── triggers.json                  # Global trigger file
│
├── CLAUDE.md                          # This file (project instructions)
├── README.md                          # Repository overview
└── connect_can.sh                     # CAN interface setup script
```

## Directory Purposes

**`myactuator_rmd/include/myactuator_rmd/`:**
- Purpose: Public API headers for C++ SDK
- Contains: Interface definitions, protocol specs, state types
- Key files:
  - `actuator_interface.hpp` - Main per-motor API (read status, send commands)
  - `actuator_constants.hpp` - Motor specifications (torque constants for X8/X8PRO/X10/X12)
  - `can/` - SocketCAN abstraction (`can::Node`, `can::Frame`)
  - `driver/` - Multi-motor driver (`CanDriver`, `CanNode`, addressing)
  - `protocol/` - RMD V3.8 message types (requests, responses, command codes)
  - `actuator_state/` - State structs returned by motor (MotorStatus1/2/3, Gains, ControlMode)

**`myactuator_rmd/src/`:**
- Purpose: C++ implementation
- Contains: Driver logic, CAN I/O, protocol parsing
- Key files:
  - `actuator_interface.cpp` - Implementation of motor control methods
  - `can/node.cpp` - SocketCAN read/write via Linux
  - `protocol/` - Parsing responses and encoding requests

**`myactuator_rmd/bindings/`:**
- Purpose: C++ to Python bridge via pybind11
- Contains: Single `myactuator_rmd.cpp` file that exposes all C++ classes
- Exposes: ActuatorInterface, CanDriver, can.Node, can.Frame, all state enums/structs
- Builds to: `myactuator_rmd_py` module (importable as `from myactuator_rmd import myactuator_rmd_py`)

**`myactuator_rmd/test/`:**
- Purpose: GTest unit tests with mock actuator
- Contains: Test fixtures, mock objects, protocol validation
- Key files:
  - `actuator_test.cpp` - Tests motor API with mock
  - `mock/` - Mock actuator that simulates motor responses
  - `can/`, `protocol/` - Lower-layer unit tests

**`myactuator_python_driver/myactuator_python_driver/`:**
- Purpose: ROS 2 integration layer and application entry points
- Core components:
  - `driver_node.py` - MotorDriverNode (main ROS 2 node)
  - `motor_wrapper.py` - MotorWrapper (unit conversion, state caching)
  - `config.py` - Configuration structures and persistence
  - `setup_tui.py` - Interactive motor discovery and config wizard
  - `recorder_tui.py` - Recording/playback control TUI
  - `studio/` - Qt-based GUI for monitoring and control

**`myactuator_python_driver/studio/`:**
- Purpose: Motor Studio GUI application
- Contains: Qt widgets, models, dialogs
- Key files:
  - `main.py` - QApplication entry point
  - `main_window.py` - Main window layout and coordination
  - `widgets/` - Tab widgets (monitor, record, playback, control panel)
  - `models/` - Data models for joint states and recordings
  - `ros_bridge.py` - ROS 2 subscription glue code
  - `recording_manager.py` - Manages rosbag2 recordings on disk

**`myactuator_python_driver/launch/`:**
- Purpose: ROS 2 launch configurations
- Files:
  - `driver.launch.py` - Starts driver node only
  - `driver_with_studio.launch.py` - Starts driver node and Motor Studio GUI

**`myactuator_python_driver/config/`:**
- Purpose: Default configuration template
- File: `driver_config.yaml` - Example config with CAN interface, motor IDs, torque constants

**`myactuator_python_driver/recordings/`:**
- Purpose: Local recording storage (individual recording directories + trigger file)
- Structure:
  - `test1/`, `test2/`, etc. - One directory per recording
    - `metadata.yaml` - Rosbag2 metadata
    - `test1_0.db3` - SQLite database with joint state messages
  - `triggers.json` - Persistent playback trigger configuration for all recordings

**`TandP_URDF_description/`:**
- Purpose: Robot description for visualization
- Contains: XACRO macros defining robot structure, STL meshes, display config
- Used by: RViz (visualization), Gazebo (simulation)

**Top-level `recordings/`:**
- Purpose: Alternative/backup recording storage location
- Structure: Same as `myactuator_python_driver/recordings/`
- Used when recordings saved to workspace root instead of package directory

## Key File Locations

**Entry Points:**
- C++ main API: `myactuator_rmd/include/myactuator_rmd/myactuator_rmd.hpp` (header-only public interface)
- Python ROS 2 node: `myactuator_python_driver/myactuator_python_driver/driver_node.py` (main() function lines 741-770)
- Setup wizard: `myactuator_python_driver/myactuator_python_driver/setup_tui.py` (main() function)
- Recorder TUI: `myactuator_python_driver/myactuator_python_driver/recorder_tui.py` (main() function)
- Motor Studio: `myactuator_python_driver/myactuator_python_driver/studio/main.py` (main() function)

**Configuration:**
- Runtime config: `~/.myactuator/config.yaml` (default location, loaded by `DriverConfig.default_config_path()`)
- Package config template: `myactuator_python_driver/config/driver_config.yaml`
- Trigger storage: `myactuator_python_driver/recordings/triggers.json` (persistent across sessions)
- CMake config: `myactuator_rmd/CMakeLists.txt` (Python bindings, testing flags)

**Core Logic:**
- Motor API (C++): `myactuator_rmd/include/myactuator_rmd/actuator_interface.hpp` (control commands, status queries)
- Motor wrapper (Python): `myactuator_python_driver/myactuator_python_driver/motor_wrapper.py` (MotorWrapper class, unit conversion)
- Driver node: `myactuator_python_driver/myactuator_python_driver/driver_node.py` (MotorDriverNode class, control modes, ROS topics/services)
- Configuration: `myactuator_python_driver/myactuator_python_driver/config.py` (DriverConfig, MotorConfig, HysteresisTorqueTrigger dataclasses)

**Testing:**
- C++ tests: `myactuator_rmd/test/actuator_test.cpp` (runs with CTest)
- Test fixtures: `myactuator_rmd/test/mock/actuator_actuator_mock_test.hpp` (mock motor for isolated testing)
- Virtual CAN setup: Required before testing (`sudo modprobe vcan && sudo ip link add dev vcan_test type vcan`)

**Utilities:**
- CAN setup script: `connect_can.sh` (interactive CAN interface configuration)
- CAN utilities: `myactuator_python_driver/myactuator_python_driver/can_utils.py` (setup helpers)

## Naming Conventions

**Files:**

- C++ headers: `.hpp` (e.g., `actuator_interface.hpp`)
- C++ implementation: `.cpp` (e.g., `actuator_interface.cpp`)
- Python files: `.py` with snake_case names (e.g., `driver_node.py`, `motor_wrapper.py`)
- Test files: `{module}_test.cpp` (e.g., `actuator_test.cpp`)
- Configuration: YAML with lowercase (e.g., `driver_config.yaml`)
- Launch files: `{name}.launch.py` (e.g., `driver.launch.py`)

**Directories:**

- Namespace-aligned to C++ namespaces: `include/myactuator_rmd/can/`, `include/myactuator_rmd/protocol/`, `include/myactuator_rmd/driver/`
- Python packages: All lowercase (e.g., `myactuator_python_driver`, `studio`)
- Module groups: Functional (e.g., `studio/models/`, `studio/widgets/`, `studio/dialogs/`)

**Classes & Functions:**

- C++ classes: PascalCase (e.g., `ActuatorInterface`, `CanDriver`, `MotorStatus2`)
- Python classes: PascalCase (e.g., `MotorDriverNode`, `MotorWrapper`, `DriverConfig`)
- Python functions: snake_case (e.g., `deg_to_rad()`, `_apply_offset()`)
- ROS 2 node: Always `MotorDriverNode`, main() entry in `driver_node.py`

**Variables:**

- C++ member variables: Suffix `_` (e.g., `driver_`, `actuator_id_`)
- Python private methods: Prefix `_` (e.g., `_control_loop()`, `_lock`)
- Protected variables: Prefix `_` (e.g., `_last_state`, `_control_positions`)
- Module-level constants: UPPER_CASE (ROS topics are lowercase with `/` prefix)

## Where to Add New Code

**New Motor Control Feature (e.g., new command type):**

1. Define protocol message in C++:
   - Add request type to `myactuator_rmd/include/myactuator_rmd/protocol/requests.hpp`
   - Add response type to `myactuator_rmd/include/myactuator_rmd/protocol/responses.hpp`

2. Add C++ motor command:
   - Add method to `myactuator_rmd/include/myactuator_rmd/actuator_interface.hpp`
   - Implement in `myactuator_rmd/src/actuator_interface.cpp`
   - Expose in pybind11 bindings: `myactuator_rmd/bindings/myactuator_rmd.cpp`

3. Add Python wrapper method:
   - Add method to `MotorWrapper` class in `myactuator_python_driver/myactuator_python_driver/motor_wrapper.py`
   - Handle unit conversion if needed (degrees ↔ radians, etc.)

4. Integrate with driver node:
   - Add ROS topic subscription or service in `myactuator_python_driver/myactuator_python_driver/driver_node.py`
   - Call wrapper method from appropriate callback or control loop

5. Test:
   - Add C++ GTest in `myactuator_rmd/test/` with mock actuator
   - Test Python wrapper unit conversions

**New Control Mode (e.g., impedance control):**

1. Add mode flag to `MotorDriverNode`:
   - New instance variable: `self._impedance_mode = False`
   - New control parameters: `self._impedance_stiffness`, `self._impedance_damping`

2. Add mode transition logic:
   - Update `_set_mode_callback()` to recognize "impedance" string
   - Set flag and log mode change
   - Capture positions before entering (same pattern as admittance mode)

3. Implement control in `_control_loop()`:
   - Add `elif self._impedance_mode:` branch
   - Calculate torque based on position error and velocity using impedance parameters
   - Call `motor.send_torque()` with calculated torque

4. Add ROS topic/service for mode control:
   - Subscribe to `~/set_impedance_params` to adjust stiffness/damping
   - Or add parameters and load from config

5. Update configuration:
   - Add impedance parameters to `MotorConfig` dataclass if motor-specific
   - Add to `driver_config.yaml` template

**New GUI Feature (e.g., motor parameter tuning dialog):**

1. Create dialog widget:
   - New file: `myactuator_python_driver/myactuator_python_driver/studio/dialogs/parameter_dialog.py`
   - Inherit from `QDialog`, create UI with sliders for PID gains

2. Integrate with main window:
   - Add button in `myactuator_python_driver/myactuator_python_driver/studio/main_window.py`
   - Connect button to dialog creation/show

3. Connect to driver node:
   - In dialog: publish to ROS service (e.g., `~/set_gains`) via `ros_bridge.py`
   - Driver node receives and applies via motor API

4. Update recording manager if needed:
   - If tuning affects recordings, update `myactuator_python_driver/myactuator_python_driver/studio/recording_manager.py`

**New ROS Topic/Service:**

1. Define message type (if not using std_msgs):
   - Add `.msg` or `.srv` file to `myactuator_python_driver/msg/` (create dir if needed)
   - Update `CMakeLists.txt` and `package.xml` to build messages
   - Regenerate with: `colcon build --packages-select myactuator_python_driver`

2. Add to driver node:
   - Create publisher/subscriber in `__init__()` with proper callback
   - Implement callback method with thread safety (acquire `self._lock`)

3. Document in docstring:
   - Update `MotorDriverNode` class docstring with topic/service description

**New Recording/Playback Feature:**

1. Extend configuration:
   - Add fields to `PlaybackTriggerConfig` or create new config class in `myactuator_python_driver/myactuator_python_driver/config.py`
   - Add YAML serialization with `to_dict()` / `from_dict()`

2. Update recorder TUI:
   - Add menu option in `myactuator_python_driver/myactuator_python_driver/recorder_tui.py`
   - Implement configuration and save to `triggers.json`

3. Update driver node:
   - Load new trigger config in initialization
   - Evaluate/apply in `_control_loop()` or add new mode

4. Update Motor Studio:
   - Add widget/dialog in `myactuator_python_driver/myactuator_python_driver/studio/widgets/` or `dialogs/`
   - Update `recording_manager.py` to persist configuration

## Special Directories

**`myactuator_rmd/test/mock/`:**
- Purpose: Mock actuator that simulates responses for isolated testing
- Generated: No (hand-written fixture)
- Committed: Yes
- Files: `actuator_actuator_mock_test.hpp` (GTest fixture with mock responses)
- Used by: All `actuator_test.cpp` unit tests

**`myactuator_python_driver/recordings/`:**
- Purpose: Persistent storage of recorded motor trajectories
- Generated: Yes (created by recorder_tui.py)
- Committed: No (user data directory)
- Structure: Per-recording directories with rosbag2 db3 + metadata
- File: `triggers.json` for trigger config across all recordings

**`TandP_URDF_description/meshes/`:**
- Purpose: STL geometry files for visualization
- Generated: No (CAD exports)
- Committed: Yes
- Files: `A_1.stl`, `B_1.stl`, `base_link.stl`, `C_1.stl`, `jig_1.stl`
- Used by: RViz/Gazebo for 3D rendering

**`myactuator_rmd/cmake/`:**
- Purpose: CMake module templates
- Generated: No
- Committed: Yes
- Files: `Config.cmake.in` (for package discovery), `CTestCustom.cmake.in` (test config)

**Top-level `log/` directory:**
- Purpose: ROS 2 build logs
- Generated: Yes (colcon build creates this)
- Committed: No (should be in `.gitignore`)
- Contains: Build logs for debugging build failures

---

*Structure analysis: 2026-02-09*
