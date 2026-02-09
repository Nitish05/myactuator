# Technology Stack

**Analysis Date:** 2026-02-09

## Languages

**Primary:**
- C++17 - `myactuator_rmd` low-level CAN driver SDK and core motor control logic
- Python 3 - `myactuator_python_driver` ROS 2 node, TUI, and GUI applications
- URDF/XML - `TandP_URDF_description` robot kinematic descriptions

**Secondary:**
- CMake - Build system for C++ components
- setuptools/Python - Package management for Python components
- YAML - Configuration files (`driver_config.yaml`)

## Runtime

**Environment:**
- ROS 2 (Jazzy) - Middleware for distributed motor control and messaging
- Linux (required) - Kernel support for SocketCAN interface, no Windows/macOS support
- C++17 compiler (GCC/Clang) - Required for building native SDK

**Package Manager:**
- pip/setuptools - Python packages (primary)
- ament (ROS 2 build system) - Workspace coordination and build management
- apt - System packages for ROS 2 dependencies
- Lockfile: `none for pip` (setup.py declares hard dependencies instead)

## Frameworks

**Core:**
- ROS 2 - Message passing, service infrastructure, distributed node execution
- pybind11 - Python-C++ bindings for `myactuator_rmd_py` module

**User Interface:**
- PyQt6 6.4.0+ - Desktop GUI framework for Motor Recording Studio
- Rich - Terminal UI formatting and styling for TUI applications
- curses - Low-level terminal UI for recorder TUI
- qt-material 2.14+ - Material Design theme for PyQt6 (optional)

**Testing:**
- GTest (Google Test) - C++ unit testing framework for `myactuator_rmd`
- pytest - Python test framework (configured in package.xml but not actively used)
- GMock - C++ mocking library for mock actuator tests

**Build/Dev:**
- CMake 3.20+ - C++ build configuration and compilation
- colcon - ROS 2 workspace build orchestration

## Key Dependencies

**Critical:**
- `rclpy` - ROS 2 Python client library for motor control node
- `rosbag2_py` 2.x (Jazzy) - ROS 2 bag recording/playback for motor trajectories
- `rosbag2_storage_mcap` - MCAP format storage backend for rosbag2
- `sensor_msgs` - ROS message type for JointState (position/velocity/effort)
- `std_msgs` - ROS standard message types (Float64MultiArray, String, Bool)
- `std_srvs` - ROS standard service types (Trigger, SetBool)
- `pybind11_vendor` - CMake dependency for Python bindings
- `myactuator_rmd` - C++ motor control SDK (internal, built from source)

**Infrastructure:**
- `ament_cmake` - CMake integration for ROS 2 packages
- `ament_cmake_python` - Python integration for ROS 2 packages
- `python_cmake_module` - CMake helpers for Python compatibility
- `ament_index_python` - Package discovery utilities

**TUI/GUI Utilities:**
- `PyQt6` - Desktop GUI framework
- `Rich` - Terminal output formatting
- `qt-material` - Optional dark theme styling

## Configuration

**Environment:**
- CAN interface name (e.g., `can0`, `vcan_test`) - Set via config YAML or environment
- Motor CAN IDs (1-255) - Defined per motor in `driver_config.yaml`
- ROS 2 domain ID - Controls network namespace isolation
- ROS 2 QoS policies - Reliability/history settings in driver node

**Build:**
- `CMakeLists.txt` (myactuator_rmd) - C++ SDK build configuration
  - `PYTHON_BINDINGS=on` - Enable pybind11 Python module compilation
  - `BUILD_TESTING=on` - Include GTest unit tests
  - `SETUP_TEST_IFNAME=on` - Auto-setup vcan_test interface for CI
  - `ENABLE_COVERAGE=on` - GCC/Clang coverage reporting
- `setup.py` (myactuator_python_driver) - Python package metadata and entry points
- `package.xml` (all packages) - ROS 2 package manifest with build/runtime dependencies

**Configuration Files:**
- `/home/nitish/work/myactuator/myactuator_python_driver/config/driver_config.yaml` - Motor configuration, CAN interface, publish rate

## Platform Requirements

**Development:**
- Linux (required) - SocketCAN only available on Linux kernel
- Python 3.7+ (tested with 3.13.11)
- C++ compiler with C++17 support (GCC 7+, Clang 5+)
- ROS 2 Jazzy distribution installed
- Virtual CAN interface support for testing: `vcan` kernel module

**Production:**
- Linux host running ROS 2 Jazzy
- CAN interface adapter (e.g., candleLight/SLCAN USB) connected to RMD motor(s)
- SocketCAN configured and up (bitrate 1000000)
- ROS 2 domain network connectivity between driver node and control nodes

---

*Stack analysis: 2026-02-09*
