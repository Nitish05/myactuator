# External Integrations

**Analysis Date:** 2026-02-09

## APIs & External Services

**Motor Hardware Control:**
- MyActuator RMD X-series motors - Low-level CAN protocol
  - SDK/Client: `myactuator_rmd_py` (pybind11 C++ bindings)
  - Protocol: RMD-X Servo Motor Control Protocol V3.8
  - Commands: Position (0xA4), Velocity (0xA2), Torque (0xA1), Status Query (0x9C), Shutdown (0x80), Stop (0x81)
  - CAN addressing: TX to `0x140 + motor_id`, RX from `0x240 + motor_id`

**ROS 2 Middleware:**
- ROS 2 Jazzy distribution
  - Node publication/subscription for distributed control
  - Service client/server for motor commands

## Data Storage

**Databases:**
- None - No persistent database used

**File Storage:**
- rosbag2 MCAP format - Motor trajectory recordings
  - Client: `rosbag2_py` (ROS 2 bag recording library)
  - Storage backend: `rosbag2_storage_mcap`
  - Format: MCAP (efficient binary recording of ROS messages)
  - Location: `workspace/recordings/` directory
  - Contents: `sensor_msgs/JointState` messages (position, velocity, effort per joint)
  - Integration points:
    - `myactuator_python_driver/recorder_tui.py` - Records bag files during motor operation
    - `myactuator_python_driver/studio/widgets/record_tab.py` - GUI recording interface
    - `myactuator_python_driver/studio/widgets/playback_tab.py` - Playback from recorded bags

**Local Filesystem:**
- Configuration YAML files: `myactuator_python_driver/config/driver_config.yaml`
- Trigger configuration JSON: Stored in `~/.myactuator/triggers.json` (path determined at runtime)
- CAN interface: Linux SocketCAN network interface (`can0`, `vcan_test`, etc.)

**Caching:**
- None - Real-time control loop, no caching layer

## Authentication & Identity

**Auth Provider:**
- None - Local system only, no cloud authentication

**Implementation:**
- Unix socket permissions - CAN interface access controlled by system user/group permissions
- ROS 2 domain isolation - Optional DDS domain separation via `ROS_DOMAIN_ID` environment variable

## Monitoring & Observability

**Error Tracking:**
- None - No external error tracking service

**Logging:**
- ROS 2 logging (rclpy/rcl) - Used in `driver_node.py` via `self.get_logger()`
- Terminal output - Rich library for formatted console messages
- File logging - Implicit ROS 2 logs at `~/.ros/log/`
- Log suppression: `myactuator_python_driver/studio/main.py` suppresses rosbag2 and ROS 2 internal logs

**Motor Status Monitoring:**
- Published via ROS 2 topics: `/motor_status` (temperature, voltage)
- Published via ROS 2 topics: `/joint_states` (position, velocity, effort)
- Real-time feedback from motor status registers (voltage, temperature, error codes)

## CI/CD & Deployment

**Hosting:**
- Local workstation/robot - No cloud hosting
- Robot/PC running ROS 2 Jazzy

**CI Pipeline:**
- None detected - No GitHub Actions, GitLab CI, or equivalent

**Testing Infrastructure:**
- Local pytest: Configured in `myactuator_python_driver/package.xml`
- Local GTest: Configured in `myactuator_rmd/CMakeLists.txt` with virtual CAN interface setup
- Manual testing via `recorder_tui` and Motor Recording Studio

## Environment Configuration

**Required env vars:**
- `ROS_DOMAIN_ID` (optional) - DDS domain isolation (default: 0)
- `CMAKE_ARGS` (optional, build time) - Additional CMake flags
- `CMAKE_BUILD_PARALLEL_LEVEL` (optional, build time) - Parallel build jobs
- `DEBUG` (optional, build time) - Enable Debug build (`myactuator_rmd/setup.py`)

**Optional env vars:**
- `AMENT_INDEX_PATH` - ROS 2 package discovery path
- `PYTHONPATH` - Python module search path (auto-configured by ROS 2)

**Secrets location:**
- None - No API keys, tokens, or credentials used

**CAN Interface Configuration:**
- Set via `driver_config.yaml`: `can_interface: can0`
- Runtime setup: `./connect_can.sh` script for interactive CAN adapter configuration
- Manual Linux commands: `ip link set can0 type can bitrate 1000000; ip link set up can0`

## Webhooks & Callbacks

**Incoming:**
- None - No webhook endpoints

**Outgoing:**
- None - No external webhook calls

**Internal ROS 2 Callbacks:**
- Subscription callbacks in `driver_node.py`:
  - `/joint_state_ctrl` (position/velocity setpoints)
  - `/joint_effort_ctrl` (torque setpoints)
  - `~/set_mode` (control mode changes)
  - `~/set_enabled` (enable/disable command)
  - `~/motor_trigger_config` (trigger configuration from TUI)
- Service callbacks:
  - `~/set_zero` - Set current position as zero
  - `~/emergency_stop` - Stop all motors
  - `~/enable_torque` - Torque mode control
  - `~/set_free` - Free mode (manual hand control)
- TUI to ROS 2 communication:
  - `recorder_tui.py` publishes mode/enable commands
  - `studio/` widgets publish/subscribe via ROS 2 topics

## Message Contracts

**JointState (Outbound):**
- Topic: `/joint_states`
- Message type: `sensor_msgs/JointState`
- Fields: `header` (timestamp), `name[]` (joint names), `position[]`, `velocity[]`, `effort[]`
- Publish rate: Configurable via `driver_config.yaml` (default: 1000.0 Hz)

**Motor Status (Outbound):**
- Topic: `/motor_status`
- Message type: `std_msgs/Float64MultiArray`
- Layout: [temp0, voltage0, temp1, voltage1, ...] per motor

**Control Commands (Inbound):**
- `/joint_state_ctrl` - Position/velocity setpoints (JointState)
- `/joint_effort_ctrl` - Torque commands (Float64MultiArray)
- `~/set_mode` - Mode string: "position", "velocity", "torque", "admittance", "free", "disabled"
- `~/set_enabled` - Boolean enable/disable

**Trigger Configuration (Inbound):**
- Topic: `~/motor_trigger_config`
- Message type: JSON string payload
- Usage: Hybrid playback with torque triggers (from `recorder_tui.py`)

---

*Integration audit: 2026-02-09*
