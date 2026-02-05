# MyActuator RMD ROS 2 Workspace

A complete ROS 2 workspace for controlling MyActuator RMD X-series brushless motors over CAN bus. This workspace provides a C++ SDK with Python bindings, a full-featured ROS 2 driver, and a graphical Motor Studio application for recording and playing back trajectories.

## Table of Contents

- [Features](#features)
- [Supported Hardware](#supported-hardware)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [CAN Interface Setup](#can-interface-setup)
- [Quick Start](#quick-start)
- [Package Overview](#package-overview)
- [Driver Configuration](#driver-configuration)
- [ROS 2 Interface](#ros-2-interface)
- [Motor Studio GUI](#motor-studio-gui)
- [Recording and Playback](#recording-and-playback)
- [Control Modes](#control-modes)
- [Advanced Features](#advanced-features)
- [C++ SDK Usage](#c-sdk-usage)
- [Python Bindings](#python-bindings)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Features

- **C++17 SDK** - Low-level motor control library with full protocol support
- **Python Bindings** - pybind11 bindings for Python integration
- **ROS 2 Driver** - Full-featured driver node with multiple control modes
- **Motor Studio** - PyQt6 GUI for monitoring, recording, and playback
- **Setup Wizard** - Interactive TUI for motor discovery and configuration
- **Multi-motor Support** - Control up to 32 motors on a single CAN bus
- **Recording/Playback** - Record trajectories and play them back with triggers
- **Admittance Control** - Move motors by hand with force feedback
- **Hybrid Playback** - Position control with torque override triggers

## Supported Hardware

### Motors
- MyActuator RMD-X4 series (V2, V3, X4-3, X4-24)
- MyActuator RMD-X6 series (V2, V3, S2V2, X6-7, X6-8, X6-40)
- MyActuator RMD-X8 series (V2, V3, Pro, S2V3, HV3, X8-20, X8-25, X8-60, X8-90)
- MyActuator RMD-X10 series (V3, S2V3, X10-40, X10-100)
- MyActuator RMD-X12-150
- MyActuator RMD-X15-400

### CAN Adapters
- CANable / CANable Pro (candleLight firmware)
- PEAK PCAN-USB
- Any SocketCAN-compatible adapter
- SLCAN adapters (serial)

## System Requirements

- **OS**: Ubuntu 22.04 or 24.04 (Linux only - requires SocketCAN)
- **ROS 2**: Humble or Jazzy
- **Python**: 3.10+
- **CMake**: 3.20+
- **Compiler**: GCC with C++17 support

### Dependencies

```bash
# ROS 2 dependencies
sudo apt install ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-std-srvs

# Build dependencies
sudo apt install python3-pybind11 libpython3-dev

# CAN utilities
sudo apt install can-utils

# Motor Studio (PyQt6)
pip install PyQt6 qt-material rich
```

## Installation

### 1. Clone the Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-repo/myactuator.git
```

### 2. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --cmake-args -DPYTHON_BINDINGS=on
source install/setup.bash
```

### 3. Build Specific Packages (Optional)

```bash
# C++ SDK only
colcon build --packages-select myactuator_rmd --cmake-args -DPYTHON_BINDINGS=on

# Python driver only
colcon build --packages-select myactuator_python_driver
```

## CAN Interface Setup

### Using the Interactive Script

```bash
./connect_can.sh
```

### Manual Setup

#### For candleLight Firmware (CANable, etc.)

```bash
# Set bitrate to 1 Mbps (standard for RMD motors)
sudo ip link set can0 type can bitrate 1000000

# Bring up the interface
sudo ip link set up can0
```

#### For SLCAN Adapters

```bash
# Create SLCAN device (s8 = 1 Mbps)
sudo slcand -o -c -s8 /dev/ttyACM0 can0

# Bring up the interface
sudo ip link set up can0
```

#### Verify Connection

```bash
# Check interface status
ip link show can0

# Monitor CAN traffic (should see motor heartbeats)
candump can0
```

### CAN Bus Wiring

```
CAN_H ──────┬──────┬──────┬────── CAN_H
            │      │      │
         [Motor1] [Motor2] [Motor3]
            │      │      │
CAN_L ──────┴──────┴──────┴────── CAN_L

Note: Use 120Ω termination resistors at both ends of the bus
```

## Quick Start

### 1. Configure Motors (First Time)

Run the interactive setup wizard to detect motors and create a configuration file:

```bash
ros2 run myactuator_python_driver setup_tui
```

This will:
- Scan for available CAN interfaces
- Discover connected motors
- Let you name joints and configure parameters
- Save configuration to `~/.config/myactuator_python_driver/driver_config.yaml`

### 2. Launch the Driver

```bash
ros2 launch myactuator_python_driver driver.launch.py
```

### 3. Test Motor Control

```bash
# Check motor state
ros2 topic echo /joint_states

# Enable motors and set to position mode
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'position'"

# Move motor to 90 degrees (1.57 rad)
ros2 topic pub --once /joint_state_ctrl sensor_msgs/msg/JointState \
  "{name: ['base_joint'], position: [1.57]}"
```

### 4. Launch with Motor Studio

```bash
ros2 launch myactuator_python_driver driver_with_studio.launch.py
```

## Package Overview

### myactuator_rmd (C++ SDK)

Low-level C++17 library for direct motor communication.

```
myactuator_rmd/
├── include/myactuator_rmd/
│   ├── actuator_interface.hpp    # Main API class
│   ├── actuator_constants.hpp    # Motor torque constants
│   ├── driver/                   # CAN driver abstractions
│   ├── protocol/                 # CAN message encoding/decoding
│   ├── can/                      # SocketCAN implementation
│   └── actuator_state/           # Feedback data structures
├── src/                          # Implementation
├── bindings/                     # pybind11 Python bindings
└── test/                         # Unit tests (GTest)
```

### myactuator_python_driver (ROS 2 Node)

Python ROS 2 driver with TUI tools and Motor Studio.

```
myactuator_python_driver/
├── myactuator_python_driver/
│   ├── driver_node.py      # Main ROS 2 driver node
│   ├── motor_wrapper.py    # Python wrapper for C++ bindings
│   ├── config.py           # Configuration dataclasses
│   ├── can_utils.py        # CAN scanning utilities
│   ├── setup_tui.py        # Interactive setup wizard
│   ├── recorder_tui.py     # Terminal recording interface
│   └── studio/             # PyQt6 Motor Studio application
├── config/
│   └── driver_config.yaml  # Default configuration
└── launch/
    ├── driver.launch.py              # Driver only
    └── driver_with_studio.launch.py  # Driver + Motor Studio
```

### TandP_URDF_description

URDF robot description for visualization.

```bash
# Launch RViz visualization
ros2 launch TandP_URDF_description display.launch.py
```

## Driver Configuration

### Configuration File Format

```yaml
# driver_config.yaml
can_interface: can0
publish_rate: 500.0        # Hz - control loop frequency
control_mode: position     # Default mode: position, velocity, torque
timeout_ms: 0              # CAN timeout (0 = disabled)

motors:
  - can_id: 1
    joint_name: base_joint
    torque_constant: 2.1   # Nm/A - depends on motor model
    max_velocity: 720.0    # deg/s
    position_offset: 0.0   # deg - zero offset
    inverted: false        # Invert direction

  - can_id: 2
    joint_name: elbow_joint
    torque_constant: 0.8
    max_velocity: 1440.0
    position_offset: 0.0
    inverted: false
```

### Torque Constants by Motor Model

| Motor Series | Torque Constant (Nm/A) |
|--------------|------------------------|
| X4           | 0.11                   |
| X6           | 0.16                   |
| X8           | 0.21                   |
| X10          | 0.32                   |
| X12          | 0.42                   |
| X15          | 0.65                   |

### Launch Parameters

```bash
ros2 launch myactuator_python_driver driver.launch.py \
    config_file:=/path/to/config.yaml \
    can_interface:=can0 \
    publish_rate:=500.0 \
    control_mode:=position \
    motor_ids:='[1,2,3]' \
    joint_names:='[base,elbow,wrist]' \
    torque_constants:='[2.1,0.8,0.8]'
```

## ROS 2 Interface

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Position (rad), velocity (rad/s), effort (Nm) |
| `/motor_status` | `std_msgs/Float64MultiArray` | Temperature (°C), voltage (V) per motor |
| `~/mode` | `std_msgs/String` | Current control mode |
| `~/trigger_states` | `std_msgs/String` | Active trigger states (JSON) |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_state_ctrl` | `sensor_msgs/JointState` | Position/velocity commands |
| `/joint_effort_ctrl` | `std_msgs/Float64MultiArray` | Torque commands (Nm) |
| `~/set_mode` | `std_msgs/String` | Set control mode |
| `~/set_enabled` | `std_msgs/Bool` | Enable/disable motors |
| `~/playback_triggers` | `std_msgs/String` | Trigger configuration (JSON) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `~/set_zero` | `std_srvs/Trigger` | Set current position as zero |
| `~/emergency_stop` | `std_srvs/Trigger` | Emergency stop all motors |
| `~/enable_torque` | `std_srvs/SetBool` | Enable/disable torque mode |
| `~/set_free` | `std_srvs/SetBool` | Enable/disable free mode |

### Command Examples

```bash
# Set control mode
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'position'"
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'velocity'"
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'torque'"
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'free'"
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'admittance'"

# Enable/disable motors
ros2 topic pub --once /motor_driver/set_enabled std_msgs/msg/Bool "data: true"
ros2 topic pub --once /motor_driver/set_enabled std_msgs/msg/Bool "data: false"

# Position command (radians)
ros2 topic pub --once /joint_state_ctrl sensor_msgs/msg/JointState \
  "{name: ['base_joint', 'elbow_joint'], position: [1.57, 0.78]}"

# Velocity command (rad/s)
ros2 topic pub --once /joint_state_ctrl sensor_msgs/msg/JointState \
  "{name: ['base_joint'], velocity: [1.0]}"

# Torque command (Nm)
ros2 topic pub --once /joint_effort_ctrl std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.3]}"

# Emergency stop
ros2 service call /motor_driver/emergency_stop std_srvs/srv/Trigger

# Set current position as zero
ros2 service call /motor_driver/set_zero std_srvs/srv/Trigger
```

## Motor Studio GUI

Motor Studio is a PyQt6 desktop application for motor control, monitoring, and trajectory recording.

### Launch

```bash
# Standalone (requires driver running separately)
ros2 run myactuator_python_driver motor_studio

# With driver (recommended)
ros2 launch myactuator_python_driver driver_with_studio.launch.py
```

### Features

#### Monitor Tab
- Real-time joint state graphs (position, velocity, effort)
- Configurable time window
- Per-joint enable/disable

#### Control Panel
- Mode selection buttons (Position, Velocity, Torque, Free, Admittance)
- Enable/Disable toggle
- Emergency Stop button
- Set Zero / Go to Zero

#### Record Tab
- Name recordings
- Start/Stop recording
- Frame counter
- Auto-saves to `~/recordings/`

#### Playback Tab
- Select recording from list
- Play/Pause/Stop controls
- Progress bar with time display
- Loop playback option
- Playback speed adjustment
- Trigger configuration for hybrid playback

#### Browse Tab
- List all recordings
- Rename/Delete recordings
- Double-click to load for playback

### Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Escape` | Emergency Stop |
| `Ctrl+Z` | Set Zero |
| `Ctrl+0` | Go to Zero |
| `F5` | Refresh Recordings |

## Recording and Playback

### Recording Trajectories

1. **Using Motor Studio:**
   - Switch to "Free" mode
   - Go to Record tab
   - Enter recording name
   - Click "Start Recording"
   - Move motors by hand
   - Click "Stop Recording"

2. **Using Terminal TUI:**
   ```bash
   ros2 run myactuator_python_driver recorder_tui
   ```

### Playback

1. **Basic Playback:**
   - Load recording in Playback tab
   - Click Play
   - Motors follow recorded trajectory

2. **Looped Playback:**
   - Enable "Loop" checkbox
   - Recording repeats until stopped

3. **Hybrid Playback with Triggers:**
   Configure torque triggers for specific joint angles - useful for tasks like:
   - Grasping objects at certain positions
   - Applying force at end of motion
   - Compliance during contact

### Recording File Format

Recordings are stored as JSON files in `~/recordings/`:

```json
{
  "name": "pick_and_place",
  "created": "2024-01-15T10:30:00",
  "joint_names": ["base_joint", "elbow_joint"],
  "frames": [
    {
      "timestamp_ns": 1000000000,
      "position": [0.0, 0.0],
      "velocity": [0.0, 0.0],
      "effort": [0.0, 0.0]
    },
    ...
  ]
}
```

## Control Modes

### Position Mode
Standard position control with trajectory following.
```bash
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'position'"
```

### Velocity Mode
Direct velocity control.
```bash
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'velocity'"
```

### Torque Mode
Direct torque/current control.
```bash
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'torque'"
```

### Free Mode
Motors can be moved by hand (no holding torque). Useful for:
- Teaching by demonstration
- Recording trajectories
- Manual positioning

```bash
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'free'"
```

### Admittance Mode
Motors assist movement when pushed. Parameters:
- **Gain**: How fast motors move per unit of applied force
- **Deadband**: Minimum force to trigger movement
- **Max velocity**: Speed limit

```bash
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'admittance'"
```

### Disabled Mode
Motors are disabled (no commands sent, only reading state).
```bash
ros2 topic pub --once /motor_driver/set_mode std_msgs/msg/String "data: 'disabled'"
```

## Advanced Features

### Hysteresis Torque Triggers

Configure position-based torque triggers for hybrid control:

```python
# Trigger structure
{
    "joint_name": "gripper_joint",
    "enter_threshold_rad": 1.5,   # Switch to torque when angle > 1.5
    "exit_threshold_rad": 1.3,    # Switch back when angle < 1.3
    "torque_nm": 5.0,             # Apply 5 Nm in torque mode
    "direction": "rising",        # or "falling"
    "enabled": true
}
```

### Multi-turn Position Tracking

The driver tracks multi-turn position, allowing:
- Continuous rotation beyond 360°
- Absolute position commands
- Position offset calibration

### Communication Timeout

Set a watchdog timeout to stop motors if communication is lost:

```yaml
timeout_ms: 1000  # Stop if no command for 1 second
```

## C++ SDK Usage

### Basic Example

```cpp
#include "myactuator_rmd/actuator_interface.hpp"
#include "myactuator_rmd/driver/can_driver.hpp"

int main() {
    // Create CAN driver
    myactuator_rmd::CanDriver driver("can0");

    // Create actuator interface (motor ID 1)
    myactuator_rmd::ActuatorInterface motor(driver, 1);

    // Get motor info
    std::string model = motor.getMotorModel();
    auto status = motor.getMotorStatus1();

    // Send position command (degrees, max speed in dps)
    auto feedback = motor.sendPositionAbsoluteSetpoint(90.0, 500.0);

    // Send velocity command (degrees per second)
    feedback = motor.sendVelocitySetpoint(100.0);

    // Send torque command (Nm, requires torque constant)
    float torque_constant = 0.21;  // X8 series
    feedback = motor.sendTorqueSetpoint(1.0, torque_constant);

    // Stop motor
    motor.stopMotor();

    return 0;
}
```

### Building Standalone (Without ROS 2)

```bash
cd myactuator_rmd
mkdir build && cd build
cmake .. -DPYTHON_BINDINGS=on
make -j$(nproc)
```

### Running Tests

```bash
# Setup virtual CAN interface
sudo modprobe vcan
sudo ip link add dev vcan_test type vcan
sudo ip link set up vcan_test

# Build and run tests
cd myactuator_rmd/build
cmake .. -DBUILD_TESTING=on
make -j$(nproc)
ctest
```

## Python Bindings

### Import

```python
# When using ROS 2
from myactuator_rmd import myactuator_rmd_py as rmd

# Standalone installation
import myactuator_rmd_py as rmd
```

### Example

```python
import myactuator_rmd_py as rmd

# Create driver and motor interface
driver = rmd.CanDriver("can0")
motor = rmd.ActuatorInterface(driver, 1)

# Get motor info
model = motor.getMotorModel()
status = motor.getMotorStatus2()
print(f"Position: {status.shaft_angle}°")
print(f"Velocity: {status.shaft_speed} dps")

# Control commands
feedback = motor.sendPositionAbsoluteSetpoint(90.0, 500.0)
feedback = motor.sendVelocitySetpoint(100.0)
feedback = motor.sendTorqueSetpoint(1.0, 0.21)

# Stop
motor.stopMotor()
```

## Troubleshooting

### CAN Interface Issues

**Problem**: `Cannot find device "can0"`
```bash
# Check if interface exists
ip link show can0

# For USB adapters, check connection
dmesg | tail -20
lsusb
```

**Problem**: `No buffer space available`
```bash
# Increase CAN buffer size
sudo ip link set can0 txqueuelen 1000
```

**Problem**: Motors not responding
```bash
# Check CAN traffic
candump can0

# Verify bitrate matches motor setting (usually 1 Mbps)
ip -details link show can0
```

### Motor Issues

**Problem**: Motor jumps to zero on startup
- The driver reads initial position on startup
- Ensure motors are powered before starting the driver

**Problem**: Jerky motion
- Increase `publish_rate` (try 500-1000 Hz)
- Check CAN bus for errors: `ip -s link show can0`

**Problem**: Wrong direction
- Set `inverted: true` in motor config

### Build Issues

**Problem**: `pybind11 not found`
```bash
sudo apt install python3-pybind11
# or
pip install pybind11
```

**Problem**: Python bindings not loading
```bash
# Ensure workspace is sourced
source install/setup.bash

# Check Python path
python3 -c "from myactuator_rmd import myactuator_rmd_py; print('OK')"
```

## Protocol Reference

The driver implements the RMD-X Servo Motor Control Protocol V3.8.

### CAN Addressing
- **TX (commands)**: `0x140 + motor_id`
- **RX (responses)**: `0x240 + motor_id`
- **Motor IDs**: 1-32

### Common Command Codes
| Code | Description |
|------|-------------|
| `0x80` | Shutdown motor |
| `0x81` | Stop motor |
| `0x88` | Reset motor |
| `0x9A` | Read status 1 (temp, voltage, errors) |
| `0x9C` | Read status 2 (position, velocity, current) |
| `0x9D` | Read status 3 (phase currents) |
| `0xA1` | Torque/current control |
| `0xA2` | Velocity control |
| `0xA4` | Position control (absolute) |

## License

This project is licensed under the MIT License.

## Acknowledgments

- Original C++ SDK by [Tobit Flatscher](https://github.com/2b-t)
- MyActuator for the RMD motor series documentation
