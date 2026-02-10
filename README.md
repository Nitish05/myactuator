# MyActuator RMD ROS 2 Workspace

A complete ROS 2 workspace for controlling MyActuator RMD X-series brushless motors over CAN bus. This workspace provides a C++ SDK with Python bindings, a full-featured ROS 2 driver, a graphical Motor Studio application, and a torque threshold calibration tool.

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
- [Desktop App Installation](#desktop-app-installation)
- [Recording and Playback](#recording-and-playback)
- [Torque Threshold Calibrator](#torque-threshold-calibrator)
- [Control Modes](#control-modes)
- [Advanced Features](#advanced-features)
- [C++ SDK Usage](#c-sdk-usage)
- [Python Bindings](#python-bindings)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Features

- **C++17 SDK** - Low-level motor control library with V4.3 protocol support
- **Python Bindings** - pybind11 bindings for Python integration
- **ROS 2 Driver** - Full-featured driver node with multiple control modes
- **Motor Studio** - PyQt6 GUI with easy mode and advanced mode for monitoring, recording, and playback
- **Torque Calibrator** - Headless CLI and GUI for automated torque threshold discovery
- **Setup Wizard** - Interactive TUI for motor discovery and configuration
- **Multi-motor Support** - Control up to 32 motors on a single CAN bus
- **Recording/Playback** - Record trajectories as ROS 2 bags and play them back with triggers
- **Admittance Control** - Move motors by hand with force feedback
- **Hybrid Playback** - Position control with hysteresis torque triggers
- **Motion Mode Control** - MIT-style impedance controller (V4.3)

## Supported Hardware

### Motors

**V4 X-Series (2024+):**
- RMD-X4-P12.5, RMD-X4-P36
- RMD-X6-P20
- RMD-X8-P20, RMD-X8-P33
- RMD-X12-P20
- RMD-X15-P20

**Legacy models:**
- X4 series (V2, V3, X4-3, X4-24)
- X6 series (V2, V3, S2V2, X6-7, X6-8, X6-40)
- X8 series (V2, V3, Pro, S2V3, HV3, X8-20, X8-25, X8-60, X8-90)
- X10 series (V3, S2V3, X10-40, X10-100)
- X12-150
- X15-400

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

Low-level C++17 library for direct motor communication. Implements the RMD V4.3 CAN protocol.

```
myactuator_rmd/
├── include/myactuator_rmd/
│   ├── actuator_interface.hpp    # Main API class
│   ├── actuator_constants.hpp    # Motor specs (all models including V4)
│   ├── driver/                   # CAN driver abstractions
│   ├── protocol/
│   │   ├── command_type.hpp      # All V4.3 command codes
│   │   ├── requests.hpp          # CAN request message encoding
│   │   ├── responses.hpp         # CAN response message decoding
│   │   └── motion_mode.hpp       # MIT-style impedance controller
│   ├── can/                      # SocketCAN implementation
│   └── actuator_state/
│       ├── gains.hpp             # Float-based PID gains (V4.3)
│       ├── gain_index.hpp        # PID parameter indices
│       ├── motor_status_1.hpp    # Temperature, MOS temp, voltage, errors
│       ├── error_code.hpp        # All V4.3 error flags
│       └── ...
├── src/                          # Implementation
├── bindings/                     # pybind11 Python bindings
└── test/                         # Unit tests (GTest)
```

### myactuator_python_driver (ROS 2 Node)

Python ROS 2 driver with TUI tools, Motor Studio, and calibration tools.

```
myactuator_python_driver/
├── myactuator_python_driver/
│   ├── driver_node.py       # Main ROS 2 driver node
│   ├── motor_wrapper.py     # Python wrapper for C++ bindings
│   ├── config.py            # Configuration dataclasses + torque constants
│   ├── can_utils.py         # CAN scanning utilities
│   ├── setup_tui.py         # Interactive setup wizard
│   ├── recorder_tui.py      # Terminal recording interface
│   ├── velocity_demo.py     # Simple velocity demo node
│   ├── studio/              # PyQt6 Motor Studio application
│   │   ├── main_window.py   # Main window (easy mode + advanced mode)
│   │   ├── ros_bridge.py    # ROS 2 <-> Qt signal bridge
│   │   ├── recording_manager.py  # ROS 2 bag recording/playback
│   │   └── widgets/         # UI components (control, monitor, record, playback, browse, easy mode)
│   └── calibrator/          # Torque threshold calibration
│       ├── controller.py    # CalibrationController state machine
│       ├── config.py        # CalibrationConfig, CalibrationResult
│       ├── window.py        # CalibrationWindow (PyQt6 GUI)
│       └── main.py          # CLI + GUI entry points
├── config/
│   └── driver_config.yaml   # Default configuration
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

| Motor | Torque Constant (Nm/A) |
|-------|------------------------|
| X4 (V2/V3/X4-3) | 0.11 |
| X4-24 | 0.11 |
| X6 (V2/V3/X6-7/X6-8/X6-40) | 0.16 |
| X8 (all legacy models) | 0.21 |
| X10 (V3/S2V3/X10-40/X10-100) | 0.32 |
| X12-150 | 0.42 |
| X15-400 | 0.65 |
| **X4V4-P12.5** | **1.0** |
| **X4V4-P36** | **2.57** |
| **X6V4-P20** | **2.0** |
| **X8V4-P20** | **2.86** |
| **X8V4-P33** | **6.6** |
| **X12V4-P20** | **4.0** |
| **X15V4-P20** | **6.0** |

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
| `/motor_status` | `std_msgs/Float64MultiArray` | Temperature, voltage per motor |
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

Motor Studio is a PyQt6 desktop application for motor control, monitoring, and trajectory recording. It starts in easy mode by default and includes an advanced mode for full control.

### Launch

```bash
# Standalone (requires driver running separately)
ros2 run myactuator_python_driver motor_studio

# With driver (recommended)
ros2 launch myactuator_python_driver driver_with_studio.launch.py
```

### Easy Mode

The default view with a simplified interface:
- Connection status LED
- Recording dropdown selector
- Play/Pause/Stop controls
- Emergency stop button
- "Advanced" button to switch to full interface

### Advanced Mode

Full-featured interface with docks and tabs:

#### Control Panel (Left Dock)
- Mode selection (Position, Velocity, Torque, Free, Admittance)
- Enable/Disable toggle
- Emergency Stop button
- Set Zero / Go to Zero

#### Monitor Tab
- Real-time joint state graphs (position, velocity, effort)
- Configurable time window
- Per-joint enable/disable

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

## Desktop App Installation

Motor Studio can be installed as a standalone desktop application that you launch by double-clicking an icon. It automatically connects CAN, starts the driver, and opens the GUI.

### How It Works

The launcher script (`scripts/motor-studio.sh`) handles everything:

1. **CAN setup** -- auto-detects native socketcan (candleLight/gs_usb) or SLCAN adapters and brings up `can0`
2. **ROS 2 sourcing** -- auto-detects the installed ROS 2 distro (Humble, Jazzy, etc.)
3. **Driver launch** -- starts the motor driver node in the background
4. **Studio launch** -- opens the PyQt6 GUI

If no CAN adapter is connected, the app still opens -- it just won't have hardware access until you plug one in and restart.

### Supported Platforms

| Platform | ROS 2 | Setup |
|----------|-------|-------|
| Desktop (Arch/Ubuntu) via distrobox | Humble | Container -- desktop entry wraps with `distrobox-enter` |
| Raspberry Pi 5 | Jazzy | Native -- desktop entry runs the launcher directly |
| Ubuntu 22.04/24.04 native | Humble/Jazzy | Native -- same as RPi5 |

### Prerequisites

1. **Build the workspace** (must be done before installing):
   ```bash
   cd ~/work/myactuator   # or wherever your workspace is
   colcon build --cmake-args -DPYTHON_BINDINGS=on
   ```

2. **CAN utilities** (for SLCAN adapters):
   ```bash
   sudo apt install can-utils
   ```

3. **PyQt6** (for the GUI):
   ```bash
   pip install PyQt6
   ```

### Install (One-Time)

Run from inside the workspace (and inside the distrobox container if using one):

```bash
./scripts/install-desktop.sh
```

This does three things:

1. **Creates a sudoers rule** (`/etc/sudoers.d/motor-studio-can`) so CAN setup runs without a password prompt
2. **Adds a desktop shortcut** to `~/Desktop/motor-studio.desktop`
3. **Adds an app menu entry** to `~/.local/share/applications/`

On a distrobox setup, the desktop entry automatically wraps the launch command with `distrobox-enter <container> --` so everything runs inside the container.

### Usage

After installation:

- **Double-click** "Motor Studio" on your desktop
- **Search** "Motor Studio" in your app menu
- **Terminal** (inside container/native):
  ```bash
  ./scripts/motor-studio.sh
  ```

### Configuration

The launcher reads environment variables for customization:

| Variable | Default | Description |
|----------|---------|-------------|
| `MOTOR_STUDIO_WORKSPACE` | Parent of `scripts/` dir | Path to the colcon workspace |
| `MOTOR_STUDIO_CAN` | `can0` | CAN interface name |
| `MOTOR_STUDIO_BITRATE` | `1000000` | CAN bitrate in bps |

Example:
```bash
MOTOR_STUDIO_CAN=can1 MOTOR_STUDIO_BITRATE=500000 ./scripts/motor-studio.sh
```

### Logs

Logs are stored in `~/.local/share/motor-studio/logs/` (last 10 kept). Check the latest log if the app fails to start:

```bash
cat ~/.local/share/motor-studio/logs/$(ls -t ~/.local/share/motor-studio/logs/ | head -1)
```

### Raspberry Pi 5 Notes

- Clone and build the workspace on the Pi (uses ROS 2 Jazzy natively)
- Run `./scripts/install-desktop.sh` -- it detects the native environment and sets up accordingly
- The first launch after install requires no password (sudoers is configured by the installer)
- To auto-start on boot (kiosk mode):
  ```bash
  mkdir -p ~/.config/autostart
  cp ~/.local/share/applications/motor-studio.desktop ~/.config/autostart/
  ```

### Reinstalling

Only re-run `./scripts/install-desktop.sh` if you:
- Move the workspace to a different directory
- Switch to a different distrobox container

Normal code changes only need `colcon build` -- the desktop shortcut points to the scripts in your workspace, which source the latest build on every launch.

## Recording and Playback

### Recording Trajectories

Recordings are stored as ROS 2 bags (sqlite3 + MCAP) in `~/recordings/`.

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
   - Load recording in Playback tab (or select from easy mode dropdown)
   - Click Play
   - Motors follow recorded trajectory

2. **Looped Playback:**
   - Enable "Loop" checkbox
   - Recording repeats until stopped

3. **Hybrid Playback with Triggers:**
   Configure hysteresis torque triggers for specific joint angles - useful for tasks like:
   - Grasping objects at certain positions
   - Applying force at end of motion
   - Compliance during contact

## Torque Threshold Calibrator

Automated tool for discovering torque thresholds -- applies torque to a joint, records the motion, detects reversal, and reports the threshold position.

### Headless CLI

```bash
ros2 run myactuator_python_driver calibrator_cli \
    --joint base_joint --torque 0.5 [--name my_calib] [--offset 0.5] [--settle-time 3.0]
```

Options:
- `--joint` (required): Joint to calibrate
- `--torque` (required): Torque in Nm (negative reverses direction)
- `--name`: Recording name (auto-generated if omitted)
- `--offset`: Threshold offset in degrees (default: 0.5)
- `--settle-time`: Seconds to track threshold after reversal (default: 3.0)

Press Ctrl+C once to stop gracefully, twice to emergency stop.

### GUI

```bash
ros2 run myactuator_python_driver calibrator_app
```

The GUI provides:
- Joint selection dropdown
- Torque and offset configuration
- Live position display during calibration
- Record/Stop/Emergency Stop controls
- Calibration result display (threshold position, extreme position, duration)

### How Calibration Works

1. Applies configured torque to the selected joint (zero effort to all others)
2. Records the entire sequence as a ROS 2 bag
3. Tracks the maximum position reached (extreme)
4. Detects reversal (position retreats by >1 degree from extreme)
5. Enters threshold tracking phase for the settle time
6. Locks and reports the threshold position

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

Configure position-based torque triggers for hybrid control. Triggers are persisted to `~/recordings/triggers.json`.

```python
# Trigger structure
{
    "joint_name": "gripper_joint",
    "enter_threshold_rad": 1.5,   # Switch to torque when angle > 1.5
    "exit_threshold_rad": 1.3,    # Switch back when angle < 1.3
    "torque_nm": 5.0,             # Apply 5 Nm in torque mode
    "direction": "rising",        # or "falling"
    "enabled": true,
    "recording_name": "pick_and_place"  # Paired with a specific recording
}
```

### Force-Controlled Position (V4.3)

Position control with torque limiting -- the motor moves to a target position but caps the applied torque:

```cpp
// C++ SDK
motor.sendForcePositionSetpoint(90.0, 500.0, 128);  // 90 deg, 500 dps, 50% torque limit
```

```python
# Python bindings
motor.sendForcePositionSetpoint(90.0, 500.0, 128)
```

### Motion Mode Control (V4.3)

MIT-style impedance controller using CAN ID `0x400+motor_id`:

```
torque = kp * (p_des - p) + kd * (v_des - v) + torque_ff
```

Parameters are bit-packed in big-endian format with configurable scaling limits.

### Multi-turn Position Tracking

The driver tracks multi-turn position, allowing:
- Continuous rotation beyond 360 degrees
- Absolute position commands
- Position offset calibration

### Communication Timeout

Set a watchdog timeout to stop motors if communication is lost:

```yaml
timeout_ms: 1000  # Stop if no command for 1 second
```

### Velocity Demo

A simple ROS 2 node that spins a joint at constant velocity:

```bash
ros2 run myactuator_python_driver velocity_demo
ros2 run myactuator_python_driver velocity_demo --ros-args -p speed:=180.0 -p joint:=base_joint
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
    // status.temperature, status.mos_temperature, status.voltage, status.error_code

    // Position control
    auto feedback = motor.sendPositionAbsoluteSetpoint(90.0, 500.0);

    // Force-controlled position (V4.3) - position with torque limit
    feedback = motor.sendForcePositionSetpoint(90.0, 500.0, 128);

    // Single-turn position (0-359.99 deg with direction)
    feedback = motor.sendSingleTurnPositionSetpoint(180.0, 500.0, 0x00);  // clockwise

    // Incremental (relative) position
    feedback = motor.sendIncrementalPositionSetpoint(45.0, 500.0);

    // Velocity control (with optional torque limit)
    feedback = motor.sendVelocitySetpoint(100.0);       // no torque limit
    feedback = motor.sendVelocitySetpoint(100.0, 128);   // 50% torque limit

    // Torque control
    float torque_constant = 0.21;  // X8 series
    feedback = motor.sendTorqueSetpoint(1.0, torque_constant);

    // PID gains (V4.3 float-based)
    auto gains = motor.getControllerGains();
    float kp = motor.getGain(myactuator_rmd::GainIndex::SPEED_KP);
    motor.setGain(myactuator_rmd::GainIndex::SPEED_KP, 50.0f, true);  // persistent

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
status = motor.getMotorStatus1()
print(f"Temperature: {status.temperature} C")
print(f"MOS Temperature: {status.mos_temperature} C")
print(f"Voltage: {status.voltage} V")

status2 = motor.getMotorStatus2()
print(f"Position: {status2.shaft_angle} deg")
print(f"Velocity: {status2.shaft_speed} dps")

# Control commands
feedback = motor.sendPositionAbsoluteSetpoint(90.0, 500.0)
feedback = motor.sendForcePositionSetpoint(90.0, 500.0, 128)
feedback = motor.sendVelocitySetpoint(100.0)
feedback = motor.sendTorqueSetpoint(1.0, 0.21)

# PID gains (V4.3)
gains = motor.getControllerGains()
motor.setGain(rmd.actuator_state.GainIndex.SPEED_KP, 50.0, True)

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

The SDK implements the MyActuator RMD V4.3 CAN protocol.

### CAN Addressing
- **TX (commands)**: `0x140 + motor_id`
- **RX (responses)**: `0x240 + motor_id`
- **Motion Mode TX**: `0x400 + motor_id`
- **Motion Mode RX**: `0x500 + motor_id`
- **Motor IDs**: 1-32

### Command Codes

| Code | Description |
|------|-------------|
| `0x20` | Function control |
| `0x30` | Read PID parameters (index-based) |
| `0x31` | Write PID to RAM (index-based) |
| `0x32` | Write PID to ROM (index-based) |
| `0x42` | Read acceleration |
| `0x43` | Write acceleration |
| `0x60-0x64` | Encoder operations |
| `0x70` | Read system operating mode |
| `0x76` | Reset system |
| `0x77` | Release brake |
| `0x78` | Lock brake |
| `0x79` | CAN ID setting |
| `0x80` | Shutdown motor |
| `0x81` | Stop motor |
| `0x90` | Read single-turn encoder |
| `0x92` | Read multi-turn angle |
| `0x94` | Read single-turn angle |
| `0x9A` | Read status 1 (temp, MOS temp, voltage, errors) |
| `0x9C` | Read status 2 (position, velocity, current) |
| `0x9D` | Read status 3 (phase currents) |
| `0xA1` | Torque/current control |
| `0xA2` | Velocity control (with optional torque limit) |
| `0xA4` | Absolute position control |
| `0xA6` | Single-turn position control |
| `0xA8` | Incremental position control |
| `0xA9` | Force-controlled position (with torque limit) |
| `0xB1` | Read system runtime |
| `0xB2` | Read software version |
| `0xB3` | Communication timeout setting |
| `0xB4` | Baud rate setting |
| `0xB5` | Read motor model |
| `0xB6` | Active reply function |
| `0x400+ID` | Motion mode control (MIT-style) |

## License

This project is licensed under the MIT License.

## Acknowledgments

- Original C++ SDK by [Tobit Flatscher](https://github.com/2b-t)
- MyActuator for the RMD motor series documentation
