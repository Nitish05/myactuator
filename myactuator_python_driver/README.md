# MyActuator Python Driver

A pure Python ROS2 driver for MyActuator RMD motors with CAN bus communication, interactive TUI setup, and recording/playback functionality.

## Features

- **TUI Setup Wizard**: Interactive terminal UI for motor configuration
- **CAN Bus Scanning**: Automatic motor discovery on the CAN bus
- **Standard ROS2 Topics**: Publishes `/joint_states`, subscribes to control topics
- **Multiple Control Modes**: Position, velocity, and torque control
- **Recording & Playback**: Record motor movements and play them back
- **Conditional Torque Override**: Switch to torque mode when angle exceeds threshold

## Installation

```bash
# Build the package
cd ~/your_ros2_ws
colcon build --packages-select myactuator_python_driver

# Source the workspace
source install/setup.bash
```

## Quick Start

### 1. Run the Setup Wizard

```bash
ros2 run myactuator_python_driver setup_tui
```

This will guide you through:
- Selecting the CAN interface
- Scanning for motors
- Configuring joint names and parameters
- Setting up torque override rules

### 2. Start the Driver

```bash
# Using the generated config
ros2 launch myactuator_python_driver driver.launch.py

# Or with manual parameters
ros2 launch myactuator_python_driver driver.launch.py \
    can_interface:=can0 \
    motor_ids:="[1, 2, 3]" \
    joint_names:="[shoulder, elbow, wrist]"
```

### 3. Recording and Playback TUI

```bash
# Launch the recorder TUI
ros2 run myactuator_python_driver recorder_tui
```

**Keyboard Shortcuts:**

| Key | Action |
|-----|--------|
| `r` | Start/Stop recording |
| `p` | Start/Stop playback |
| `SPACE` | Pause/Resume playback |
| `[` / `]` | Decrease/Increase speed |
| `l` | Toggle loop |
| `z` | Set zero (current pos = 0) |
| `0` | Go to zero position |
| `m` | Cycle mode (position/velocity/torque/free) |
| `e` | Toggle enable/disable |
| `ESC` | Emergency stop |
| `↑` / `↓` | Select recording |
| `d` | Delete selected |
| `n` | Rename selected |
| `?` | Toggle help |
| `q` | Quit |

The TUI automatically:
- Enables **free mode** when recording starts (move motors by hand)
- Switches to **position mode** when recording stops or playback starts
- Stores recordings as ROS bags (mcap) in `~/motor_recordings/`

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Position (rad), velocity (rad/s), effort (Nm) |
| `/motor_status` | `std_msgs/Float64MultiArray` | Temperature, voltage per motor |
| `/motor_driver/mode` | `std_msgs/String` | Current control mode |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_state_ctrl` | `sensor_msgs/JointState` | Position/velocity commands |
| `/joint_effort_ctrl` | `std_msgs/Float64MultiArray` | Direct torque commands (Nm) |
| `/motor_driver/set_mode` | `std_msgs/String` | Set mode (position/velocity/torque/free/disabled) |
| `/motor_driver/set_enabled` | `std_msgs/Bool` | Enable/disable motors |

## Services

### Driver Node (`/motor_driver`)

| Service | Type | Description |
|---------|------|-------------|
| `~/set_zero` | `Trigger` | Set current position as encoder zero |
| `~/emergency_stop` | `Trigger` | Stop all motors immediately |
| `~/enable_torque` | `SetBool` | Enable/disable torque control mode |
| `~/set_free` | `SetBool` | Enable/disable free mode (move by hand) |

### Recorder Node (`/motor_recorder`)

| Service | Type | Description |
|---------|------|-------------|
| `~/start_recording` | `Trigger` | Start recording joint states |
| `~/stop_recording` | `Trigger` | Stop and save recording |
| `~/start_playback` | `Trigger` | Start playback of last recording |
| `~/stop_playback` | `Trigger` | Stop playback |
| `~/toggle_loop` | `SetBool` | Enable/disable loop playback |

## Configuration

### YAML Configuration File

```yaml
can_interface: can0
publish_rate: 100.0
control_mode: position  # position, velocity, torque, readonly

motors:
  - can_id: 1
    joint_name: joint1
    torque_constant: 0.21  # Nm/A
    max_velocity: 720.0    # deg/s
    inverted: false
    torque_rule:           # Optional: torque override
      threshold_deg: 45.0
      torque_nm: 2.0
      direction: above     # Switch to torque when angle > 45°
```

### Torque Constants by Motor Model

| Model Series | Torque Constant (Nm/A) |
|--------------|------------------------|
| X4 | 0.11 |
| X6 | 0.16 |
| X8 | 0.21 |
| X10 | 0.32 |
| X12 | 0.42 |
| X15 | 0.65 |

## Conditional Torque Override

During playback, you can configure joints to switch from position control to torque mode when exceeding an angle threshold:

```yaml
motors:
  - can_id: 1
    joint_name: elbow
    torque_rule:
      threshold_deg: 90.0   # Trigger at 90 degrees
      torque_nm: 3.0        # Apply 3 Nm torque
      direction: above      # When angle > 90°
```

This is useful for:
- Adding resistance at end-of-range
- Safety limits
- Compliance behavior

## CAN Interface Setup

```bash
# Bring up CAN interface (replace with your settings)
sudo ip link set can0 up type can bitrate 1000000

# Or use the provided script
./connect_can.sh
```

## Example: Manual Torque Control

```bash
# Enable torque mode
ros2 service call /motor_driver/enable_torque std_srvs/srv/SetBool "{data: true}"

# Send torque commands (Nm)
ros2 topic pub /joint_effort_ctrl std_msgs/msg/Float64MultiArray \
    "{data: [0.5, 0.0, 0.0]}"

# Disable torque mode (returns to position control)
ros2 service call /motor_driver/enable_torque std_srvs/srv/SetBool "{data: false}"
```

## Dependencies

- ROS2 Jazzy
- `myactuator_rmd` package (provides Python bindings)
- `sensor_msgs`, `std_msgs`, `std_srvs`
- `rosbag2` (for recording)
- `rich` (for TUI)

## License

MIT License
