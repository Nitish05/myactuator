# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a ROS 2 workspace for controlling MyActuator RMD X-series motors over CAN bus. It contains three packages:

- **myactuator_rmd** - C++17 SDK with Python bindings (pybind11) for low-level motor control via Linux SocketCAN
- **myactuator_python_driver** - Pure Python ROS 2 driver node with TUI setup wizard and recording/playback
- **TandP_URDF_description** - URDF robot description for visualization and simulation

## Build Commands

### Build entire workspace (ROS 2)
```bash
colcon build --cmake-args -DPYTHON_BINDINGS=on
source install/setup.bash
```

### Build specific package
```bash
colcon build --packages-select myactuator_rmd --cmake-args -DPYTHON_BINDINGS=on
colcon build --packages-select myactuator_python_driver
```

### Build C++ library standalone (without ROS 2)
```bash
cd myactuator_rmd
mkdir build && cd build
cmake .. -DPYTHON_BINDINGS=on
make -j$(nproc)
```

### Build and run tests
```bash
cd myactuator_rmd/build
cmake .. -DBUILD_TESTING=on
make -j$(nproc)
ctest
```

Tests require a virtual CAN interface:
```bash
sudo modprobe vcan
sudo ip link add dev vcan_test type vcan
sudo ip link set up vcan_test
```

## CAN Interface Setup

Use the interactive script:
```bash
./connect_can.sh
```

Or manual setup for candleLight firmware:
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

For SLCAN firmware:
```bash
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set up can0
```

Verify with: `candump can0`

## Architecture

### myactuator_rmd (C++ SDK)

```
myactuator_rmd/
├── include/myactuator_rmd/
│   ├── actuator_interface.hpp    # Main API: ActuatorInterface class
│   ├── actuator_constants.hpp    # Motor specs (torque constants, etc.)
│   ├── driver/                   # CanDriver, CanNode abstractions
│   ├── protocol/                 # CAN message encoding/decoding
│   ├── can/                      # SocketCAN node, frame handling
│   └── actuator_state/           # Feedback structs (MotorStatus1/2/3, Gains, etc.)
├── src/                          # Implementation files
├── bindings/                     # pybind11 Python bindings
└── test/                         # GTest unit tests with mock actuator
```

Key classes:
- `CanDriver` - Creates SocketCAN connection (pass interface name like "can0")
- `ActuatorInterface` - Per-motor interface with all control methods
- Motor constants in `actuator_constants.hpp` provide torque_constant for each motor model

CAN addressing: Commands TX to `0x140 + motor_id`, responses RX from `0x240 + motor_id`

### myactuator_python_driver (ROS 2 Node)

```
myactuator_python_driver/
├── myactuator_python_driver/
│   ├── driver_node.py      # MotorDriverNode: main ROS 2 driver
│   ├── motor_wrapper.py    # MotorWrapper: wraps C++ bindings
│   ├── config.py           # DriverConfig, MotorConfig dataclasses
│   ├── setup_tui.py        # Interactive terminal setup wizard
│   └── recorder_tui.py     # Recording/playback TUI
└── launch/
    └── driver.launch.py
```

ROS 2 topics:
- Publishes: `/joint_states`, `/motor_status`, `~/mode`
- Subscribes: `/joint_state_ctrl`, `/joint_effort_ctrl`, `~/set_mode`, `~/set_enabled`

Control modes: position, velocity, torque, free (move by hand), disabled

### Python Import Paths

When built with ROS 2:
```python
from myactuator_rmd import myactuator_rmd_py as rmd
```

When installed standalone:
```python
import myactuator_rmd_py as rmd
```

## Running the Driver

```bash
# Interactive setup (creates config file)
ros2 run myactuator_python_driver setup_tui

# Start driver
ros2 launch myactuator_python_driver driver.launch.py

# Recording/playback interface
ros2 run myactuator_python_driver recorder_tui
```

## Protocol Reference

Motor commands follow RMD-X Servo Motor Control Protocol V3.8. Common command codes:
- `0xA4` - Position control (absolute, closed-loop)
- `0xA2` - Velocity control
- `0xA1` - Torque/current control
- `0x9C` - Read motor status
- `0x80` - Shutdown motor
- `0x81` - Stop motor

Position units: 0.01 degrees. Speed units: 1 dps. Current resolution varies by command.
