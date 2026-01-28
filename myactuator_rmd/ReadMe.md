# MyActuator RMD CAN Driver SDK

A C++17 software development kit for controlling MyActuator RMD X-series motors over CAN bus using Linux SocketCAN. Includes Python bindings via pybind11.

Original Author: [Tobit Flatscher](https://github.com/2b-t) (2023-2024)

## Overview

This SDK provides:

- C++ library for MyActuator RMD motor control
- Python bindings for rapid prototyping
- Support for all RMD X-series commands
- Integration with ROS 2 via SocketCAN

## Supported Motors

- RMD X4, X6, X8, X10, X12 series
- Both planetary (RMD-X) and direct-drive variants

## Hardware Requirements

### CAN Adapter Options

| Option | Cost | Pros | Cons |
|--------|------|------|------|
| **CANable** | $15-50 | Open-source, well-supported | Requires firmware choice |
| **PCAN-USB** | $200+ | Professional, reliable | Expensive |
| **Embedded CAN** | Varies | Native support | Hardware-specific |

### Recommended: CANable Setup

See the main [README.md](../README.md#myactuator-motors-canableslcan) for detailed CANable and SLCAN setup instructions.

**Quick Setup (candleLight firmware):**

```bash
# Configure CAN interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Verify
candump can0
```

**Quick Setup (SLCAN firmware):**

```bash
# Create SLCAN interface from serial device
sudo slcand -o -c -s8 /dev/ttyACM0 slcan0
sudo ip link set up slcan0
```

## Installation

### System Dependencies

```bash
sudo apt install -y build-essential cmake can-utils iproute2
sudo apt install -y python3 python3-pip python3-pybind11  # For Python bindings
```

### Build as Standalone Library

```bash
cd myactuator_rmd
mkdir build && cd build
cmake .. -DPYTHON_BINDINGS=on
make -j$(nproc)
sudo make install
```

### Build with ROS 2

```bash
cd ~/TA_T-P_Satin_R
colcon build --packages-select myactuator_rmd --cmake-args -DPYTHON_BINDINGS=on
```

### Install Python Package Only

```bash
cd myactuator_rmd
pip3 install .
```

## Usage

### C++ Example

```cpp
#include <myactuator_rmd/myactuator_rmd.hpp>

int main() {
    // Create CAN driver (interface name)
    myactuator_rmd::CanDriver driver{"can0"};

    // Create actuator interface (driver, motor ID)
    myactuator_rmd::ActuatorInterface actuator{driver, 1};

    // Read motor information
    std::cout << actuator.getVersionDate() << std::endl;

    // Move to position (degrees, max speed in dps)
    actuator.sendPositionAbsoluteSetpoint(180.0, 500.0);

    // Shutdown motor
    actuator.shutdownMotor();

    return 0;
}
```

### CMake Integration

```cmake
find_package(myactuator_rmd REQUIRED)

add_executable(my_app src/main.cpp)
target_link_libraries(my_app myactuator_rmd::myactuator_rmd)
target_compile_features(my_app PUBLIC cxx_std_17)
```

### Python Example

```python
import myactuator_rmd_py as rmd

# Create driver and actuator
driver = rmd.CanDriver("can0")
actuator = rmd.ActuatorInterface(driver, 1)

# Read motor info
print(actuator.getVersionDate())

# Move to position
actuator.sendPositionAbsoluteSetpoint(180.0, 500.0)

# Shutdown
actuator.shutdownMotor()
```

If installed via ROS 2:
```python
import myactuator_rmd.myactuator_rmd_py as rmd
```

## CAN Protocol Reference

### CAN ID Structure

| Direction | Formula | Example (Motor ID 1) |
|-----------|---------|---------------------|
| Command (TX) | 0x140 + ID | 0x141 |
| Response (RX) | 0x240 + ID | 0x241 |

### Common Commands

| Command | Code | Description |
|---------|------|-------------|
| Position Control | 0xA4 | Absolute position closed-loop |
| Velocity Control | 0xA2 | Velocity closed-loop |
| Torque Control | 0xA1 | Current/torque closed-loop |
| Read Status | 0x9C | Position, velocity, current feedback |
| Shutdown | 0x80 | Disable motor |
| Stop | 0x81 | Stop and hold position |

### Position Command Example

```bash
# Move motor ID 1 to 45 degrees at 720 dps
# Frame: A4 00 SpeedL SpeedH PosB0 PosB1 PosB2 PosB3
# Speed: 720 = 0x02D0 (little-endian: D0 02)
# Position: 45.00 deg = 4500 units = 0x1194 (little-endian: 94 11 00 00)
cansend can0 141#A400D00294110000
```

### Reading Motor Status

```bash
# Request status from motor ID 1
cansend can0 141#9C00000000000000

# Response format: 9C Temp CurrentL CurrentH SpeedL SpeedH EncoderL EncoderH
```

## Testing

### Build Tests

```bash
mkdir build && cd build
cmake .. -DBUILD_TESTING=on
make -j$(nproc)
```

### Setup Virtual CAN for Testing

```bash
sudo modprobe vcan
sudo ip link add dev vcan_test type vcan
sudo ip link set up vcan_test
```

### Run Tests

```bash
ctest
```

## API Reference

### ActuatorInterface Methods

| Method | Description |
|--------|-------------|
| `getVersionDate()` | Get firmware version date |
| `sendPositionAbsoluteSetpoint(deg, dps)` | Move to absolute position |
| `sendVelocitySetpoint(dps)` | Set velocity |
| `sendTorqueSetpoint(current_A)` | Set torque via current |
| `getMotorStatus1()` | Read temperature, voltage, errors |
| `getMotorStatus2()` | Read position, velocity, current |
| `shutdownMotor()` | Disable motor output |
| `stopMotor()` | Stop and hold position |

### Error Handling

The library throws exceptions on communication errors:

```cpp
try {
    actuator.sendPositionAbsoluteSetpoint(180.0, 500.0);
} catch (const myactuator_rmd::Exception& e) {
    std::cerr << "Motor error: " << e.what() << std::endl;
}
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "Failed to open socket" | Check CAN interface is up: `ip link show can0` |
| No motor response | Verify CAN ID, wiring, termination, power |
| "Bus error" | Check bitrate matches motor config (1 Mbps default) |
| Timeout errors | Reduce control loop frequency, check bus load |

### Debugging CAN Traffic

```bash
# Monitor all CAN frames
candump can0

# With timestamps
candump -t d can0

# Filter by motor ID 1 (0x141 and 0x241)
candump can0,141:7FF,241:7FF
```

## License

MIT License - See [License.md](License.md)
