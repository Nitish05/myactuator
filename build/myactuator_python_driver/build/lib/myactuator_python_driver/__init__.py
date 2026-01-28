# MyActuator Python Driver
"""
Python ROS2 driver for MyActuator RMD motors.

Provides:
- TUI setup wizard for motor configuration
- Driver node publishing /joint_states
- Recorder node for teach-and-playback with torque override
"""

from myactuator_python_driver.config import DriverConfig, MotorConfig, TorqueRule
