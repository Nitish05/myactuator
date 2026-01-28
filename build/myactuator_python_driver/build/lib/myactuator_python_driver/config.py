"""
Configuration module for MyActuator Python Driver.

Defines dataclasses for motor and driver configuration with YAML support.
"""

import os
import yaml
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional
from pathlib import Path


@dataclass
class TorqueRule:
    """Torque override rule for playback."""
    threshold_deg: float = 45.0
    torque_nm: float = 1.0
    direction: str = "above"  # "above" or "below"

    def check(self, angle_deg: float) -> bool:
        """Check if angle triggers this rule."""
        if self.direction == "above":
            return angle_deg > self.threshold_deg
        else:
            return angle_deg < self.threshold_deg


@dataclass
class MotorConfig:
    """Configuration for a single motor."""
    can_id: int = 1
    joint_name: str = "joint1"
    torque_constant: float = 0.3  # Nm/A - depends on motor model
    max_velocity: float = 720.0  # deg/s
    position_offset: float = 0.0  # deg - zero offset
    inverted: bool = False  # Invert direction
    torque_rule: Optional[TorqueRule] = None

    def __post_init__(self):
        if self.torque_rule is not None and isinstance(self.torque_rule, dict):
            self.torque_rule = TorqueRule(**self.torque_rule)


@dataclass
class DriverConfig:
    """Main driver configuration."""
    can_interface: str = "can0"
    motors: List[MotorConfig] = field(default_factory=list)
    publish_rate: float = 100.0  # Hz
    control_mode: str = "position"  # "position", "velocity", "torque"
    timeout_ms: int = 0  # Communication timeout (0 = disabled)

    def __post_init__(self):
        # Convert dict motors to MotorConfig objects
        if self.motors and isinstance(self.motors[0], dict):
            self.motors = [MotorConfig(**m) for m in self.motors]

    @property
    def joint_names(self) -> List[str]:
        """Get ordered list of joint names."""
        return [m.joint_name for m in self.motors]

    @property
    def num_motors(self) -> int:
        """Get number of motors."""
        return len(self.motors)

    def get_motor_by_joint(self, joint_name: str) -> Optional[MotorConfig]:
        """Get motor config by joint name."""
        for m in self.motors:
            if m.joint_name == joint_name:
                return m
        return None

    def get_motor_by_id(self, can_id: int) -> Optional[MotorConfig]:
        """Get motor config by CAN ID."""
        for m in self.motors:
            if m.can_id == can_id:
                return m
        return None

    def to_yaml(self) -> str:
        """Serialize config to YAML string."""
        data = {
            'can_interface': self.can_interface,
            'publish_rate': self.publish_rate,
            'control_mode': self.control_mode,
            'timeout_ms': self.timeout_ms,
            'motors': []
        }
        for m in self.motors:
            motor_dict = {
                'can_id': m.can_id,
                'joint_name': m.joint_name,
                'torque_constant': m.torque_constant,
                'max_velocity': m.max_velocity,
                'position_offset': m.position_offset,
                'inverted': m.inverted,
            }
            if m.torque_rule:
                motor_dict['torque_rule'] = asdict(m.torque_rule)
            data['motors'].append(motor_dict)
        return yaml.dump(data, default_flow_style=False, sort_keys=False)

    def save(self, path: Path):
        """Save config to YAML file."""
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w') as f:
            f.write(self.to_yaml())

    @classmethod
    def from_yaml(cls, yaml_str: str) -> 'DriverConfig':
        """Load config from YAML string."""
        data = yaml.safe_load(yaml_str)
        return cls(**data)

    @classmethod
    def load(cls, path: Path) -> 'DriverConfig':
        """Load config from YAML file."""
        with open(path, 'r') as f:
            return cls.from_yaml(f.read())

    @classmethod
    def default_config_path(cls) -> Path:
        """Get default config path."""
        # Try package share directory first
        try:
            from ament_index_python.packages import get_package_share_directory
            return Path(get_package_share_directory('myactuator_python_driver')) / 'config' / 'driver_config.yaml'
        except Exception:
            pass
        # Fallback to home directory
        return Path.home() / '.config' / 'myactuator_python_driver' / 'driver_config.yaml'


# Motor model torque constants (Nm/A) - from myactuator_rmd actuator_constants.hpp
MOTOR_TORQUE_CONSTANTS = {
    'X4V2': 0.11,
    'X4V3': 0.11,
    'X4_3': 0.11,
    'X4_24': 0.11,
    'X6V2': 0.16,
    'X6S2V2': 0.16,
    'X6V3': 0.16,
    'X6_7': 0.16,
    'X6_8': 0.16,
    'X6_40': 0.16,
    'X8V2': 0.21,
    'X8ProV2': 0.21,
    'X8S2V3': 0.21,
    'X8HV3': 0.21,
    'X8ProHV3': 0.21,
    'X8_20': 0.21,
    'X8_25': 0.21,
    'X8_60': 0.21,
    'X8_90': 0.21,
    'X10V3': 0.32,
    'X10S2V3': 0.32,
    'X10_40': 0.32,
    'X10_100': 0.32,
    'X12_150': 0.42,
    'X15_400': 0.65,
}


def get_torque_constant(motor_model: str) -> float:
    """Get torque constant for a motor model."""
    # Normalize model name
    model = motor_model.upper().replace('-', '').replace('_', '')
    for key, value in MOTOR_TORQUE_CONSTANTS.items():
        if key.upper().replace('_', '') in model:
            return value
    return 0.21  # Default to X8 series
