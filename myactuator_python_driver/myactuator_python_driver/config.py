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
class HysteresisTorqueTrigger:
    """
    Hysteresis-based torque override trigger for playback.

    When joint angle crosses enter_threshold, switches to torque control.
    When joint angle crosses exit_threshold (in opposite direction),
    returns to position control. The hysteresis gap prevents oscillation.

    Example (rising direction):
        enter_threshold_rad = 1.5  # switch to torque when angle > 1.5
        exit_threshold_rad = 1.3   # switch back to position when angle < 1.3
        torque_nm = 5.0            # apply 5 Nm when in torque mode
    """
    joint_name: str
    enter_threshold_rad: float
    exit_threshold_rad: float
    torque_nm: float
    direction: str = "rising"  # "rising" or "falling"
    enabled: bool = True
    name: str = ""  # Optional trigger name
    recording_name: str = ""  # Recording this trigger is paired with

    def __post_init__(self):
        if self.direction == "rising":
            if self.exit_threshold_rad >= self.enter_threshold_rad:
                raise ValueError("For rising: exit_threshold must be < enter_threshold")
        elif self.direction == "falling":
            if self.exit_threshold_rad <= self.enter_threshold_rad:
                raise ValueError("For falling: exit_threshold must be > enter_threshold")
        else:
            raise ValueError("direction must be 'rising' or 'falling'")

    @classmethod
    def create_falling(cls, name: str, joint_name: str, threshold_rad: float,
                       torque_nm: float, recording_name: str = "",
                       hysteresis: float = 0.1) -> 'HysteresisTorqueTrigger':
        """Create a falling trigger (activates when position goes below threshold)."""
        return cls(
            name=name,
            joint_name=joint_name,
            enter_threshold_rad=threshold_rad,
            exit_threshold_rad=threshold_rad + hysteresis,
            torque_nm=torque_nm,
            direction="falling",
            recording_name=recording_name,
        )


@dataclass
class PlaybackTriggerConfig:
    """Configuration for all triggers during playback."""
    triggers: List['HysteresisTorqueTrigger'] = field(default_factory=list)

    def get_trigger_for_joint(self, joint_name: str) -> Optional['HysteresisTorqueTrigger']:
        """Get trigger config for a specific joint."""
        for t in self.triggers:
            if t.joint_name == joint_name and t.enabled:
                return t
        return None

    def to_dict(self) -> dict:
        """Serialize for JSON/ROS message."""
        return {'triggers': [asdict(t) for t in self.triggers]}

    @classmethod
    def from_dict(cls, data: dict) -> 'PlaybackTriggerConfig':
        """Deserialize from JSON/ROS message."""
        triggers = [HysteresisTorqueTrigger(**t) for t in data.get('triggers', [])]
        return cls(triggers=triggers)


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
    publish_rate: float = 500.0  # Hz - higher for smoother admittance control
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


class TriggerStore:
    """Persistent storage for torque triggers."""

    def __init__(self, storage_dir: Path):
        self.storage_dir = storage_dir
        self.storage_file = storage_dir / "triggers.json"
        self._triggers: List[HysteresisTorqueTrigger] = []
        self.load()

    def load(self):
        """Load triggers from disk."""
        import json
        self._triggers = []
        if self.storage_file.exists():
            try:
                with open(self.storage_file, 'r') as f:
                    data = json.load(f)
                for t in data.get('triggers', []):
                    try:
                        self._triggers.append(HysteresisTorqueTrigger(**t))
                    except (TypeError, ValueError):
                        pass  # Skip invalid triggers
            except Exception:
                pass

    def save(self):
        """Save triggers to disk."""
        import json
        self.storage_dir.mkdir(parents=True, exist_ok=True)
        data = {'triggers': [asdict(t) for t in self._triggers]}
        with open(self.storage_file, 'w') as f:
            json.dump(data, f, indent=2)

    def get_all(self) -> List[HysteresisTorqueTrigger]:
        """Get all triggers."""
        return list(self._triggers)

    def get_for_recording(self, recording_name: str) -> List[HysteresisTorqueTrigger]:
        """Get triggers paired with a specific recording."""
        return [t for t in self._triggers if t.recording_name == recording_name]

    def add(self, trigger: HysteresisTorqueTrigger):
        """Add a trigger and save."""
        self._triggers.append(trigger)
        self.save()

    def remove(self, trigger: HysteresisTorqueTrigger):
        """Remove a trigger and save."""
        self._triggers = [t for t in self._triggers if not (
            t.name == trigger.name and t.joint_name == trigger.joint_name
            and t.recording_name == trigger.recording_name
        )]
        self.save()

    def remove_by_name(self, name: str):
        """Remove trigger by name and save."""
        self._triggers = [t for t in self._triggers if t.name != name]
        self.save()

    def update(self, old_trigger: HysteresisTorqueTrigger, new_trigger: HysteresisTorqueTrigger):
        """Update a trigger and save."""
        self.remove(old_trigger)
        self.add(new_trigger)
