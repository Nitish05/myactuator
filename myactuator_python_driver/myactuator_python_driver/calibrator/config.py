"""
Data models for the calibration controller.

Defines the state machine enum, configuration, and result dataclasses
used by CalibrationController.
"""

from dataclasses import dataclass
from enum import Enum, auto


class CalibrationState(Enum):
    """States of the calibration state machine."""

    IDLE = auto()       # Ready, not recording
    RECORDING = auto()  # Torque applied, bag recording, tracking max position
    STOPPING = auto()   # Transitioning back to safe state
    ERROR = auto()      # Something went wrong (connection lost, etc.)


@dataclass
class CalibrationConfig:
    """Configuration for a calibration run."""

    joint_name: str             # Which joint receives torque
    torque_nm: float            # Torque to apply (signed, direction matters)
    recording_name: str = ""    # Optional recording name (auto-generated if empty)
    offset_deg: float = 0.5     # Threshold offset in degrees (used in Phase 3)


@dataclass
class CalibrationResult:
    """Result of a completed calibration run."""

    recording_name: str         # Name of the bag recording
    joint_name: str             # Which joint was calibrated
    max_position_rad: float     # Highest position reached in torque direction
    torque_nm: float            # Torque that was applied
    duration_sec: float         # How long calibration ran
