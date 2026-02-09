"""
Calibrator package -- automated torque threshold calibration.

Provides a headless CalibrationController that orchestrates torque application,
position tracking, bag recording, and safety monitoring using existing
RosBridge and RecordingManager components.
"""

from .config import CalibrationConfig, CalibrationResult, CalibrationState
from .controller import CalibrationController

__all__ = [
    "CalibrationConfig",
    "CalibrationController",
    "CalibrationResult",
    "CalibrationState",
]
