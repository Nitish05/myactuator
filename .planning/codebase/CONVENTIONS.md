# Coding Conventions

**Analysis Date:** 2026-02-09

## Naming Patterns

**Files:**
- Python: `snake_case.py` (e.g., `config.py`, `motor_wrapper.py`, `driver_node.py`, `setup_tui.py`)
- C++: `snake_case.cpp` and `snake_case.hpp` (e.g., `actuator_interface.hpp`, `requests.cpp`)
- Test files: Suffix with `_test.cpp` or `_test.hpp` (e.g., `actuator_test.cpp`, `requests_test.cpp`)

**Functions/Methods:**
- Python: `snake_case()` for functions and methods (e.g., `get_state()`, `send_position()`, `_apply_direction()`)
  - Private/internal methods prefix with underscore: `_load_config_from_params()`, `_init_hardware()`
- C++: `camelCase()` for public methods, `snake_case_` for member variables (e.g., `getMotorStatus1()`, `driver_`, `actuator_id_`)

**Variables:**
- Python: `snake_case` for local and module-level variables (e.g., `self.motors`, `config`, `self._running`)
  - Private/internal instance variables prefix with underscore: `self._lock`, `self._torque_mode`, `self._admittance_gain`
  - Constants in UPPERCASE: `MOTOR_TORQUE_CONSTANTS`, `VCAN_IFNAME`
- C++: `camelCase` for local variables, `snake_case_` for member variables with trailing underscore

**Types/Classes:**
- Python: `PascalCase` for classes (e.g., `MotorConfig`, `DriverConfig`, `HysteresisTorqueTrigger`, `MotorWrapper`, `TriggerStore`)
  - Dataclasses are used extensively with `@dataclass` decorator
- C++: `PascalCase` for classes (e.g., `ActuatorInterface`, `CanDriver`, `MotorStatus1`)
  - Namespace: `myactuator_rmd` (lowercase with underscores)

## Code Style

**Formatting:**
- No explicit formatter enforced, but code follows PEP 8-like conventions for Python
- C++ code uses modern C++17 features with 2-space indentation

**Linting:**
- Python: Uses ament_flake8 for linting (ROS 2 standard)
  - Configuration: `setup.cfg` minimal config in `myactuator_python_driver/` and other packages
  - Flake8 test: `test/test_flake8.py` (ament test infrastructure)
- C++: No explicit linting config found, but code is well-structured with headers using `#pragma once` and `#ifndef` guards

**Documentation:**
- Module headers: Each file starts with docstring explaining purpose
- PEP 257 compliance tested in ROS 2 packages via `test/test_pep257.py`

## Import Organization

**Order (Python):**
1. Standard library imports (`os`, `sys`, `math`, `threading`, `pathlib`)
2. Third-party library imports (`rclpy`, `yaml`, `PyQt6`, `dataclasses`)
3. Local package imports (relative, e.g., `from myactuator_python_driver.config import ...`)

**Example from** `driver_node.py`:
```python
import math
import threading
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Bool

from myactuator_python_driver.config import (
    DriverConfig, MotorConfig, HysteresisTorqueTrigger, PlaybackTriggerConfig
)
```

**Path Aliases:**
- No path aliases configured; absolute imports used throughout

**C++ Include Organization:**
1. Standard library (`<chrono>`, `<cstdint>`, `<string>`)
2. Third-party (`<gtest/gtest.h>`, `<gmock/gmock.h>`, `<pybind11/...>`)
3. Local includes using `"myactuator_rmd/..."` relative to include directory

## Error Handling

**Patterns:**
- **Broad exception catching**: Most error handling uses bare `except Exception:` or `except Exception as e:` (seen in `driver_node.py`, `motor_wrapper.py`)
  - Example: `except Exception:` followed by `pass` (graceful degradation, e.g., timeout setting in `MotorWrapper.__init__`)
- **Specific exceptions**: Some specific exception handling (e.g., `except ImportError:` for optional dependencies)
- **Validation in `__post_init__`**: Dataclasses use `__post_init__()` for validation with `raise ValueError()`
  - Example in `HysteresisTorqueTrigger`: validates threshold relationships for rising/falling directions
- **Try/except at CAN communication**: Low-level communication failures caught and handled gracefully
  - Pattern: Read motor status in try/except, return cached state on failure

**C++ error handling:**
- Exceptions used (headers include exception types via includes)
- Return value validation (getters check response validity)

## Logging

**Framework:** Python uses `logging` module (standard library) and `print()` statements

**Patterns:**
- ROS 2 nodes use `self.get_logger()` (from rclpy)
- Logging suppression for noisy libraries (e.g., in `studio/main.py`):
  ```python
  import logging
  logging.getLogger('rosbag2_cpp').setLevel(logging.CRITICAL)
  ```
- TUI modules use `print()` for console output
- Debug logging not heavily used; relies on exceptions and return values

## Comments

**When to Comment:**
- Class docstrings required (using triple-quoted strings)
- Method docstrings required with Args/Returns sections
- Inline comments used sparingly; code should be self-documenting
- Section comments in setup wizards (e.g., `# Motor state`, `# Admittance control parameters`)

**JSDoc/TSDoc:**
- Not used (Python project); docstrings follow Google/NumPy style informally
- C++ uses Doxygen-style comments (`\file`, `\brief`, `\param`, `\return`)
  - Example: headers in C++ files have `/**\file name ... */` blocks with `\author` tags

**Example docstring (Python):**
```python
def send_position(self, position_rad: float, max_speed_rad_s: Optional[float] = None) -> MotorState:
    """
    Send position setpoint.

    Args:
        position_rad: Target position in radians
        max_speed_rad_s: Maximum speed in rad/s (optional)

    Returns:
        MotorState with new position feedback
    """
```

**Example C++ comment (Doxygen):**
```cpp
/**\fn getMotorStatus2
 * \brief
 *    Reads the motor status 2
 *
 * \return
 *    The motor status 2 containing current, speed and position
*/
[[nodiscard]]
MotorStatus2 getMotorStatus2();
```

## Function Design

**Size:** Functions are typically 20-50 lines max; longer functions (like `_init_hardware()` in driver_node) are separated into logical sections with comments

**Parameters:**
- Type hints used throughout Python code (e.g., `position_rad: float`, `config: Optional[DriverConfig]`)
- C++ uses `const` references for non-primitive types
- Default parameters common in configuration dataclasses

**Return Values:**
- Single return values (primitives or dataclass instances) preferred
- Tuples used for multiple values (e.g., `get_extended_status()` returns `Tuple[MotorState, int, float]`)
- Optional returns for query methods (e.g., `get_motor_by_joint()` returns `Optional[MotorConfig]`)
- No `None` return checks in casual code; uses Optional typing

**Example:**
```python
def get_motor_by_joint(self, joint_name: str) -> Optional[MotorConfig]:
    """Get motor config by joint name."""
    for m in self.motors:
        if m.joint_name == joint_name:
            return m
    return None
```

## Module Design

**Exports:**
- Python modules export main classes and functions explicitly
- No `__all__` lists found (implicit exports)
- Dataclasses and utility functions exported from `config.py` module

**Barrel Files:**
- Not used; imports are direct from modules
- Example: `from myactuator_python_driver.config import DriverConfig` (not from `__init__.py`)

**File organization:**
- One main class per file in core modules
- Utility functions grouped in modules (e.g., `deg_to_rad()`, `rad_to_deg()` in `motor_wrapper.py`)
- Configuration dataclasses grouped in single file (`config.py`)

---

*Convention analysis: 2026-02-09*
