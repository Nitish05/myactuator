"""
Motor wrapper for MyActuator RMD motors using Python bindings.

Provides a high-level interface for motor control with unit conversions.
"""

import math
from typing import Optional, Tuple
from dataclasses import dataclass

# Import myactuator_rmd Python bindings
try:
    from myactuator_rmd import myactuator_rmd_py as rmd
    from myactuator_rmd.myactuator_rmd_py import actuator_state
    RMD_AVAILABLE = True
except ImportError:
    try:
        # Fallback to direct import
        import myactuator_rmd_py as rmd
        from myactuator_rmd_py import actuator_state
        RMD_AVAILABLE = True
    except ImportError:
        RMD_AVAILABLE = False
        print("Warning: myactuator_rmd_py not available. Motor control disabled.")


@dataclass
class MotorState:
    """Current state of a motor."""
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    effort_nm: float = 0.0
    temperature_c: int = 0
    voltage_v: float = 0.0
    current_a: float = 0.0
    error_code: int = 0


def deg_to_rad(deg: float) -> float:
    """Convert degrees to radians."""
    return deg * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    """Convert radians to degrees."""
    return rad * 180.0 / math.pi


class MotorWrapper:
    """
    Wrapper class for a single MyActuator RMD motor.
    
    Handles unit conversions (radians <-> degrees) and provides
    a simplified interface for motor control.
    """

    def __init__(self, driver: 'rmd.CanDriver', can_id: int, 
                 torque_constant: float = 0.21,
                 max_velocity: float = 720.0,
                 position_offset: float = 0.0,
                 inverted: bool = False,
                 timeout_ms: int = 100):
        """
        Initialize motor wrapper.
        
        Args:
            driver: CAN driver instance
            can_id: Motor CAN ID (1-32)
            torque_constant: Motor torque constant in Nm/A
            max_velocity: Maximum velocity in deg/s
            position_offset: Position offset in degrees
            inverted: Invert motor direction
            timeout_ms: Communication timeout in milliseconds
        """
        if not RMD_AVAILABLE:
            raise RuntimeError("myactuator_rmd_py bindings not available")
        
        self.driver = driver
        self.can_id = can_id
        self.torque_constant = torque_constant
        self.max_velocity = max_velocity
        self.position_offset = position_offset
        self.inverted = inverted
        
        # Create actuator interface
        self.actuator = rmd.ActuatorInterface(driver, can_id)
        
        # Set communication timeout
        import datetime
        try:
            self.actuator.setTimeout(datetime.timedelta(milliseconds=timeout_ms))
        except Exception:
            pass  # Timeout setting may fail on some firmware
        
        # Cache last known state
        self._last_state = MotorState()

    def _apply_direction(self, value: float) -> float:
        """Apply direction inversion if needed."""
        return -value if self.inverted else value

    def _apply_offset(self, position_deg: float) -> float:
        """Apply position offset."""
        return position_deg - self.position_offset

    def _remove_offset(self, position_deg: float) -> float:
        """Remove position offset for sending commands."""
        return position_deg + self.position_offset

    def get_state(self) -> MotorState:
        """
        Get current motor state.
        
        Returns:
            MotorState with position (rad), velocity (rad/s), effort (Nm)
        """
        try:
            status2 = self.actuator.getMotorStatus2()
            
            # Apply offset and direction
            position_deg = self._apply_direction(self._apply_offset(status2.shaft_angle))
            velocity_deg_s = self._apply_direction(status2.shaft_speed)
            current_a = status2.current
            
            # Convert to ROS units
            self._last_state = MotorState(
                position_rad=deg_to_rad(position_deg),
                velocity_rad_s=deg_to_rad(velocity_deg_s),
                effort_nm=current_a * self.torque_constant,
                temperature_c=status2.temperature,
                current_a=current_a,
            )
            return self._last_state
            
        except Exception as e:
            # Return last known state on error
            return self._last_state

    def get_extended_status(self) -> Tuple[MotorState, int, float]:
        """
        Get extended motor status including voltage and error code.
        
        Returns:
            Tuple of (MotorState, error_code, voltage)
        """
        state = self.get_state()
        try:
            status1 = self.actuator.getMotorStatus1()
            state.voltage_v = status1.voltage
            state.error_code = status1.error_code.value if hasattr(status1.error_code, 'value') else int(status1.error_code)
            state.temperature_c = status1.temperature
            return state, state.error_code, state.voltage_v
        except Exception:
            return state, 0, 0.0

    def send_position(self, position_rad: float, max_speed_rad_s: Optional[float] = None) -> MotorState:
        """
        Send position setpoint.
        
        Args:
            position_rad: Target position in radians
            max_speed_rad_s: Maximum speed in rad/s (optional)
            
        Returns:
            Motor state after command
        """
        position_deg = rad_to_deg(position_rad)
        position_deg = self._apply_direction(position_deg)
        position_deg = self._remove_offset(position_deg)
        
        if max_speed_rad_s is not None:
            max_speed_deg = rad_to_deg(max_speed_rad_s)
        else:
            max_speed_deg = self.max_velocity
        
        try:
            feedback = self.actuator.sendPositionAbsoluteSetpoint(position_deg, max_speed_deg)
            return self._feedback_to_state(feedback)
        except Exception:
            return self._last_state

    def send_velocity(self, velocity_rad_s: float) -> MotorState:
        """
        Send velocity setpoint.
        
        Args:
            velocity_rad_s: Target velocity in rad/s
            
        Returns:
            Motor state after command
        """
        velocity_deg = rad_to_deg(velocity_rad_s)
        velocity_deg = self._apply_direction(velocity_deg)
        
        try:
            feedback = self.actuator.sendVelocitySetpoint(velocity_deg)
            return self._feedback_to_state(feedback)
        except Exception:
            return self._last_state

    def send_torque(self, torque_nm: float) -> MotorState:
        """
        Send torque setpoint.
        
        Args:
            torque_nm: Target torque in Nm
            
        Returns:
            Motor state after command
        """
        torque = self._apply_direction(torque_nm)
        
        try:
            feedback = self.actuator.sendTorqueSetpoint(torque, self.torque_constant)
            return self._feedback_to_state(feedback)
        except Exception:
            return self._last_state

    def send_current(self, current_a: float) -> MotorState:
        """
        Send current setpoint.
        
        Args:
            current_a: Target current in Amperes
            
        Returns:
            Motor state after command
        """
        current = self._apply_direction(current_a)
        
        try:
            feedback = self.actuator.sendCurrentSetpoint(current)
            return self._feedback_to_state(feedback)
        except Exception:
            return self._last_state

    def _feedback_to_state(self, feedback) -> MotorState:
        """Convert feedback to MotorState."""
        position_deg = self._apply_direction(self._apply_offset(feedback.shaft_angle))
        velocity_deg_s = self._apply_direction(feedback.shaft_speed)
        
        self._last_state = MotorState(
            position_rad=deg_to_rad(position_deg),
            velocity_rad_s=deg_to_rad(velocity_deg_s),
            effort_nm=feedback.current * self.torque_constant,
            temperature_c=feedback.temperature,
            current_a=feedback.current,
        )
        return self._last_state

    def stop(self):
        """Stop the motor."""
        try:
            self.actuator.stopMotor()
        except Exception:
            pass

    def release(self):
        """
        Release the motor (make it free to move by hand).
        
        Sends zero current command which effectively disables holding torque.
        """
        try:
            # Send zero current to release motor
            self.actuator.sendCurrentSetpoint(0.0)
        except Exception:
            pass

    def shutdown(self):
        """Shutdown the motor."""
        try:
            self.actuator.shutdownMotor()
        except Exception:
            pass

    def set_zero(self):
        """Set current position as encoder zero."""
        try:
            self.actuator.setCurrentPositionAsEncoderZero()
        except Exception:
            pass

    def release_brake(self):
        """Release the holding brake."""
        try:
            self.actuator.releaseBrake()
        except Exception:
            pass

    def lock_brake(self):
        """Lock the holding brake."""
        try:
            self.actuator.lockBrake()
        except Exception:
            pass

    def get_model(self) -> str:
        """Get motor model string."""
        try:
            return self.actuator.getMotorModel()
        except Exception:
            return "Unknown"
