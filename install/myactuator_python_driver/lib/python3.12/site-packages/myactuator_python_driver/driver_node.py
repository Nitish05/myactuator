#!/usr/bin/env python3
"""
Main driver node for MyActuator RMD motors.

Publishes /joint_states and subscribes to control topics.
"""

import math
import threading
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Bool
from std_srvs.srv import Trigger, SetBool

from myactuator_python_driver.config import DriverConfig, MotorConfig
from myactuator_python_driver.motor_wrapper import MotorWrapper, MotorState, RMD_AVAILABLE

if RMD_AVAILABLE:
    try:
        from myactuator_rmd import myactuator_rmd_py as rmd
    except ImportError:
        import myactuator_rmd_py as rmd


class MotorDriverNode(Node):
    """
    ROS2 node for driving multiple MyActuator RMD motors.
    
    Topics Published:
        /joint_states (sensor_msgs/JointState): Position, velocity, effort
        /motor_status (std_msgs/Float64MultiArray): Temperature, voltage per motor
        ~/mode (std_msgs/String): Current control mode
        
    Topics Subscribed:
        /joint_state_ctrl (sensor_msgs/JointState): Position/velocity control
        /joint_effort_ctrl (std_msgs/Float64MultiArray): Torque control
        ~/set_mode (std_msgs/String): Set control mode (position/velocity/torque/free)
        ~/set_enabled (std_msgs/Bool): Enable/disable motors
        
    Services:
        ~/set_zero (std_srvs/Trigger): Set current position as zero for all motors
        ~/emergency_stop (std_srvs/Trigger): Emergency stop all motors
        ~/enable_torque (std_srvs/SetBool): Enable/disable torque mode
        ~/set_free (std_srvs/SetBool): Enable/disable free mode (move motors by hand)
    """

    def __init__(self, config: Optional[DriverConfig] = None):
        super().__init__('motor_driver')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Load config from parameters or use provided
        if config is None:
            config = self._load_config_from_params()
        self.config = config
        
        # Motor state
        self.motors: Dict[str, MotorWrapper] = {}
        self.driver: Optional['rmd.CanDriver'] = None
        self._lock = threading.Lock()
        self._running = True
        
        # Control mode state
        self._torque_mode = False
        self._free_mode = False  # When True, motors are free to move by hand
        self._enabled = True  # When False, motors are disabled (no commands sent)
        self._control_positions: Dict[str, float] = {}
        self._control_velocities: Dict[str, float] = {}
        self._control_efforts: Dict[str, float] = {}
        
        # Initialize control values for configured joints
        for motor_cfg in self.config.motors:
            self._control_positions[motor_cfg.joint_name] = 0.0
            self._control_velocities[motor_cfg.joint_name] = 0.0
            self._control_efforts[motor_cfg.joint_name] = 0.0
        
        # Initialize hardware (continue even if it fails for testing)
        hardware_ok = self._init_hardware()
        if not hardware_ok:
            self.get_logger().warn("Hardware init failed - running in simulation mode")
        
        # QoS for real-time topics
        realtime_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        self.motor_status_pub = self.create_publisher(
            Float64MultiArray, '/motor_status', 10)
        self.mode_pub = self.create_publisher(
            String, '~/mode', 10)
        
        # Subscribers
        self.joint_ctrl_sub = self.create_subscription(
            JointState, '/joint_state_ctrl',
            self._joint_ctrl_callback, 10,
            callback_group=self.callback_group)
        self.effort_ctrl_sub = self.create_subscription(
            Float64MultiArray, '/joint_effort_ctrl',
            self._effort_ctrl_callback, realtime_qos,
            callback_group=self.callback_group)
        self.set_mode_sub = self.create_subscription(
            String, '~/set_mode',
            self._set_mode_callback, 10,
            callback_group=self.callback_group)
        self.set_enabled_sub = self.create_subscription(
            Bool, '~/set_enabled',
            self._set_enabled_callback, 10,
            callback_group=self.callback_group)
        
        # Services
        self.set_zero_srv = self.create_service(
            Trigger, '~/set_zero', self._set_zero_callback,
            callback_group=self.callback_group)
        self.estop_srv = self.create_service(
            Trigger, '~/emergency_stop', self._emergency_stop_callback,
            callback_group=self.callback_group)
        self.enable_torque_srv = self.create_service(
            SetBool, '~/enable_torque', self._enable_torque_callback,
            callback_group=self.callback_group)
        self.set_free_srv = self.create_service(
            SetBool, '~/set_free', self._set_free_callback,
            callback_group=self.callback_group)
        
        # Main control loop timer
        period = 1.0 / self.config.publish_rate
        self.timer = self.create_timer(period, self._control_loop, callback_group=self.callback_group)
        
        # Publish initial mode
        self._publish_mode()
        
        self.get_logger().info(
            f"Motor driver initialized with {len(self.motors)} motors on {self.config.can_interface}")

    def _load_config_from_params(self) -> DriverConfig:
        """Load configuration from ROS parameters."""
        # Declare parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('control_mode', 'position')
        self.declare_parameter('config_file', '')
        
        # Check for config file parameter
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        if config_file:
            try:
                from pathlib import Path
                return DriverConfig.load(Path(config_file))
            except Exception as e:
                self.get_logger().warn(f"Failed to load config file: {e}")
        
        # Build config from individual parameters
        config = DriverConfig(
            can_interface=self.get_parameter('can_interface').get_parameter_value().string_value,
            publish_rate=self.get_parameter('publish_rate').get_parameter_value().double_value,
            control_mode=self.get_parameter('control_mode').get_parameter_value().string_value,
        )
        
        # Try to load motor configs
        self.declare_parameter('motor_ids', [1])
        self.declare_parameter('joint_names', ['joint1'])
        self.declare_parameter('torque_constants', [0.21])
        
        motor_ids = self.get_parameter('motor_ids').get_parameter_value().integer_array_value
        joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        torque_constants = self.get_parameter('torque_constants').get_parameter_value().double_array_value
        
        # Pad arrays if needed
        while len(joint_names) < len(motor_ids):
            joint_names.append(f'joint{len(joint_names) + 1}')
        while len(torque_constants) < len(motor_ids):
            torque_constants.append(0.21)
        
        for i, can_id in enumerate(motor_ids):
            config.motors.append(MotorConfig(
                can_id=can_id,
                joint_name=joint_names[i],
                torque_constant=torque_constants[i],
            ))
        
        return config

    def _init_hardware(self) -> bool:
        """Initialize CAN driver and motors."""
        if not RMD_AVAILABLE:
            self.get_logger().error("myactuator_rmd_py bindings not available!")
            return False
        
        try:
            self.driver = rmd.CanDriver(self.config.can_interface)
            
            for motor_cfg in self.config.motors:
                wrapper = MotorWrapper(
                    driver=self.driver,
                    can_id=motor_cfg.can_id,
                    torque_constant=motor_cfg.torque_constant,
                    max_velocity=motor_cfg.max_velocity,
                    position_offset=motor_cfg.position_offset,
                    inverted=motor_cfg.inverted,
                )
                self.motors[motor_cfg.joint_name] = wrapper
                
                # Get motor model
                try:
                    model = wrapper.get_model()
                    self.get_logger().info(
                        f"Motor {motor_cfg.joint_name} (ID {motor_cfg.can_id}): {model}")
                except Exception:
                    self.get_logger().warn(
                        f"Could not get model for motor {motor_cfg.joint_name}")
                
                # Read current position and use as initial setpoint (don't jump to zero!)
                try:
                    state = wrapper.get_state()
                    self._control_positions[motor_cfg.joint_name] = state.position_rad
                    self.get_logger().info(
                        f"Motor {motor_cfg.joint_name} initial position: {state.position_rad:.3f} rad")
                except Exception:
                    self.get_logger().warn(
                        f"Could not read initial position for {motor_cfg.joint_name}, using 0.0")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Hardware init failed: {e}")
            return False

    def _get_current_mode(self) -> str:
        """Get the current control mode as a string."""
        if not self._enabled:
            return "disabled"
        elif self._free_mode:
            return "free"
        elif self._torque_mode:
            return "torque"
        else:
            return self.config.control_mode  # position or velocity

    def _publish_mode(self):
        """Publish the current control mode."""
        msg = String()
        msg.data = self._get_current_mode()
        self.mode_pub.publish(msg)

    def _control_loop(self):
        """Main control loop - read states and send commands."""
        if not self._running:
            return
        
        # If no motors connected, publish zero state for configured joints
        if not self.motors:
            self.get_logger().debug("No motors, publishing zeros")
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = self.config.joint_names
            joint_msg.position = [0.0] * len(self.config.joint_names)
            joint_msg.velocity = [0.0] * len(self.config.joint_names)
            joint_msg.effort = [0.0] * len(self.config.joint_names)
            self.joint_state_pub.publish(joint_msg)
            return
        
        # Create joint state message
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = []
        joint_msg.position = []
        joint_msg.velocity = []
        joint_msg.effort = []
        
        # Status message: [temp1, voltage1, temp2, voltage2, ...]
        status_msg = Float64MultiArray()
        status_data = []
        
        with self._lock:
            for joint_name in self.config.joint_names:
                motor = self.motors.get(joint_name)
                if motor is None:
                    # No motor connected, publish zeros
                    joint_msg.name.append(joint_name)
                    joint_msg.position.append(0.0)
                    joint_msg.velocity.append(0.0)
                    joint_msg.effort.append(0.0)
                    continue
                
                try:
                    # Send command based on mode
                    if not self._enabled:
                        # Disabled: just read state, no commands
                        state = motor.get_state()
                    elif self._free_mode:
                        # Free mode: release motor and just read state
                        motor.release()
                        state = motor.get_state()
                    elif self._torque_mode:
                        effort = self._control_efforts.get(joint_name, 0.0)
                        state = motor.send_torque(effort)
                    elif self.config.control_mode == 'velocity':
                        velocity = self._control_velocities.get(joint_name, 0.0)
                        state = motor.send_velocity(velocity)
                    elif self.config.control_mode == 'position':
                        position = self._control_positions.get(joint_name, 0.0)
                        state = motor.send_position(position)
                    else:
                        # Read-only mode
                        state = motor.get_state()
                    
                    joint_msg.name.append(joint_name)
                    joint_msg.position.append(state.position_rad)
                    joint_msg.velocity.append(state.velocity_rad_s)
                    joint_msg.effort.append(state.effort_nm)
                    
                    status_data.extend([float(state.temperature_c), state.voltage_v])
                    
                except Exception as e:
                    # On error, still publish with last known or zero state
                    joint_msg.name.append(joint_name)
                    joint_msg.position.append(0.0)
                    joint_msg.velocity.append(0.0)
                    joint_msg.effort.append(0.0)
                    self.get_logger().warn(f"Error reading motor {joint_name}: {e}", throttle_duration_sec=5.0)
        
        # Publish
        if joint_msg.name:
            self.joint_state_pub.publish(joint_msg)
        if status_data:
            status_msg.data = status_data
            self.motor_status_pub.publish(status_msg)

    def _joint_ctrl_callback(self, msg: JointState):
        """Handle position/velocity control commands."""
        with self._lock:
            for i, name in enumerate(msg.name):
                if name not in self.motors:
                    continue
                
                if i < len(msg.position) and msg.position[i] != 0.0:
                    self._control_positions[name] = msg.position[i]
                
                if i < len(msg.velocity) and msg.velocity[i] != 0.0:
                    self._control_velocities[name] = msg.velocity[i]
                
                if i < len(msg.effort):
                    self._control_efforts[name] = msg.effort[i]

    def _effort_ctrl_callback(self, msg: Float64MultiArray):
        """Handle torque control commands."""
        with self._lock:
            self._torque_mode = True
            for i, effort in enumerate(msg.data):
                if i < len(self.config.joint_names):
                    joint_name = self.config.joint_names[i]
                    self._control_efforts[joint_name] = effort

    def _set_zero_callback(self, request, response):
        """Set current position as zero for all motors."""
        with self._lock:
            for motor in self.motors.values():
                try:
                    motor.set_zero()
                except Exception as e:
                    response.success = False
                    response.message = f"Failed: {e}"
                    return response
        
        response.success = True
        response.message = "All motors zeroed"
        self.get_logger().info("All motors zeroed")
        return response

    def _emergency_stop_callback(self, request, response):
        """Emergency stop all motors."""
        with self._lock:
            self._torque_mode = False
            for motor in self.motors.values():
                try:
                    motor.stop()
                except Exception:
                    pass
        
        response.success = True
        response.message = "Emergency stop executed"
        self.get_logger().warn("Emergency stop executed!")
        return response

    def _enable_torque_callback(self, request, response):
        """Enable or disable torque mode."""
        with self._lock:
            if request.data and self._free_mode:
                self._capture_positions_locked()
            self._torque_mode = request.data
            self._free_mode = False  # Exit free mode when enabling torque control
            self._enabled = True
            if not request.data:
                # Reset efforts when disabling
                for name in self._control_efforts:
                    self._control_efforts[name] = 0.0
        
        self._publish_mode()
        response.success = True
        response.message = f"Torque mode {'enabled' if request.data else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def _set_free_callback(self, request, response):
        """Enable or disable free mode (motors can be moved by hand)."""
        with self._lock:
            self._free_mode = request.data
            if request.data:
                # Disable other control modes when entering free mode
                self._torque_mode = False
                self.get_logger().info("Free mode enabled - motors can be moved by hand")
            else:
                # When exiting free mode, read current positions as new setpoints
                for joint_name, motor in self.motors.items():
                    try:
                        state = motor.get_state()
                        self._control_positions[joint_name] = state.position_rad
                        self.get_logger().info(
                            f"Motor {joint_name} position captured: {state.position_rad:.3f} rad")
                    except Exception:
                        pass
                self.get_logger().info("Free mode disabled - motors will hold position")
        
        self._publish_mode()
        response.success = True
        response.message = f"Free mode {'enabled' if request.data else 'disabled'}"
        return response

    def _set_mode_callback(self, msg: String):
        """Handle mode change from topic."""
        mode = msg.data.lower().strip()
        
        with self._lock:
            if mode == "free":
                if not self._free_mode:
                    self._free_mode = True
                    self._torque_mode = False
                    self._enabled = True
                    self.get_logger().info("Mode changed to: free")
            elif mode == "torque":
                if self._free_mode:
                    self._capture_positions_locked()
                self._free_mode = False
                self._torque_mode = True
                self._enabled = True
                self.get_logger().info("Mode changed to: torque")
            elif mode == "position":
                if self._free_mode:
                    self._capture_positions_locked()
                self._free_mode = False
                self._torque_mode = False
                self._enabled = True
                self.config.control_mode = "position"
                self.get_logger().info("Mode changed to: position")
            elif mode == "velocity":
                if self._free_mode:
                    self._capture_positions_locked()
                self._free_mode = False
                self._torque_mode = False
                self._enabled = True
                self.config.control_mode = "velocity"
                self.get_logger().info("Mode changed to: velocity")
            elif mode == "disabled" or mode == "disable":
                self._enabled = False
                self._free_mode = False
                self._torque_mode = False
                self.get_logger().info("Motors disabled")
            else:
                self.get_logger().warn(f"Unknown mode: {mode}. Valid: position, velocity, torque, free, disabled")
                return
        
        self._publish_mode()

    def _set_enabled_callback(self, msg: Bool):
        """Handle enable/disable from topic."""
        with self._lock:
            if msg.data:
                # Re-enable: capture current positions first
                if not self._enabled:
                    self._capture_positions_locked()
                self._enabled = True
                self.get_logger().info("Motors enabled")
            else:
                self._enabled = False
                self._free_mode = False
                self._torque_mode = False
                self.get_logger().info("Motors disabled")
        
        self._publish_mode()

    def _capture_positions_locked(self):
        """Capture current motor positions as setpoints. Must be called with lock held."""
        for joint_name, motor in self.motors.items():
            try:
                state = motor.get_state()
                self._control_positions[joint_name] = state.position_rad
                self.get_logger().info(
                    f"Motor {joint_name} position captured: {state.position_rad:.3f} rad")
            except Exception:
                pass

    def shutdown(self):
        """Shutdown the driver."""
        self._running = False
        with self._lock:
            for motor in self.motors.values():
                try:
                    motor.shutdown()
                except Exception:
                    pass
        self.get_logger().info("Motor driver shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    # Try to load config from default location
    config = None
    try:
        config_path = DriverConfig.default_config_path()
        if config_path.exists():
            config = DriverConfig.load(config_path)
            print(f"Loaded config from {config_path}")
    except Exception:
        pass
    
    node = MotorDriverNode(config)
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
