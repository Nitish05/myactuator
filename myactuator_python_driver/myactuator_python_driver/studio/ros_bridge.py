"""
ROS 2 bridge for Motor Recording Studio.

Handles all ROS 2 communication in a background thread, emitting Qt signals
for thread-safe GUI updates.
"""

import json
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from PyQt6.QtCore import QObject, QThread, pyqtSignal

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

from myactuator_python_driver.config import PlaybackTriggerConfig, HysteresisTorqueTrigger


@dataclass
class JointData:
    """Data for a single joint."""
    name: str
    position: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0
    temperature: float = 0.0


@dataclass
class RobotState:
    """Complete robot state."""
    joints: List[JointData] = field(default_factory=list)
    mode: str = "unknown"
    connected: bool = False
    trigger_states: Dict[str, dict] = field(default_factory=dict)


class RosBridgeWorker(QObject):
    """Worker that runs ROS 2 communication in a background thread."""

    # Signals for GUI updates
    joint_state_received = pyqtSignal(object)  # JointState msg
    mode_changed = pyqtSignal(str)
    trigger_state_changed = pyqtSignal(dict)
    connection_status_changed = pyqtSignal(bool)
    status_message = pyqtSignal(str)
    error_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._running = False
        self._connected = False
        self._last_msg_time = 0.0
        self._lock = threading.Lock()

    def start_ros(self):
        """Initialize ROS 2 and start spinning."""
        try:
            if not rclpy.ok():
                rclpy.init()

            self._node = Node('motor_studio')
            self._setup_ros()

            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)

            self._running = True
            self.connection_status_changed.emit(False)

            # Spin until stopped
            while self._running and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)
                self._check_connection()

        except Exception as e:
            self.error_message.emit(f"ROS initialization failed: {e}")
        finally:
            self._cleanup()

    def _setup_ros(self):
        """Set up ROS 2 publishers, subscribers, and service clients."""
        cb_group = ReentrantCallbackGroup()

        # Subscribers
        self._joint_state_sub = self._node.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10, callback_group=cb_group)

        self._mode_sub = self._node.create_subscription(
            String, '/motor_driver/mode',
            self._mode_cb, 10, callback_group=cb_group)

        self._trigger_state_sub = self._node.create_subscription(
            String, '/motor_driver/trigger_states',
            self._trigger_state_cb, 10, callback_group=cb_group)

        # Publishers
        self._joint_ctrl_pub = self._node.create_publisher(
            JointState, '/joint_state_ctrl', 10)

        self._set_mode_pub = self._node.create_publisher(
            String, '/motor_driver/set_mode', 10)

        self._set_enabled_pub = self._node.create_publisher(
            Bool, '/motor_driver/set_enabled', 10)

        self._trigger_config_pub = self._node.create_publisher(
            String, '/motor_driver/playback_triggers', 10)

        # Service clients
        self._set_zero_client = self._node.create_client(
            Trigger, '/motor_driver/set_zero')

        self._estop_client = self._node.create_client(
            Trigger, '/motor_driver/emergency_stop')

    def _joint_state_cb(self, msg: JointState):
        """Handle incoming joint states."""
        import time
        with self._lock:
            self._last_msg_time = time.time()
            if not self._connected:
                self._connected = True
                self.connection_status_changed.emit(True)

        self.joint_state_received.emit(msg)

    def _mode_cb(self, msg: String):
        """Handle mode updates from driver."""
        self.mode_changed.emit(msg.data)

    def _trigger_state_cb(self, msg: String):
        """Receive trigger state updates from driver."""
        try:
            states = json.loads(msg.data)
            self.trigger_state_changed.emit(states)
        except Exception:
            pass

    def _check_connection(self):
        """Check if we're still receiving messages."""
        import time
        with self._lock:
            now = time.time()
            was_connected = self._connected
            # Consider disconnected if no message for 2 seconds
            self._connected = (now - self._last_msg_time) < 2.0

            if was_connected and not self._connected:
                self.connection_status_changed.emit(False)

    def stop(self):
        """Stop the ROS bridge."""
        self._running = False

    def _cleanup(self):
        """Clean up ROS resources."""
        if self._executor:
            self._executor.shutdown()
        if self._node:
            self._node.destroy_node()

    # === Methods called from GUI thread (via signal/slot or thread-safe) ===

    def set_mode(self, mode: str):
        """Set motor control mode."""
        if self._node and self._running:
            msg = String()
            msg.data = mode
            self._set_mode_pub.publish(msg)
            self.status_message.emit(f"Mode: {mode}")

    def set_enabled(self, enabled: bool):
        """Enable/disable motors."""
        if self._node and self._running:
            msg = Bool()
            msg.data = enabled
            self._set_enabled_pub.publish(msg)
            self.status_message.emit(f"Motors {'enabled' if enabled else 'disabled'}")

    def emergency_stop(self):
        """Call emergency stop service."""
        if self._node and self._running:
            if self._estop_client.wait_for_service(timeout_sec=0.5):
                self._estop_client.call_async(Trigger.Request())
                self.status_message.emit("EMERGENCY STOP!")
            else:
                self.error_message.emit("Emergency stop service not available")

    def set_zero(self):
        """Call set_zero service."""
        if self._node and self._running:
            if self._set_zero_client.wait_for_service(timeout_sec=0.5):
                self._set_zero_client.call_async(Trigger.Request())
                self.status_message.emit("Set zero requested")
            else:
                self.error_message.emit("set_zero service not available")

    def go_to_zero(self, joint_names: List[str]):
        """Command all joints to zero position."""
        if self._node and self._running:
            msg = JointState()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.name = joint_names
            msg.position = [0.0] * len(joint_names)
            self._joint_ctrl_pub.publish(msg)
            self.status_message.emit("Going to zero")

    def send_joint_command(self, msg: JointState):
        """Send a joint command."""
        if self._node and self._running:
            msg.header.stamp = self._node.get_clock().now().to_msg()
            self._joint_ctrl_pub.publish(msg)

    def get_joint_ctrl_publisher(self):
        """Get the joint control publisher for direct publishing."""
        if self._node and self._running:
            return self._joint_ctrl_pub
        return None

    def set_trigger_config(self, config: PlaybackTriggerConfig):
        """Send trigger configuration to driver."""
        if self._node and self._running:
            msg = String()
            msg.data = json.dumps(config.to_dict())
            self._trigger_config_pub.publish(msg)
            self.status_message.emit(f"Sent {len(config.triggers)} trigger(s)")

    def clear_trigger_config(self):
        """Clear triggers on driver."""
        if self._node and self._running:
            msg = String()
            msg.data = json.dumps({'triggers': []})
            self._trigger_config_pub.publish(msg)


class RosBridge(QObject):
    """
    Main ROS bridge class that manages the worker thread.

    This class provides a clean interface for the GUI to interact with ROS 2.
    All ROS communication happens in a background thread.
    """

    # Forward signals from worker
    joint_state_received = pyqtSignal(object)
    mode_changed = pyqtSignal(str)
    trigger_state_changed = pyqtSignal(dict)
    connection_status_changed = pyqtSignal(bool)
    status_message = pyqtSignal(str)
    error_message = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread: Optional[QThread] = None
        self._worker: Optional[RosBridgeWorker] = None
        self._current_mode = "unknown"
        self._connected = False
        self._joint_names: List[str] = []

    def start(self):
        """Start the ROS bridge in a background thread."""
        if self._thread is not None:
            return

        self._thread = QThread()
        self._worker = RosBridgeWorker()
        self._worker.moveToThread(self._thread)

        # Connect worker signals to our signals
        self._worker.joint_state_received.connect(self._on_joint_state)
        self._worker.mode_changed.connect(self._on_mode_changed)
        self._worker.trigger_state_changed.connect(self.trigger_state_changed)
        self._worker.connection_status_changed.connect(self._on_connection_changed)
        self._worker.status_message.connect(self.status_message)
        self._worker.error_message.connect(self.error_message)

        # Start worker when thread starts
        self._thread.started.connect(self._worker.start_ros)

        self._thread.start()

    def stop(self):
        """Stop the ROS bridge."""
        if self._worker:
            self._worker.stop()
        if self._thread:
            self._thread.quit()
            self._thread.wait(3000)
            self._thread = None
            self._worker = None

    def _on_joint_state(self, msg):
        """Handle joint state and update joint names."""
        self._joint_names = list(msg.name)
        self.joint_state_received.emit(msg)

    def _on_mode_changed(self, mode: str):
        """Handle mode change."""
        self._current_mode = mode
        self.mode_changed.emit(mode)

    def _on_connection_changed(self, connected: bool):
        """Handle connection status change."""
        self._connected = connected
        self.connection_status_changed.emit(connected)

    @property
    def is_connected(self) -> bool:
        """Check if connected to driver."""
        return self._connected

    @property
    def current_mode(self) -> str:
        """Get current mode."""
        return self._current_mode

    @property
    def joint_names(self) -> List[str]:
        """Get list of known joint names."""
        return self._joint_names

    # === Control methods (thread-safe) ===

    def set_mode(self, mode: str):
        """Set motor control mode."""
        if self._worker:
            self._worker.set_mode(mode)

    def set_enabled(self, enabled: bool):
        """Enable/disable motors."""
        if self._worker:
            self._worker.set_enabled(enabled)

    def emergency_stop(self):
        """Emergency stop."""
        if self._worker:
            self._worker.emergency_stop()

    def set_zero(self):
        """Set current position as zero."""
        if self._worker:
            self._worker.set_zero()

    def go_to_zero(self):
        """Go to zero position."""
        if self._worker:
            if self._joint_names:
                self._worker.go_to_zero(self._joint_names)
            else:
                self.error_message.emit("No joints detected - is driver connected?")

    def send_joint_command(self, msg: JointState):
        """Send a joint command."""
        if self._worker:
            self._worker.send_joint_command(msg)

    def set_trigger_config(self, config: PlaybackTriggerConfig):
        """Set trigger configuration."""
        if self._worker:
            self._worker.set_trigger_config(config)

    def clear_trigger_config(self):
        """Clear trigger configuration."""
        if self._worker:
            self._worker.clear_trigger_config()

    def get_joint_ctrl_publisher(self):
        """Get the joint control publisher for direct publishing (bypasses Qt signals)."""
        if self._worker:
            return self._worker.get_joint_ctrl_publisher()
        return None
