#!/usr/bin/env python3
"""
Simple ROS 2 demo: spin a joint at constant velocity and display speed + effort.

Requires the motor driver node to be running:
    ros2 launch myactuator_python_driver driver.launch.py

Usage:
    ros2 run myactuator_python_driver velocity_demo                        # defaults: joint1, 90 deg/s
    ros2 run myactuator_python_driver velocity_demo --ros-args -p speed:=180.0
    ros2 run myactuator_python_driver velocity_demo --ros-args -p joint:=joint2
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class VelocityDemo(Node):
    def __init__(self):
        super().__init__('velocity_demo')

        self.declare_parameter('joint', 'base_joint')
        self.declare_parameter('speed', 90.0)  # deg/s

        self.joint_name = self.get_parameter('joint').get_parameter_value().string_value
        speed_dps = self.get_parameter('speed').get_parameter_value().double_value
        self.target_vel_rad = speed_dps * math.pi / 180.0

        # Publisher: velocity command to the driver
        self.cmd_pub = self.create_publisher(JointState, '/joint_state_ctrl', 10)

        # Publisher: set driver to velocity mode
        self.mode_pub = self.create_publisher(String, '/motor_driver/set_mode', 10)

        # Subscriber: feedback from the driver
        self.create_subscription(JointState, '/joint_states', self._feedback_cb, 10)

        # Wait a moment for the driver to be ready, then set velocity mode
        self.create_timer(1.0, self._set_mode_once)
        self._mode_set = False

        # Send velocity commands at 50 Hz
        self.create_timer(0.02, self._send_cmd)

        self.get_logger().info(
            f'Velocity demo: {self.joint_name} @ {speed_dps:.1f} deg/s '
            f'({self.target_vel_rad:.3f} rad/s)')
        self.get_logger().info('Waiting for driver...')

    def _set_mode_once(self, *_):
        if self._mode_set:
            return
        msg = String()
        msg.data = 'velocity'
        self.mode_pub.publish(msg)
        self._mode_set = True
        self.get_logger().info('Set driver to velocity mode')

    def _send_cmd(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [0.0]
        msg.velocity = [self.target_vel_rad]
        msg.effort = [0.0]
        self.cmd_pub.publish(msg)

    def _feedback_cb(self, msg: JointState):
        if self.joint_name not in msg.name:
            return
        idx = msg.name.index(self.joint_name)

        vel_rad = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
        effort = msg.effort[idx] if idx < len(msg.effort) else 0.0
        vel_deg = vel_rad * 180.0 / math.pi

        print(
            f'\rSpeed: {vel_deg:7.1f} deg/s ({vel_rad:6.3f} rad/s)  |  '
            f'Effort: {effort:6.3f} Nm   ', end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the motor by sending zero velocity before exiting
        stop_msg = JointState()
        stop_msg.name = [node.joint_name]
        stop_msg.position = [0.0]
        stop_msg.velocity = [0.0]
        stop_msg.effort = [0.0]
        node.cmd_pub.publish(stop_msg)
        node.get_logger().info('Stopping motor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
