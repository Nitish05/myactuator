#!/usr/bin/env python3
"""Launch file for MyActuator Python Driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to driver config YAML file'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Joint state publish rate in Hz'
    )
    
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='position',
        description='Control mode: position, velocity, torque, or readonly'
    )
    
    motor_ids_arg = DeclareLaunchArgument(
        'motor_ids',
        default_value='[1]',
        description='List of motor CAN IDs'
    )
    
    joint_names_arg = DeclareLaunchArgument(
        'joint_names',
        default_value='[joint1]',
        description='List of joint names'
    )
    
    torque_constants_arg = DeclareLaunchArgument(
        'torque_constants',
        default_value='[0.21]',
        description='List of torque constants (Nm/A)'
    )
    
    # Driver node
    driver_node = Node(
        package='myactuator_python_driver',
        executable='driver_node',
        name='motor_driver',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'can_interface': LaunchConfiguration('can_interface'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'control_mode': LaunchConfiguration('control_mode'),
            'motor_ids': LaunchConfiguration('motor_ids'),
            'joint_names': LaunchConfiguration('joint_names'),
            'torque_constants': LaunchConfiguration('torque_constants'),
        }],
    )
    
    return LaunchDescription([
        config_file_arg,
        can_interface_arg,
        publish_rate_arg,
        control_mode_arg,
        motor_ids_arg,
        joint_names_arg,
        torque_constants_arg,
        driver_node,
    ])
