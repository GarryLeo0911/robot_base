"""
Robot Base Bringup Launch File

This launch file starts the core robot base functionality:
- Motor control node
- Optional teleop node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'max_duty',
            default_value='1000',
            description='Maximum duty cycle for motor control'
        ),
        DeclareLaunchArgument(
            'cmd_vel_timeout',
            default_value='1.0',
            description='Timeout for cmd_vel safety stop (seconds)'
        ),
        DeclareLaunchArgument(
            'enable_teleop',
            default_value='false',
            description='Enable keyboard teleop node'
        ),
        DeclareLaunchArgument(
            'linear_scale',
            default_value='1.0',
            description='Linear velocity scale for teleop'
        ),
        DeclareLaunchArgument(
            'angular_scale',
            default_value='1.0',
            description='Angular velocity scale for teleop'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Namespace for robot nodes'
        )
    ]

    return LaunchDescription(declared_arguments + [
        # Motor control node
        Node(
            package='robot_base',
            executable='motor_node',
            name='motor_node',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[{
                'max_duty': LaunchConfiguration('max_duty'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
                'publish_status': True,
                'status_frequency': 10.0
            }]
        ),
        
        # Optional teleop node
        Node(
            package='robot_base',
            executable='teleop_wasd',
            name='teleop_wasd',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[{
                'linear_scale': LaunchConfiguration('linear_scale'),
                'angular_scale': LaunchConfiguration('angular_scale'),
                'publish_rate': 10.0
            }],
            condition=IfCondition(LaunchConfiguration('enable_teleop'))
        )
    ])