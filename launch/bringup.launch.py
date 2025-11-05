from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Motor control (listens to cmd_vel)
        Node(
            package='autoslam',
            executable='motor_node',
            name='motor_node',
            output='screen',
            parameters=[{'max_duty': 1000}],
        ),
        
        # Keyboard teleop (publishes cmd_vel)
        Node(
            package='autoslam',
            executable='teleop_wasd',
            name='teleop_wasd',
            output='screen',
        ),
    ])
