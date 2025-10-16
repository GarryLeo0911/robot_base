from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_freenove_4wd',
            executable='motor_node',
            name='motor_node',
            output='screen',
            parameters=[{'max_duty': 2000}],
        ),
        Node(
            package='ros2_freenove_4wd',
            executable='teleop_wasd',
            name='teleop_wasd',
            output='screen',
        ),
        Node(
            package='ros2_freenove_4wd',
            executable='led_node',
            name='led_node',
            output='screen',
            parameters=[{'count': 8, 'bus': 0, 'device': 0}],
        ),
        Node(
            package='ros2_freenove_4wd',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen',
            parameters=[{'trigger_pin': 27, 'echo_pin': 22, 'frame_id': 'ultrasonic', 'rate_hz': 10.0}],
        ),
        Node(
            package='ros2_freenove_4wd',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{'width': 640, 'height': 480, 'fps': 15, 'frame_id': 'camera'}],
        ),
    ])
