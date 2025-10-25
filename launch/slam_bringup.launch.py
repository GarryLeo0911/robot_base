from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Common launch args
    use_sim_time = LaunchConfiguration('use_sim_time')
    cam_parent_frame = LaunchConfiguration('cam_parent_frame')
    cam_child_frame = LaunchConfiguration('cam_child_frame')
    cam_x = LaunchConfiguration('cam_x')
    cam_y = LaunchConfiguration('cam_y')
    cam_z = LaunchConfiguration('cam_z')
    cam_roll = LaunchConfiguration('cam_roll')
    cam_pitch = LaunchConfiguration('cam_pitch')
    cam_yaw = LaunchConfiguration('cam_yaw')

    pkg_share = get_package_share_directory('ros2_freenove_4wd')
    urdf_path = os.path.join(pkg_share, 'urdf', 'freenove_4wd.urdf')

    # Camera-only include (run sensors on robot, do all processing on laptop)
    camera_pkg = LaunchConfiguration('camera_pkg')
    camera_launch = LaunchConfiguration('camera_launch')
    include_camera = LaunchConfiguration('include_camera')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('camera_pkg', default_value='depthai_ros_driver', description='Package providing camera-only launch (e.g., depthai_ros_driver)'),
        DeclareLaunchArgument('camera_launch', default_value='camera.launch.py', description='Camera-only launch file name under camera_pkg'),
        DeclareLaunchArgument('include_camera', default_value='true', description='If true, include camera-only launch on robot. No SLAM runs here.'),
        DeclareLaunchArgument('cam_parent_frame', default_value='base_link', description='Parent frame for camera'),
        DeclareLaunchArgument('cam_child_frame', default_value='oak-d_frame', description='Camera frame id published by DepthAI'),
        DeclareLaunchArgument('cam_x', default_value='0.08', description='Camera X offset (m) from parent'),
        DeclareLaunchArgument('cam_y', default_value='0.0', description='Camera Y offset (m) from parent'),
        DeclareLaunchArgument('cam_z', default_value='0.08', description='Camera Z offset (m) from parent'),
        DeclareLaunchArgument('cam_roll', default_value='0', description='Camera roll (rad)'),
        DeclareLaunchArgument('cam_pitch', default_value='0', description='Camera pitch (rad)'),
        DeclareLaunchArgument('cam_yaw', default_value='0', description='Camera yaw (rad)'),

        # Robot state publisher (publishes base_footprint, base_link, etc.)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': open(urdf_path, 'r').read(),
            }],
        ),

        # Static transform from base_link -> OAK-D frame (or your chosen camera frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_cam',
            output='screen',
            arguments=[
                cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw,
                cam_parent_frame, cam_child_frame,
            ],
        ),

        # Motor control (listens to cmd_vel)
        Node(
            package='ros2_freenove_4wd',
            executable='motor_node',
            name='motor_node',
            output='screen',
            parameters=[{'max_duty': 2000}],
        ),

        # Keyboard teleop (publishes cmd_vel)
        Node(
            package='ros2_freenove_4wd',
            executable='teleop_wasd',
            name='teleop_wasd',
            output='screen',
        ),

        # Camera-only pipeline (for running on the robot). No SLAM or heavy processing here.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare(camera_pkg), '/launch/', camera_launch
            ]),
            condition=IfCondition(include_camera),
            launch_arguments={
                # Provide your camera config here if needed.
            }.items(),
        ),
    ])
