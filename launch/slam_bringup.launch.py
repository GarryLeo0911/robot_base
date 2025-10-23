from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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

    # DepthAI + RTAB-Map package/launch (parameterized)
    depthai_pkg = LaunchConfiguration('depthai_pkg')
    depthai_launch = LaunchConfiguration('depthai_launch')
    # Camera-only include (for running sensors on the robot and RTAB-Map remotely)
    camera_pkg = LaunchConfiguration('camera_pkg')
    camera_launch = LaunchConfiguration('camera_launch')
    only_camera = LaunchConfiguration('only_camera')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('depthai_pkg', default_value='depthai_ros_driver', description='Package providing rtabmap.launch.py'),
        DeclareLaunchArgument('depthai_launch', default_value='rtabmap.launch.py', description='DepthAI RTAB-Map launch file name'),
        DeclareLaunchArgument('camera_pkg', default_value='depthai_ros_driver', description='Package providing camera-only launch'),
        # Default to a depth-capable camera launch
        DeclareLaunchArgument('camera_launch', default_value='rtabmap.launch.py', description='Camera-only launch file name'),
        DeclareLaunchArgument('only_camera', default_value='false', description='If true, include camera-only launch (Pi). If false, include RTAB-Map launch (laptop or single-machine).'),
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

        # DepthAI + RTAB-Map pipeline (local mapping) unless only_camera is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare(depthai_pkg), '/launch/', depthai_launch
            ]),
            condition=UnlessCondition(only_camera),
            # If your rtabmap.launch.py exposes args (e.g., use_imu, base_frame, odom_frame),
            # you can pass them here via launch_arguments.
            launch_arguments={
                # Examples (uncomment and adjust to match your installed launch):
                # 'base_frame': 'base_link',
                # 'odom_frame': 'odom',
                # 'map_frame': 'map',
                # 'publish_tf': 'true',
                # 'use_imu': 'true',
                # 'camera_model': 'OAK-D',
            }.items(),
        ),

        # Camera-only pipeline (for running on the robot) if only_camera is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare(camera_pkg), '/launch/', camera_launch
            ]),
            condition=IfCondition(only_camera),
            launch_arguments={
                # Provide your camera config here if needed.
            }.items(),
        ),
    ])
