from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    use_composition = LaunchConfiguration('use_composition')

    pkg_share = get_package_share_directory('ros2_freenove_4wd')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_urdf = os.path.join(pkg_share, 'urdf', 'freenove_4wd.urdf')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock'),
        DeclareLaunchArgument('map', default_value='', description='Full path to map yaml to load'),
        DeclareLaunchArgument('params_file', default_value=default_params, description='Nav2 params YAML'),
        DeclareLaunchArgument('slam', default_value='False', description='Run SLAM instead of AMCL'),
        DeclareLaunchArgument('use_composition', default_value='True', description='Launch Nav2 in a composed container'),

        # Robot description publisher (all joints are fixed)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': open(default_urdf, 'r').read(),
            }],
        ),

        # Open-loop odometry integrator (replace with real odom if available)
        Node(
            package='ros2_freenove_4wd',
            executable='odom_integrator_node',
            name='odom_integrator_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'odom_frame': 'odom', 'base_frame': 'base_link', 'publish_rate_hz': 50.0}],
        ),

        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'map': map_yaml,
                'slam': slam,
                'use_composition': use_composition,
            }.items(),
        ),
    ])
