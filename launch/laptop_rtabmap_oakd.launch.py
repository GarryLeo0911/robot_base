from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Run RTAB-Map locally using a 3D scan cloud from oakd_pcloud.

    Example:
        ros2 launch ros2_freenove_4wd laptop_rtabmap_oakd.launch.py \
            cloud_topic:=/oak/stereo/points use_sim_time:=false

    Adjust cloud_topic to match oakd_pcloud's published PointCloud2.
    """

    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    cloud_topic = LaunchConfiguration('cloud_topic')
    start_icp_odom = LaunchConfiguration('start_icp_odom')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Robot base frame id'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odom frame id'),
        DeclareLaunchArgument('map_frame', default_value='map', description='Map frame id'),
        DeclareLaunchArgument('cloud_topic', default_value='/oak/stereo/points', description='PointCloud2 topic from oakd_pcloud'),
        DeclareLaunchArgument('start_icp_odom', default_value='true', description='Start ICP odometry on the cloud'),

        # RTAB-Map core in scan-cloud mode
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap_scan',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': base_frame,
                'odom_frame_id': odom_frame,
                'map_frame_id': map_frame,
                'subscribe_scan_cloud': True,
                'subscribe_depth': False,
                'queue_size': 10,
            }],
            remappings=[
                ('scan_cloud', cloud_topic),
            ],
        ),

        # Optional: ICP odometry from 3D cloud
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': base_frame,
                'odom_frame_id': odom_frame,
            }],
            remappings=[
                ('cloud', cloud_topic),
            ],
            # Users can disable by setting start_icp_odom:=false at launch
            condition=IfCondition(start_icp_odom),
        ),
    ])
