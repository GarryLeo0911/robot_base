from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')

    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    image_transport = LaunchConfiguration('image_transport')
    use_scan_cloud = LaunchConfiguration('use_scan_cloud')
    cloud_topic = LaunchConfiguration('cloud_topic')
    start_icp_odom = LaunchConfiguration('start_icp_odom')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Robot base frame id'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odom frame id'),
        DeclareLaunchArgument('map_frame', default_value='map', description='Map frame id'),

        # Topics published by the robot (adjust to match your DepthAI driver namespace)
        DeclareLaunchArgument('rgb_topic', default_value='/oak/rgb/image_raw', description='RGB image topic from robot'),
        DeclareLaunchArgument('depth_topic', default_value='/oak/depth/image_raw', description='Depth image topic from robot'),
        DeclareLaunchArgument('camera_info_topic', default_value='/oak/rgb/camera_info', description='Camera info topic from robot'),

        # LiDAR-like point cloud (from OAK-D depth). If you want to run without RGB,
        # enable scan cloud mode and provide the cloud topic.
        DeclareLaunchArgument('use_scan_cloud', default_value='false', description='Use 3D scan cloud instead of RGB-D'),
        DeclareLaunchArgument('cloud_topic', default_value='/oak/stereo/points', description='PointCloud2 topic (from OAK-D or depth_image_proc)'),
        DeclareLaunchArgument('start_icp_odom', default_value='true', description='Start ICP odometry on the cloud'),

        # Prefer compressed transport over Wi‑Fi
        DeclareLaunchArgument('image_transport', default_value='compressed', description='image_transport hint for subscriptions'),

        # RTAB-Map core (RGB-D mode)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            condition=UnlessCondition(use_scan_cloud),
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': base_frame,
                'odom_frame_id': odom_frame,
                'map_frame_id': map_frame,
                'subscribe_depth': True,
                'approx_sync': True,
                'queue_size': 10,
                'image_transport': image_transport,
            }],
            remappings=[
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),
            ],
        ),

        # RTAB-Map core (Scan Cloud mode - LiDAR-like)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap_scan',
            output='screen',
            condition=IfCondition(use_scan_cloud),
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

        # ICP Odometry from 3D cloud (optional, when using scan cloud)
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            condition=IfCondition(PythonExpression([use_scan_cloud, ' and ', start_icp_odom])),
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': base_frame,
                'odom_frame_id': odom_frame,
            }],
            remappings=[
                ('cloud', cloud_topic),
            ],
        ),

        # RTAB-Map visualization on the laptop
        # In ROS 2 Jazzy, the viz executable is in package 'rtabmap_viz' as 'rtabmap_viz'.
        # RGB-D viz (default)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmapviz',
            output='screen',
            condition=UnlessCondition(use_scan_cloud),
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': base_frame,
                'image_transport': image_transport,
            }],
            remappings=[
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),
            ],
        ),

        # Scan-cloud viz
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmapviz_scan',
            output='screen',
            condition=IfCondition(use_scan_cloud),
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': base_frame,
            }],
            remappings=[
                ('scan_cloud', cloud_topic),
            ],
        ),
    ])
