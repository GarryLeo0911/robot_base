from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Robot base frame id'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odom frame id'),
        DeclareLaunchArgument('map_frame', default_value='map', description='Map frame id'),

        # Topics published by the robot (adjust to match your DepthAI driver namespace)
        DeclareLaunchArgument('rgb_topic', default_value='/oak/rgb/image_raw', description='RGB image topic from robot'),
        DeclareLaunchArgument('depth_topic', default_value='/oak/depth/image_raw', description='Depth image topic from robot'),
        DeclareLaunchArgument('camera_info_topic', default_value='/oak/rgb/camera_info', description='Camera info topic from robot'),

        # Prefer compressed transport over Wi‑Fi
        DeclareLaunchArgument('image_transport', default_value='compressed', description='image_transport hint for subscriptions'),

        # RTAB-Map core (subscribes RGBD)
        # In ROS 2 Jazzy, the executable is provided by package 'rtabmap_slam'.
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
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
                # If you have odometry coming from elsewhere, remap ('odom', '/your/odom') here.
            ],
        ),

        # RTAB-Map visualization on the laptop
        # In ROS 2 Jazzy, the viz executable is in package 'rtabmap_viz' as 'rtabmap_viz'.
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmapviz',
            output='screen',
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
    ])
