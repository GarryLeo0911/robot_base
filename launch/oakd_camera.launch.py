from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Integrate the oakd_pcloud camera driver by including its launch file.

    This is a thin wrapper so you can run:
        ros2 launch ros2_freenove_4wd oakd_camera.launch.py

    and override which launch file inside oakd_pcloud to use via camera_launch.
    """

    camera_launch = LaunchConfiguration('camera_launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_launch',
            # Default is a best guess; override as needed with the actual file name
            default_value='oakd_pcloud.launch.py',
            description='Launch file inside oakd_pcloud/launch to start the camera pipeline',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('oakd_pcloud'), '/launch/', camera_launch
            ]),
            launch_arguments={
                # Pass through additional oakd_pcloud-specific args here if needed
            }.items(),
        ),
    ])

