from setuptools import setup

package_name = 'ros2_freenove_4wd'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.nodes', f'{package_name}.hw'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/bringup.launch.py']),
        (f'share/{package_name}', ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS 2 nodes for Freenove 4WD Smart Car: motors, LEDs, camera, ultrasonic.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = ros2_freenove_4wd.nodes.motor_node:main',
            'led_node = ros2_freenove_4wd.nodes.led_node:main',
            'ultrasonic_node = ros2_freenove_4wd.nodes.ultrasonic_node:main',
            'camera_node = ros2_freenove_4wd.nodes.camera_node:main',
            'teleop_wasd = ros2_freenove_4wd.nodes.teleop_wasd:main',
        ],
    },
)
