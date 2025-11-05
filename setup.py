from setuptools import setup
from glob import glob
import os

package_name = 'robot_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=['robot_base', 'robot_base.nodes', 'robot_base.hw'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        # Install all launch files automatically
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (f'share/{package_name}', ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Robot Base - Basic robotic vehicle motor control and teleop functionality for AutoSLAM system.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = robot_base.nodes.motor_node:main',
            'teleop_wasd = robot_base.nodes.teleop_wasd:main',
        ],
    },
)
