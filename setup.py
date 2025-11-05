from setuptools import setup
from glob import glob
import os

package_name = 'autoslam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.nodes', f'{package_name}.hw'],
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
    description='AutoSLAM - Basic robotic vehicle control with ROS 2: motor control and teleop.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = autoslam.nodes.motor_node:main',
            'teleop_wasd = autoslam.nodes.teleop_wasd:main',
        ],
    },
)
