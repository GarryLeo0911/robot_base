# Robot Base Package

Robot Base provides the fundamental motor control and teleoperation functionality for the AutoSLAM robotic system.

## Overview

This package contains:
- **Motor Control**: Hardware abstraction for robot motor control
- **Teleoperation**: Keyboard-based robot control interface
- **Hardware Support**: PCA9685 PWM driver interface for servo/motor control

## Features

### Motor Control Node (`motor_node`)
- Subscribes to `cmd_vel` messages for robot movement commands
- Provides safety timeout to stop motors if no commands received
- Publishes motor status information
- Hardware abstraction for different motor configurations

### Teleop WASD Node (`teleop_wasd`)
- Keyboard-based robot control using WASD keys
- Real-time speed adjustment
- Emergency stop functionality
- Intuitive control interface

## Usage

### Basic Motor Control
```bash
# Start motor control node
ros2 run robot_base motor_node

# Send movement commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

### Keyboard Teleoperation
```bash
# Start teleop node
ros2 run robot_base teleop_wasd

# Use WASD keys to control robot
# W/S: Forward/Backward
# A/D: Left/Right turns
# Q/Space: Stop
```

### Complete Robot Bringup
```bash
# Start motor control with teleop
ros2 launch robot_base bringup.launch.py enable_teleop:=true

# Start motor control only
ros2 launch robot_base bringup.launch.py
```

## Parameters

### Motor Node Parameters
- `max_duty`: Maximum duty cycle for motor control (default: 1000)
- `cmd_vel_timeout`: Safety timeout in seconds (default: 1.0)
- `publish_status`: Enable status publishing (default: true)
- `status_frequency`: Status publish rate in Hz (default: 10.0)

### Teleop Node Parameters
- `linear_scale`: Linear velocity scaling factor (default: 1.0)
- `angular_scale`: Angular velocity scaling factor (default: 1.0)
- `publish_rate`: Command publish rate in Hz (default: 10.0)

## Topics

### Subscribed Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot movement

### Published Topics
- `/motor_status` (std_msgs/String): Motor status information (JSON format)

## Integration

This package is designed to work with:
- **autoslam**: Main SLAM integration package
- **oakd_driver**: Camera and sensor data
- **map_builder**: RTAB-Map SLAM processing

## Architecture Support

### Distributed Processing
Robot side runs motor control, laptop handles teleoperation and SLAM processing.

### Edge Computing  
Robot runs motor control and SLAM processing, laptop provides teleoperation only.

## Hardware Requirements

- Robot platform with motor control capability
- Optional: PCA9685 PWM driver for servo control
- I2C interface for hardware communication

## Dependencies

- rclpy
- geometry_msgs
- std_msgs
- Python 3.8+

## License

MIT License