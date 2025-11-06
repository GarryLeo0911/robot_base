#!/usr/bin/env python3
"""
Motor Node - ROS 2 Motor Control Node

This node subscribes to cmd_vel messages and controls robot motors.
It provides the interface between ROS 2 navigation commands and hardware motors.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import logging

from robot_base.hw.motor import Motor


class MotorNode(Node):
    """ROS 2 node for motor control."""
    
    def __init__(self):
        super().__init__('motor_node')
        
        # Configure logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Declare parameters with better defaults
        self.declare_parameter('max_duty', 4095)  # Full 12-bit range
        self.declare_parameter('cmd_vel_timeout', 2.0)  # Longer timeout for network delays
        self.declare_parameter('publish_status', True)
        self.declare_parameter('status_frequency', 10.0)  # Hz
        self.declare_parameter('i2c_address', 0x40)  # PCA9685 I2C address
        self.declare_parameter('pwm_frequency', 1000)  # Motor PWM frequency
        
        # Get parameters
        max_duty = self.get_parameter('max_duty').get_parameter_value().integer_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        self.publish_status = self.get_parameter('publish_status').get_parameter_value().bool_value
        status_frequency = self.get_parameter('status_frequency').get_parameter_value().double_value
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        pwm_frequency = self.get_parameter('pwm_frequency').get_parameter_value().integer_value
        
        # Initialize motor controller with enhanced parameters
        try:
            self.motor = Motor(
                max_duty=max_duty,
                i2c_address=i2c_address,
                pwm_freq=pwm_frequency
            )
            self.get_logger().info(f'Motor controller initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize motor controller: {e}')
            # Continue with a dummy motor for testing
            from robot_base.hw.motor import Motor
            self.motor = Motor(max_duty=max_duty)
        
        # Initialize ROS 2 interfaces
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Status publisher (optional)
        if self.publish_status:
            self.status_pub = self.create_publisher(
                String,
                'motor_status',
                10
            )
            
            # Status publishing timer
            status_timer_period = 1.0 / status_frequency
            self.status_timer = self.create_timer(
                status_timer_period,
                self.publish_status_callback
            )
        
        # Safety timer - stop motors if no cmd_vel received
        self.safety_timer = self.create_timer(
            self.cmd_vel_timeout,
            self.safety_stop_callback
        )
        
        self.last_cmd_vel_time = self.get_clock().now()
        
        self.get_logger().info(f'Motor node started with max_duty={max_duty}')
        self.get_logger().info(f'I2C address: 0x{i2c_address:02X}, PWM freq: {pwm_frequency}Hz')
        self.get_logger().info(f'Cmd_vel timeout: {self.cmd_vel_timeout}s')
        self.get_logger().info(f'Status publishing: {self.publish_status}')
    
    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Handle incoming cmd_vel messages with enhanced error handling.
        
        Args:
            msg: Twist message with linear and angular velocities
        """
        # Update last command time
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Validate input ranges
        if abs(linear_x) > 2.0 or abs(angular_z) > 2.0:
            self.get_logger().warn(
                f'Large velocity command received: linear={linear_x:.3f}, angular={angular_z:.3f}'
            )
        
        try:
            # Send to motor controller
            self.motor.set_velocity(linear_x, angular_z)
            
            self.get_logger().debug(
                f'Cmd_vel: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}'
            )
        except Exception as e:
            self.get_logger().error(f'Error setting motor velocity: {e}')
            # Emergency stop on error
            self.motor.stop()
    
    def safety_stop_callback(self) -> None:
        """Safety callback to stop motors if no recent cmd_vel received."""
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.cmd_vel_timeout:
            # Stop motors for safety
            self.motor.stop()
            self.get_logger().warn(
                f'No cmd_vel received for {time_since_last_cmd:.1f}s - stopping motors'
            )
    
    def publish_status_callback(self) -> None:
        """Publish motor status information."""
        if not self.publish_status:
            return
            
        status = self.motor.get_status()
        status_msg = String()
        status_msg.data = json.dumps(status)
        
        self.status_pub.publish(status_msg)
    
    def destroy_node(self) -> None:
        """Clean shutdown - stop motors and close hardware properly."""
        self.get_logger().info('Shutting down motor node - stopping motors')
        try:
            self.motor.stop()
            # Close hardware interface if available
            if hasattr(self.motor, 'close'):
                self.motor.close()
        except Exception as e:
            self.get_logger().error(f'Error during motor shutdown: {e}')
        
        super().destroy_node()


def main(args=None):
    """Main entry point for motor node."""
    rclpy.init(args=args)
    
    try:
        motor_node = MotorNode()
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error(f"Motor node error: {e}")
    finally:
        if 'motor_node' in locals():
            motor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()