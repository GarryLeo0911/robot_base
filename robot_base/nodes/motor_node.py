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
        
        # Declare parameters
        self.declare_parameter('max_duty', 1000)
        self.declare_parameter('cmd_vel_timeout', 1.0)  # seconds
        self.declare_parameter('publish_status', True)
        self.declare_parameter('status_frequency', 10.0)  # Hz
        
        # Get parameters
        max_duty = self.get_parameter('max_duty').get_parameter_value().integer_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        self.publish_status = self.get_parameter('publish_status').get_parameter_value().bool_value
        status_frequency = self.get_parameter('status_frequency').get_parameter_value().double_value
        
        # Initialize motor controller
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
        self.get_logger().info(f'Cmd_vel timeout: {self.cmd_vel_timeout}s')
        self.get_logger().info(f'Status publishing: {self.publish_status}')
    
    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Handle incoming cmd_vel messages.
        
        Args:
            msg: Twist message with linear and angular velocities
        """
        # Update last command time
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Send to motor controller
        self.motor.set_velocity(linear_x, angular_z)
        
        self.get_logger().debug(
            f'Received cmd_vel: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}'
        )
    
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
        """Clean shutdown - stop motors."""
        self.get_logger().info('Shutting down motor node - stopping motors')
        self.motor.stop()
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