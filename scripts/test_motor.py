#!/usr/bin/env python3
"""
Motor Control Test Script

Test the enhanced motor control system based on Freenove implementation.
This script allows manual testing of motor functions and hardware connectivity.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys


class MotorTestNode(Node):
    """Test node for motor control system."""
    
    def __init__(self):
        super().__init__('motor_test')
        
        # Publisher for cmd_vel commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Motor test node started')
        self.get_logger().info('Use the following commands:')
        self.get_logger().info('  w - Forward')
        self.get_logger().info('  s - Backward') 
        self.get_logger().info('  a - Turn left')
        self.get_logger().info('  d - Turn right')
        self.get_logger().info('  q - Stop')
        self.get_logger().info('  x - Exit')
    
    def send_velocity(self, linear: float, angular: float):
        """Send velocity command to motor node."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Sent: linear={linear:.2f}, angular={angular:.2f}')
    
    def run_interactive_test(self):
        """Run interactive motor test."""
        try:
            while True:
                cmd = input("Enter command (w/a/s/d/q/x): ").strip().lower()
                
                if cmd == 'w':
                    self.send_velocity(0.5, 0.0)  # Forward
                elif cmd == 's':
                    self.send_velocity(-0.5, 0.0)  # Backward
                elif cmd == 'a':
                    self.send_velocity(0.0, 0.5)  # Turn left
                elif cmd == 'd':
                    self.send_velocity(0.0, -0.5)  # Turn right
                elif cmd == 'q':
                    self.send_velocity(0.0, 0.0)  # Stop
                elif cmd == 'x':
                    self.send_velocity(0.0, 0.0)  # Stop and exit
                    break
                else:
                    print("Invalid command!")
                
                time.sleep(0.1)  # Small delay
                
        except KeyboardInterrupt:
            pass
        finally:
            # Ensure motors are stopped
            self.send_velocity(0.0, 0.0)
            self.get_logger().info('Test completed - motors stopped')
    
    def run_automatic_test(self):
        """Run automatic test sequence."""
        test_sequence = [
            (0.3, 0.0, "Forward"),
            (0.0, 0.0, "Stop"),
            (-0.3, 0.0, "Backward"), 
            (0.0, 0.0, "Stop"),
            (0.0, 0.5, "Turn left"),
            (0.0, 0.0, "Stop"),
            (0.0, -0.5, "Turn right"),
            (0.0, 0.0, "Stop"),
        ]
        
        for linear, angular, description in test_sequence:
            self.get_logger().info(f'Testing: {description}')
            self.send_velocity(linear, angular)
            time.sleep(2.0)  # Run each command for 2 seconds
        
        self.get_logger().info('Automatic test completed')


def main(args=None):
    """Main entry point for motor test."""
    rclpy.init(args=args)
    
    test_node = MotorTestNode()
    
    # Check command line arguments
    if len(sys.argv) > 1 and sys.argv[1] == '--auto':
        # Run automatic test
        test_node.run_automatic_test()
    else:
        # Run interactive test
        test_node.run_interactive_test()
    
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()