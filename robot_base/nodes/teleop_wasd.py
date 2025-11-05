#!/usr/bin/env python3
"""
Teleop WASD Node - Keyboard Teleoperation Node

This node provides keyboard-based robot control using WASD keys.
It publishes Twist messages to cmd_vel topic for robot navigation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import threading
import time


class TeleopWASDNode(Node):
    """ROS 2 node for keyboard teleoperation using WASD keys."""
    
    def __init__(self):
        super().__init__('teleop_wasd')
        
        # Declare parameters
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        # Get parameters
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Initialize publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Initialize movement state
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_cmd_vel)
        
        # Terminal settings for keyboard input
        self.settings = None
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
        
        # Start keyboard input thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.print_instructions()
        self.get_logger().info('Teleop WASD node started')
        self.get_logger().info(f'Linear scale: {self.linear_scale}')
        self.get_logger().info(f'Angular scale: {self.angular_scale}')
        self.get_logger().info(f'Publish rate: {publish_rate} Hz')
    
    def print_instructions(self):
        """Print keyboard control instructions."""
        instructions = """
        Robot Teleoperation Controls:
        
        Movement:
        --------
        W : Move Forward
        S : Move Backward
        A : Turn Left
        D : Turn Right
        Q : Stop All Movement
        
        Speed Control:
        -------------
        R : Increase Linear Speed
        F : Decrease Linear Speed
        T : Increase Angular Speed
        G : Decrease Angular Speed
        
        Other:
        -----
        SPACE : Emergency Stop
        ESC   : Quit
        
        Current Speeds:
        - Linear Scale:  {:.2f}
        - Angular Scale: {:.2f}
        
        Use Ctrl+C to exit
        """.format(self.linear_scale, self.angular_scale)
        
        print(instructions)
    
    def get_key(self):
        """Get a single key press from terminal."""
        if not sys.stdin.isatty():
            return ''
        
        tty.setraw(sys.stdin.fileno())
        
        # Check if input is available
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
        else:
            key = ''
        
        if self.settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
        return key
    
    def keyboard_loop(self):
        """Main keyboard input loop."""
        while self.running and rclpy.ok():
            try:
                key = self.get_key()
                
                if key:
                    self.process_key(key)
                
                time.sleep(0.05)  # Small delay to prevent high CPU usage
                
            except Exception as e:
                self.get_logger().error(f"Keyboard input error: {e}")
                break
    
    def process_key(self, key):
        """
        Process keyboard input and update movement commands.
        
        Args:
            key: Character pressed
        """
        key = key.lower()
        
        # Movement keys
        if key == 'w':
            self.linear_x = self.linear_scale
            self.angular_z = 0.0
            self.get_logger().info("Moving Forward")
            
        elif key == 's':
            self.linear_x = -self.linear_scale
            self.angular_z = 0.0
            self.get_logger().info("Moving Backward")
            
        elif key == 'a':
            self.linear_x = 0.0
            self.angular_z = self.angular_scale
            self.get_logger().info("Turning Left")
            
        elif key == 'd':
            self.linear_x = 0.0
            self.angular_z = -self.angular_scale
            self.get_logger().info("Turning Right")
            
        elif key == 'q' or key == ' ':
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.get_logger().info("Stopped")
            
        # Speed adjustment keys
        elif key == 'r':
            self.linear_scale = min(2.0, self.linear_scale + 0.1)
            self.get_logger().info(f"Linear scale increased to {self.linear_scale:.2f}")
            
        elif key == 'f':
            self.linear_scale = max(0.1, self.linear_scale - 0.1)
            self.get_logger().info(f"Linear scale decreased to {self.linear_scale:.2f}")
            
        elif key == 't':
            self.angular_scale = min(2.0, self.angular_scale + 0.1)
            self.get_logger().info(f"Angular scale increased to {self.angular_scale:.2f}")
            
        elif key == 'g':
            self.angular_scale = max(0.1, self.angular_scale - 0.1)
            self.get_logger().info(f"Angular scale decreased to {self.angular_scale:.2f}")
            
        # Exit key
        elif key == '\x1b':  # ESC key
            self.get_logger().info("ESC pressed - shutting down")
            self.running = False
            rclpy.shutdown()
            
        elif key == '\x03':  # Ctrl+C
            self.running = False
            rclpy.shutdown()
    
    def publish_cmd_vel(self):
        """Publish current velocity command."""
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_z
        
        self.cmd_vel_pub.publish(msg)
    
    def destroy_node(self):
        """Clean shutdown."""
        self.running = False
        
        # Restore terminal settings
        if self.settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
        # Send stop command
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        self.get_logger().info("Teleop WASD node shutting down")
        super().destroy_node()


def main(args=None):
    """Main entry point for teleop WASD node."""
    rclpy.init(args=args)
    
    try:
        teleop_node = TeleopWASDNode()
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Teleop node error: {e}")
    finally:
        if 'teleop_node' in locals():
            teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()