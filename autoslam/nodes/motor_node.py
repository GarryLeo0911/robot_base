import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ..hw.motor import Ordinary_Car


class MotorNode(Node):
    """ROS 2 node that converts Twist commands to motor PWM outputs.

    Subscribes to `cmd_vel` and maps linear.x and angular.z to left/right
    wheel duties, then forwards them to the PCA9685 motor driver via
    `Ordinary_Car`.
    """

    # Initialize node, parameters, hardware, and subscription.
    def __init__(self):
        """Initialize the node, parameters, hardware, and subscription."""
        super().__init__('motor_node')
        self.declare_parameter('max_duty', 1000)
        self.max_duty = int(self.get_parameter('max_duty').get_parameter_value().integer_value)
        self.car = Ordinary_Car()
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.get_logger().info(f'Motor node started. max_duty={self.max_duty}')

    # Handle incoming Twist at `cmd_vel`, compute wheel duties, drive motors.
    def cmd_vel_cb(self, msg: Twist):
        """Callback for `cmd_vel` messages.

        - Clamps `linear.x` and `angular.z` to [-1.0, 1.0].
        - Computes left/right duties using skid-steer mapping.
        - Sends four wheel duties (FL, BL, FR, BR) to the motor driver.

        Args:
            msg: Incoming geometry_msgs/Twist command.
        """
        lin = float(msg.linear.x)
        ang = float(msg.angular.z)
        lin = max(min(lin, 1.0), -1.0)
        ang = max(min(ang, 1.0), -1.0)

        left = lin * self.max_duty - ang * self.max_duty
        right = lin * self.max_duty + ang * self.max_duty

        # Map to 4 wheels: FL, BL, FR, BR
        fl = int(left)
        bl = int(left)
        fr = int(right)
        br = int(right)
        self.car.set_motor_model(fl, bl, fr, br)

    # Stop the motors safely and release hardware on shutdown.
    def destroy_node(self):
        """Stop motors safely and tear down hardware on shutdown."""
        try:
            self.car.set_motor_model(0, 0, 0, 0)
            self.car.close()
        except Exception:
            pass
        return super().destroy_node()


# Entry point to run the motor node as a console script.
def main(args=None):
    """Entry point for the motor node executable.

    Initializes rclpy, spins the node, and ensures clean shutdown on Ctrl+C.
    """
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
