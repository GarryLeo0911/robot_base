import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from ..hw.ultrasonic import Ultrasonic


class UltrasonicNode(Node):
    # Initialize node, parameters, sensor, publisher, and timer.
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.declare_parameter('trigger_pin', 27)
        self.declare_parameter('echo_pin', 22)
        self.declare_parameter('frame_id', 'ultrasonic')
        self.declare_parameter('rate_hz', 10.0)
        trig = int(self.get_parameter('trigger_pin').get_parameter_value().integer_value)
        echo = int(self.get_parameter('echo_pin').get_parameter_value().integer_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        rate = float(self.get_parameter('rate_hz').get_parameter_value().double_value)
        self.sensor = Ultrasonic(trigger_pin=trig, echo_pin=echo)
        self.pub = self.create_publisher(Range, 'range', 10)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_cb)
        self.get_logger().info('Ultrasonic node started.')

    # Periodic timer callback: read distance and publish sensor_msgs/Range.
    def timer_cb(self):
        d = self.sensor.get_distance_cm()
        if d is None:
            return
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.5
        msg.min_range = 0.02
        msg.max_range = 3.0
        msg.range = float(d) / 100.0
        self.pub.publish(msg)

    # Close ultrasonic sensor on shutdown.
    def destroy_node(self):
        try:
            self.sensor.close()
        except Exception:
            pass
        return super().destroy_node()


# Entry point to run the ultrasonic node as a console script.
def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
