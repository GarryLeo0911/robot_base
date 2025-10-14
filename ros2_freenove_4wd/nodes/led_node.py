from typing import Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from ..hw.spi_ledpixel import Freenove_SPI_LedPixel


class LEDNode(Node):
    # Initialize node, parameters, LED driver, and subscription.
    def __init__(self):
        super().__init__('led_node')
        self.declare_parameter('count', 8)
        self.declare_parameter('bus', 0)
        self.declare_parameter('device', 0)
        count = int(self.get_parameter('count').get_parameter_value().integer_value)
        bus = int(self.get_parameter('bus').get_parameter_value().integer_value)
        device = int(self.get_parameter('device').get_parameter_value().integer_value)
        self.leds: Optional[Freenove_SPI_LedPixel] = None
        try:
            self.leds = Freenove_SPI_LedPixel(count=count, bright=255, sequence='GRB', bus=bus, device=device)
            if self.leds.check_spi_state() == 0:
                self.get_logger().warn('SPI not initialized; LED control disabled')
        except Exception as e:
            self.get_logger().error(f'Failed to init LED strip: {e}')
        self.sub = self.create_subscription(ColorRGBA, 'led_color', self.cb, 10)
        self.get_logger().info('LED node started.')

    # Apply received ColorRGBA to all LEDs (0..1 scaled to 0..255).
    def cb(self, msg: ColorRGBA):
        if not self.leds or self.leds.check_spi_state() == 0:
            return
        r = max(0, min(int(round(msg.r * 255.0)), 255))
        g = max(0, min(int(round(msg.g * 255.0)), 255))
        b = max(0, min(int(round(msg.b * 255.0)), 255))
        self.leds.set_all_led_color(r, g, b)

    # Turn off LEDs and close SPI on shutdown.
    def destroy_node(self):
        try:
            if self.leds:
                self.leds.set_all_led_color(0, 0, 0)
                self.leds.led_close()
        except Exception:
            pass
        return super().destroy_node()


# Entry point to run the LED node as a console script.
def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
