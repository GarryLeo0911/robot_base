import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from picamera2 import Picamera2
from libcamera import Transform
import cv2


class CameraNode(Node):
    # Initialize node, parameters, camera, publisher, and timer.
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('frame_id', 'camera')
        w = int(self.get_parameter('width').get_parameter_value().integer_value)
        h = int(self.get_parameter('height').get_parameter_value().integer_value)
        fps = int(self.get_parameter('fps').get_parameter_value().integer_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()

        self.cam = Picamera2()
        transform = Transform(hflip=0, vflip=0)
        config = self.cam.create_video_configuration(main={"size": (w, h), "format": "RGB888"}, transform=transform)
        self.cam.configure(config)
        self.cam.start()

        self.timer = self.create_timer(1.0 / max(fps, 1), self.capture_and_publish)
        self.get_logger().info(f'Camera node started at {w}x{h}@{fps}fps')

    # Grab a frame from Picamera2 and publish as sensor_msgs/Image.
    def capture_and_publish(self):
        try:
            # Picamera2 returns RGB; convert to BGR for cv_bridge default encoding mapping
            frame = self.cam.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Capture failed: {e}')

    # Stop and close the camera on shutdown.
    def destroy_node(self):
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass
        return super().destroy_node()


# Entry point to run the camera node as a console script.
def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
