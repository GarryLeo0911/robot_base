import sys
import time
import select
import termios
import tty
from contextlib import contextmanager

import rclpy
from geometry_msgs.msg import Twist


@contextmanager
def raw_stdin():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def keymap_to_twist(pressed_keys: set) -> Twist:
    lin = 0.0
    ang = 0.0
    # Invert linear direction so 'w' is forward and 's' is backward
    if 'w' in pressed_keys:
        lin -= 1.0
    if 's' in pressed_keys:
        lin += 1.0
    if 'a' in pressed_keys:
        ang += 1.0
    if 'd' in pressed_keys:
        ang -= 1.0

    msg = Twist()
    msg.linear.x = max(min(lin, 1.0), -1.0)
    msg.angular.z = max(min(ang, 1.0), -1.0)
    return msg


def main(argv=None):
    rclpy.init(args=argv)
    node = rclpy.create_node('teleop_wasd')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    node.get_logger().info('WASD teleop started. Hold keys to move, release to stop. Ctrl+C to exit.')

    # Track currently-pressed keys via expiry timestamps refreshed by key auto-repeat
    pressed_until = {}
    hold_ttl = 0.25  # seconds to keep a key considered pressed without repeats

    last_sent = Twist()  # remember last message to avoid spamming zeros

    try:
        with raw_stdin():
            # Non-blocking read loop
            while rclpy.ok():
                now = time.monotonic()

                # Read all available chars without blocking
                while select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break
                    if ch == '\u0003':  # Ctrl+C
                        raise KeyboardInterrupt
                    key = ch.lower()
                    if key in ('w', 'a', 's', 'd'):
                        pressed_until[key] = now + hold_ttl

                # Expire any keys whose TTL passed
                for k in list(pressed_until.keys()):
                    if pressed_until[k] < now:
                        del pressed_until[k]

                current_keys = set(pressed_until.keys())
                msg = keymap_to_twist(current_keys)

                # Publish only when state changes or non-zero command
                if (msg.linear.x != last_sent.linear.x) or (msg.angular.z != last_sent.angular.z):
                    pub.publish(msg)
                    last_sent = msg

                time.sleep(0.02)  # ~50 Hz loop

    except KeyboardInterrupt:
        pass
    finally:
        # Send a stop command on exit
        stop = Twist()
        pub.publish(stop)
        node.get_logger().info('Teleop stopped.')
        node.destroy_node()
        rclpy.shutdown()
