"""Mock OptiTrack: publishes a synthetic camera trajectory as PoseStamped.

Stands in for the real driver when no OptiTrack system is reachable, so the
record → export pipeline can be tested end-to-end. The pose moves on a circle
around the origin while always facing it — roughly what a handheld camera
orbiting a fixed object produces.

Parameters:
  topic   (str,   default '/optitrack/camera')
  rate    (float, default 120.0)  — Hz, matches a typical Motive rate
  radius  (float, default 1.5)    — metres
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class MockPosePublisher(Node):
    def __init__(self):
        super().__init__('mock_optitrack')
        self.declare_parameter('topic', '/optitrack/camera')
        self.declare_parameter('rate', 120.0)
        self.declare_parameter('radius', 1.5)

        topic = self.get_parameter('topic').value
        rate = self.get_parameter('rate').value
        self._radius = self.get_parameter('radius').value

        self._pub = self.create_publisher(PoseStamped, topic, 10)
        self._t0 = self.get_clock().now()
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().warn(f'MOCK OPTITRACK — synthetic circular trajectory on {topic} @ {rate} Hz')

    def _tick(self):
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9
        a = 2.0 * math.pi * t / 20.0  # one orbit every 20 s

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = self._radius * math.cos(a)
        msg.pose.position.y = self._radius * math.sin(a)
        msg.pose.position.z = 1.0
        # Yaw so the camera faces the origin (rotation about z by a + pi)
        yaw = a + math.pi
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = MockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
