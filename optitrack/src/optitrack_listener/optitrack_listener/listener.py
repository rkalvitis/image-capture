"""Subscribes to /optitrack/<rigid_body_name> (geometry_msgs/PoseStamped)
topics published by optitrack_ros_py and prints each pose at ~1 Hz per body.

Topic discovery is dynamic: the listener polls the ROS graph every 2 s and
subscribes to any new PoseStamped topic under /optitrack/. You don't need to
hard-code rigid body names — define them in Motive and they show up here.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


PREFIX = '/optitrack/'
POSE_TYPE = 'geometry_msgs/msg/PoseStamped'


class OptitrackListener(Node):
    def __init__(self):
        super().__init__('optitrack_listener')
        self.subs: dict[str, rclpy.subscription.Subscription] = {}
        self.last_log_ns: dict[str, int] = {}
        self.create_timer(2.0, self.discover)
        self.get_logger().info(f'Watching for {PREFIX}* PoseStamped topics...')

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            if not topic.startswith(PREFIX):
                continue
            if POSE_TYPE not in types:
                continue
            if topic in self.subs:
                continue
            name = topic[len(PREFIX):]
            self.subs[topic] = self.create_subscription(
                PoseStamped, topic,
                lambda msg, n=name: self.on_pose(n, msg),
                10,
            )
            self.get_logger().info(f'Subscribed to {topic}')

    def on_pose(self, name: str, msg: PoseStamped):
        # Throttle per rigid body to ~1 Hz so logs stay readable.
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns.get(name, 0) < 1_000_000_000:
            return
        self.last_log_ns[name] = now_ns
        p = msg.pose.position
        q = msg.pose.orientation
        self.get_logger().info(
            f'[{name}] pos=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) '
            f'quat=({q.x:+.3f}, {q.y:+.3f}, {q.z:+.3f}, {q.w:+.3f})'
        )


def main():
    rclpy.init()
    node = OptitrackListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
