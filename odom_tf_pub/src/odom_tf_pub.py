#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import time

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish transforms
        self.timer = self.create_timer(0.1, self.broadcast_transforms)  # 10 Hz

    def broadcast_transforms(self):
        # Get the current time
        now = self.get_clock().now().to_msg()

        # Map to Odom (Static, assuming identity with some offset)
        self.send_transform('map', 'odom', now, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        # Odom to Base Footprint (Simulated position update)
        self.send_transform('odom', 'base_footprint', now, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        # Base Link to Wheels (Static offsets)
        self.send_transform('base_link', 'wheel_left_link', now, -0.1, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0)
        self.send_transform('base_link', 'wheel_right_link', now, -0.1, -0.15, 0.0, 0.0, 0.0, 0.0, 1.0)

    def send_transform(self, parent, child, time, x, y, z, qx, qy, qz, qw):
        """ Helper function to send transform """
        t = TransformStamped()
        t.header.stamp = time
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
