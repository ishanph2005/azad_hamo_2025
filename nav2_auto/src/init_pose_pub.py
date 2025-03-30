#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class PosePublisher(Node):

    def __init__(self):
        super().__init__('pose_estimate_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id =  'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0062499274499714375
        msg.pose.pose.position.y = -0.003124893642961979
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = -1.848155632559608e-06
        msg.pose.pose.orientation.w = 0.9999999999982921
        msg.pose.covariance = [
                                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,  # Row 1
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,  # Row 2
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Row 3
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Row 4
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Row 5
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467  # Row 6
                            ]

        self.publisher_.publish(msg)
        self.get_logger().info('Published initial pose')

def main(args=None):
    rclpy.init(args=args)

    pose_publisher = PosePublisher()

    rclpy.spin(pose_publisher)

    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
