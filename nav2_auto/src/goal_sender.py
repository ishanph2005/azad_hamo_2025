#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np

class PoseNavigationClient(Node):

    def __init__(self):
        super().__init__('pose_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self):
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 3.6351635456085205
        goal_msg.pose.pose.position.y = -1.4310942888259888
        goal_msg.pose.pose.position.z = 0.002471923828125
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self._action_client.wait_for_server()
        self.get_logger().info("Sending goal...")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.current_pose.pose.position
        self.get_logger().info('(x, y, z) = {0}, {1}, {2}'.format(np.round(feedback.x, 4), np.round(feedback.y, 4), np.round(feedback.z, 4)))


def main(args=None):
    rclpy.init(args=args)
    node = PoseNavigationClient()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
