#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from franka_msgs.action import ErrorRecovery

class ErrorRecoveryClient(Node):
    def __init__(self):
        super().__init__('error_recovery_client')
        self._action_client = ActionClient(self, ErrorRecovery, '/action_server/error_recovery')

    def send_goal(self):
        goal_msg = ErrorRecovery.Goal()
        self._action_client.wait_for_server()
        self.get_logger().info('Sending error recovery goal...')
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Error recovery completed successfully!')

def main(args=None):
    rclpy.init(args=args)
    client = ErrorRecoveryClient()
    client.send_goal()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
