# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from moveit_commander.robot_trajectory import RobotTrajectory
# from moveit_commander.move_group import MoveGroupCommander
# from geometry_msgs.msg import Pose
# from std_msgs.msg import String

# class GoalPositionNode(Node):
#     def __init__(self):
#         super().__init__('goal_position_node')

#         # Initialize MoveIt!
#         self.move_group = MoveGroupCommander("fr3_arm")
        
#         # Create a subscription to receive goal positions
#         self.goal_subscriber = self.create_subscription(
#             Pose, 
#             'goal_position_topic',  # The topic to subscribe to
#             self.goal_callback,
#             10
#         )
        
#         # Optionally, add a publisher to confirm goals received or execution state
#         self.goal_status_publisher = self.create_publisher(String, 'goal_status_topic', 10)
        
#     def goal_callback(self, msg: Pose):
#         self.get_logger().info(f"Received new goal position: {msg.position.x}, {msg.position.y}, {msg.position.z}")
        
#         # Plan and execute motion
#         self.plan_and_execute(msg)
        
#     def plan_and_execute(self, goal_pose: Pose):
#         # Set the target pose for the arm (we assume it's a position goal)
#         self.move_group.set_pose_target(goal_pose)
        
#         # Plan the motion
#         plan = self.move_group.plan()
        
#         # If planning was successful, execute it
#         if plan:
#             self.get_logger().info("Planning successful, executing...")
#             self.move_group.execute(plan, wait=True)
#             # Optionally publish the status
#             self.publish_goal_status("Execution successful")
#         else:
#             self.get_logger().warn("Planning failed, not executing.")
#             self.publish_goal_status("Execution failed")
    
#     def publish_goal_status(self, status: str):
#         msg = String()
#         msg.data = status
#         self.goal_status_publisher.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = GoalPositionNode()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
