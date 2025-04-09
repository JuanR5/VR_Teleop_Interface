#!/usr/bin/env python3
"""
ROS 2 Node for commanding a Franka gripper using action clients.

This node subscribes to the `/gripper_command_input` topic, which must publish a
Float32MultiArray message containing two elements [open, grasp].

Logic:
  - If the input array length is less than 2, it warns and does nothing.
  - If both elements are 1, it reports a conflicting command and ignores it.
  - If open is 1 (and grasp is 0), it sends a Move action goal (to open the gripper)
    with:
       • width = max_width parameter
       • speed = open_speed parameter.
  - If grasp is 1 (and open is 0), it sends a Grasp action goal (to close the gripper)
    with:
       • width = 0.0
       • speed = grasp_speed parameter (should be 0.03)
       • force = grasp_force parameter (should be 0.5)
       • epsilon.inner = epsilon.outer = grasp_epsilon parameter (should be 0.08)
  - If the target state is already the current state, no action is taken.
  
Ensure that your gripper launch file (which launches the gripper node and its action servers)
is running. Action servers are expected on `/fr3_gripper/move` and `/fr3_gripper/grasp`.

Usage:
  $ ros2 run <your_package> gripper_command_node.py

Test by publishing, for example:
  $ ros2 topic pub /gripper_command_input std_msgs/msg/Float32MultiArray "{data: [1, 0]}"
  $ ros2 topic pub /gripper_command_input std_msgs/msg/Float32MultiArray "{data: [0, 1]}"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.action import ActionClient

# Import the required action definitions. Ensure the package franka_msgs is installed.
from franka_msgs.action import Move, Grasp

class GripperCommandNode(Node):
    def __init__(self):
        super().__init__('gripper_command_node')

        # Declare parameters and default values.
        self.declare_parameter('max_width', 0.08)      # Maximum opening width for "open" command.
        self.declare_parameter('open_speed', 0.03)       # Speed for Move action (open).
        self.declare_parameter('grasp_speed', 0.03)      # Speed for Grasp action.
        self.declare_parameter('grasp_force', 0.5)       # Force for Grasp action.
        self.declare_parameter('grasp_epsilon', 0.08)    # Epsilon (both inner and outer) for Grasp action.

        self.max_width = self.get_parameter('max_width').value
        self.open_speed = self.get_parameter('open_speed').value
        self.grasp_speed = self.get_parameter('grasp_speed').value
        self.grasp_force = self.get_parameter('grasp_force').value
        self.grasp_epsilon = self.get_parameter('grasp_epsilon').value

        self.get_logger().info(
            f"GripperCommandNode Parameters:\n"
            f"  max_width: {self.max_width}\n"
            f"  open_speed: {self.open_speed}\n"
            f"  grasp_speed: {self.grasp_speed}\n"
            f"  grasp_force: {self.grasp_force}\n"
            f"  grasp_epsilon: {self.grasp_epsilon}"
        )

        # Create action clients for Move and Grasp actions.
        self._move_client = ActionClient(self, Move, '/fr3_gripper/move')
        self._grasp_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')

        # Subscribe to the gripper command input topic.
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/gripper_command_input',
            self.command_callback,
            10
        )
        self.get_logger().info("Waiting for commands on '/gripper_command_input' topic...")

        # Maintain the current state to avoid repeated commands.
        self.current_state = None

    def command_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn("Invalid input: expected [open, grasp]")
            return

        open_cmd = int(msg.data[0])
        grasp_cmd = int(msg.data[1])
        self.get_logger().info(f"Received command: open={open_cmd}, grasp={grasp_cmd}")

        if open_cmd == 1 and grasp_cmd == 1:
            self.get_logger().info("Conflicting command: both open and grasp set. Ignoring.")
            return

        target_state = None
        if open_cmd == 1:
            target_state = 'open'
        elif grasp_cmd == 1:
            target_state = 'grasp'

        if target_state is None:
            self.get_logger().info("No valid command set (both commands are 0). Doing nothing.")
            return

        if target_state == self.current_state:
            self.get_logger().info(f"Already in target state '{target_state}'. No action taken.")
            return

        # Send the corresponding action goal based on the target state.
        self.send_gripper_command(target_state)
        self.current_state = target_state

    def send_gripper_command(self, target_state: str):
        if target_state == 'open':
            self.send_move_goal()
        elif target_state == 'grasp':
            self.send_grasp_goal()
        else:
            self.get_logger().warn("Unknown target state.")

    def send_move_goal(self):
        self.get_logger().info("Preparing Move action goal to open gripper...")
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Move action server not available.")
            return

        goal_msg = Move.Goal()
        goal_msg.width = self.max_width
        goal_msg.speed = self.open_speed

        self.get_logger().info(f"Sending Move goal: width={goal_msg.width}, speed={goal_msg.speed}")
        send_goal_future = self._move_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.move_response_callback)

    def move_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Move goal rejected.")
            return
        self.get_logger().info("Move goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.move_result_callback)

    def move_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Move action result: {result}")

    def send_grasp_goal(self):
        self.get_logger().info("Preparing Grasp action goal to close gripper...")
        if not self._grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Grasp action server not available.")
            return

        goal_msg = Grasp.Goal()
        # Updated parameters based on your command:
        goal_msg.width = 0.0
        goal_msg.speed = self.grasp_speed   # Should be 0.03 as per your spec.
        goal_msg.force = self.grasp_force   # Should be 0.5 as per your spec.
        goal_msg.epsilon.inner = self.grasp_epsilon  # Should be 0.08.
        goal_msg.epsilon.outer = self.grasp_epsilon  # Should be 0.08.

        self.get_logger().info(
            f"Sending Grasp goal: width={goal_msg.width}, speed={goal_msg.speed}, force={goal_msg.force}, "
            f"epsilon(inner,outer)=({goal_msg.epsilon.inner}, {goal_msg.epsilon.outer})"
        )
        send_goal_future = self._grasp_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.grasp_response_callback)

    def grasp_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Grasp goal rejected.")
            return
        self.get_logger().info("Grasp goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.grasp_result_callback)

    def grasp_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Grasp action result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


## TODO
# TO TEST THE CODE
# ros2 topic pub /gripper_command_input std_msgs/msg/Float32MultiArray "data: [1, 0]"  # Open
# ros2 topic pub /gripper_command_input std_msgs/msg/Float64MultiArray "data: [0, 1]"  # Grasp
# ros2 topic pub /gripper_command_input std_msgs/msg/Float64MultiArray "data: [1, 1]"  # Ignored
## TODO