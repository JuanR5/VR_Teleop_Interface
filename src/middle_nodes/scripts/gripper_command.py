import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/gripper_command_input',
            self.command_callback,
            10
        )
        self.gripper_client = ActionClient(self, GripperCommand, '/franka_gripper_node/gripper_action')

        # Internal state
        self.current_state = None  # 'open', 'grasp', or None

        # Desired open and grasp positions (meters)
        self.open_width = 0.08
        self.grasp_width = 0.0
        self.max_effort = 20.0

        self.get_logger().info("Gripper controller node started")

    def command_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn("Invalid input: expected [open, grasp]")
            return

        open_cmd, grasp_cmd = int(msg.data[0]), int(msg.data[1])

        if open_cmd == 1 and grasp_cmd == 1:
            self.get_logger().info("Conflicting command: both open and grasp set. Ignoring.")
            return

        # Determine target state
        target_state = None
        if open_cmd == 1:
            target_state = 'open'
        elif grasp_cmd == 1:
            target_state = 'grasp'

        # If already in the target state, do nothing
        if target_state == self.current_state:
            return

        # Send action to gripper
        self.send_gripper_command(target_state)

    def send_gripper_command(self, target_state):
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper action server not available')
            return

        goal_msg = GripperCommand.Goal()

        if target_state == 'open':
            goal_msg.command.position = self.open_width / 2.0  # Franka expects half width
            goal_msg.command.max_effort = 0.0
        elif target_state == 'grasp':
            goal_msg.command.position = self.grasp_width / 2.0
            goal_msg.command.max_effort = self.max_effort
        else:
            self.get_logger().warn("Unknown target state")
            return

        self.get_logger().info(f"Sending gripper command: {target_state}")
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.handle_result(future, target_state))

    def handle_result(self, future, target_state):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Gripper goal was rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.handle_action_result(f, target_state))

    def handle_action_result(self, future, target_state):
        result = future.result().result
        if result.reached_goal:
            self.get_logger().info(f"Gripper {target_state} action succeeded")
            self.current_state = target_state
        else:
            self.get_logger().warn(f"Gripper {target_state} action failed")

def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


## TODO
# TO TEST THE CODE
# ros2 topic pub /gripper_command_input std_msgs/msg/Float64MultiArray "data: [1, 0]"  # Open
# ros2 topic pub /gripper_command_input std_msgs/msg/Float64MultiArray "data: [0, 1]"  # Grasp
# ros2 topic pub /gripper_command_input std_msgs/msg/Float64MultiArray "data: [1, 1]"  # Ignored
## TODO