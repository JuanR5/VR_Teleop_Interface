#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaRobotState
from std_msgs.msg import String

class FrankaStateMonitor(Node):
    def __init__(self):
        super().__init__('franka_state_monitor')

        # Subscribers
        self.create_subscription(
            PoseStamped,
            '/franka_robot_state_broadcaster/current_pose',
            self.pose_callback,
            10
        )

        self.create_subscription(
            FrankaRobotState,
            '/franka_robot_state_broadcaster/robot_state',
            self.robot_state_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/franka_robot/pose', 10)
        self.error_pub = self.create_publisher(String, '/franka_robot/errors', 10)

    def pose_callback(self, msg):
        """Handles pose messages with clear formatting."""
        self.pose_pub.publish(msg)

        self.get_logger().info(
            f"\n[POSE UPDATE] 📍\n"
            f"Position: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}\n"
            f"Orientation: x={msg.pose.orientation.x:.3f}, y={msg.pose.orientation.y:.3f}, "
            f"z={msg.pose.orientation.z:.3f}, w={msg.pose.orientation.w:.3f}\n"
            f"----------------------------------"
        )

    def robot_state_callback(self, msg):
        """Handles robot state messages, filtering and displaying only relevant issues."""

        # Extract collision status
        collision_detected = any([
            any(msg.collision_indicators.is_joint_collision),
            any(msg.collision_indicators.is_joint_contact),
            msg.collision_indicators.is_cartesian_linear_collision.x != 0.0,
            msg.collision_indicators.is_cartesian_linear_collision.y != 0.0,
            msg.collision_indicators.is_cartesian_linear_collision.z != 0.0,
        ])

        # Extract error flags dynamically
        error_flags = {field: getattr(msg.current_errors, field) for field in dir(msg.current_errors) if not field.startswith('_')}
        active_errors = [key for key, value in error_flags.items() if value]

        if collision_detected:
            self.get_logger().warning("⚠️ COLLISION DETECTED!")

        if active_errors:
            error_msg = f"⚠️ ACTIVE ERRORS: {', '.join(active_errors)}"
            self.get_logger().error(error_msg)
            self.error_pub.publish(String(data=error_msg))

        # Log only if something relevant is happening
        if collision_detected or active_errors:
            self.get_logger().warning(
                f"\n[ROBOT STATE ALERT] ⚠️\n"
                f"{'Collision detected!' if collision_detected else 'No collision.'}\n"
                f"{'Errors present: ' + ', '.join(active_errors) if active_errors else 'No errors detected.'}\n"
                f"----------------------------------"
            )

def main(args=None):
    rclpy.init(args=args)
    node = FrankaStateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
