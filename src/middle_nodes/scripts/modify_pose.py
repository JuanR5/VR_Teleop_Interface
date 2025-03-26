#!/usr/bin/env python3

## @file equilibrium_pose_publisher.py
## @brief ROS2 node to generate and smooth target poses based on twist commands.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import transforms3d as tf_transformations

class EquilibriumPosePublisher(Node):
    """
    @class EquilibriumPosePublisher
    @brief A ROS2 node to compute and publish a smoothed equilibrium pose based on twist commands.

    Subscribes to Twist for user input and PoseStamped for current pose feedback. 
    Publishes an updated goal pose periodically.
    """
    def __init__(self):
        """
        @brief Initialize the equilibrium pose publisher.
        """
        super().__init__('equilibrium_pose_publisher')

        # Declare parameters with default values
        self.declare_parameter('deadband', 0.001)
        self.declare_parameter('step_linear', 0.005)
        self.declare_parameter('step_angular', 0.005)
        self.declare_parameter('filter_alpha', 0.1)

        self.deadband = self.get_parameter('deadband').value
        self.step_linear = self.get_parameter('step_linear').value
        self.step_angular = self.get_parameter('step_angular').value
        self.filter_alpha = self.get_parameter('filter_alpha').value

        # Subscriptions
        self.sub_movement = self.create_subscription(Twist, 'controller_movement', self.controller_movement_callback, 10)
        self.sub_current_pose = self.create_subscription(PoseStamped, 'franka_robot_state_broadcaster/current_pose', self.current_pose_callback, 10)

        # Publisher
        self.pub_equilibrium_pose = self.create_publisher(PoseStamped, 'new_goal_pose', 20)

        self.latest_cmd = np.zeros(6)  # [dx, dy, dz, d_roll, d_pitch, d_yaw]
        self.current_pose = None
        self.desired_pose = None

        # Timer for publishing at fixed rate
        self.timer = self.create_timer(0.1, self.timer_callback)

    def controller_movement_callback(self, msg: Twist):
        """
        @brief Callback for movement input.
        @param msg Twist message containing user input.
        """
        vx, vy, vz = msg.linear.x, msg.linear.y, msg.linear.z
        wx, wy, wz = msg.angular.x, msg.angular.y, msg.angular.z

        if not all(np.isfinite([vx, vy, vz, wx, wy, wz])):
            self.get_logger().warn("Received invalid twist command; ignoring.")
            return

        # Deadband filtering
        vx = 0.0 if abs(vx) < self.deadband else vx
        vy = 0.0 if abs(vy) < self.deadband else vy
        vz = 0.0 if abs(vz) < self.deadband else vz
        wx = 0.0 if abs(wx) < self.deadband else wx
        wy = 0.0 if abs(wy) < self.deadband else wy
        wz = 0.0 if abs(wz) < self.deadband else wz

        linear_increment = np.array([vx, vy, vz], dtype=float) * self.step_linear
        angular_increment = np.array([wx, wy, wz], dtype=float) * self.step_angular
        self.latest_cmd = np.concatenate((linear_increment, angular_increment))

        self.get_logger().info(f"Received movement command: linear {linear_increment}, angular {angular_increment}")

    def current_pose_callback(self, msg: PoseStamped):
        """
        @brief Callback to update the robot's current pose.
        @param msg PoseStamped message of the current pose.
        """
        self.current_pose = msg
        if self.desired_pose is None:
            self.desired_pose = PoseStamped()
            self.desired_pose.header = msg.header
            self.desired_pose.pose.position.x = msg.pose.position.x
            self.desired_pose.pose.position.y = msg.pose.position.y
            self.desired_pose.pose.position.z = msg.pose.position.z
            self.desired_pose.pose.orientation = msg.pose.orientation

    def timer_callback(self):
        """
        @brief Periodic timer callback to compute and publish the smoothed target pose.
        """
        if self.current_pose is None or self.desired_pose is None:
            return

        delta = self.latest_cmd

        current_desired_pos = np.array([
            self.desired_pose.pose.position.x,
            self.desired_pose.pose.position.y,
            self.desired_pose.pose.position.z
        ])
        new_pos = current_desired_pos + delta[0:3]

        current_quat = [
            self.desired_pose.pose.orientation.x,
            self.desired_pose.pose.orientation.y,
            self.desired_pose.pose.orientation.z,
            self.desired_pose.pose.orientation.w
        ]
        current_euler = tf_transformations.euler_from_quaternion(current_quat)
        new_euler = np.array(current_euler) + delta[3:6]
        new_quat = tf_transformations.quaternion_from_euler(*new_euler)

        smoothed_pos = (1 - self.filter_alpha) * current_desired_pos + self.filter_alpha * new_pos
        smoothed_euler = (1 - self.filter_alpha) * np.array(current_euler) + self.filter_alpha * new_euler
        smoothed_quat = tf_transformations.quaternion_from_euler(*smoothed_euler)

        self.desired_pose.pose.position.x = smoothed_pos[0]
        self.desired_pose.pose.position.y = smoothed_pos[1]
        self.desired_pose.pose.position.z = smoothed_pos[2]
        self.desired_pose.pose.orientation.x = smoothed_quat[0]
        self.desired_pose.pose.orientation.y = smoothed_quat[1]
        self.desired_pose.pose.orientation.z = smoothed_quat[2]
        self.desired_pose.pose.orientation.w = smoothed_quat[3]

        self.desired_pose.header.stamp = self.get_clock().now().to_msg()
        self.pub_equilibrium_pose.publish(self.desired_pose)

        self.get_logger().info(
            f"Published new equilibrium pose: Position: {smoothed_pos}, Orientation: {smoothed_quat}")


def main(args=None):
    """
    @brief Main function to initialize and run the ROS2 node.
    @param args ROS2 launch arguments.
    """
    rclpy.init(args=args)
    node = EquilibriumPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
