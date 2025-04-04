#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class EquilibriumPosePublisher(Node):
    def __init__(self):
        super().__init__('equilibrium_pose_publisher')

        self.declare_parameter('deadband', 0.001)
        self.declare_parameter('step_linear', 0.2)
        self.declare_parameter('step_angular', 0.5)
        self.declare_parameter('filter_alpha', 0.1)

        self.deadband = self.get_parameter('deadband').value
        self.step_linear = self.get_parameter('step_linear').value
        self.step_angular = self.get_parameter('step_angular').value
        self.filter_alpha = self.get_parameter('filter_alpha').value

        self.sub_movement = self.create_subscription(
            Twist, 'controller_movement', self.controller_movement_callback, 10)
        self.sub_current_pose = self.create_subscription(
            PoseStamped, 'franka_robot_state_broadcaster/current_pose', self.current_pose_callback, 10)

        self.pub_equilibrium_pose = self.create_publisher(PoseStamped, 'new_goal_pose', 20)

        self.latest_cmd = np.zeros(6)  # [dx, dy, dz, d_roll, d_pitch, d_yaw]
        self.has_new_input = False

        self.current_pose = None
        self.desired_pose = None

        self.timer = self.create_timer(0.1, self.timer_callback)

    def controller_movement_callback(self, msg: Twist):
        vx, vy, vz = msg.linear.x, msg.linear.y, msg.linear.z
        wx, wy, wz = msg.angular.x, msg.angular.y, msg.angular.z

        if not all(np.isfinite([vx, vy, vz, wx, wy, wz])):
            self.get_logger().warn("Received invalid twist command; ignoring.")
            return

        vx = 0.0 if abs(vx) < self.deadband else vx
        vy = 0.0 if abs(vy) < self.deadband else vy
        vz = 0.0 if abs(vz) < self.deadband else vz
        wx = 0.0 if abs(wx) < self.deadband else wx
        wy = 0.0 if abs(wy) < self.deadband else wy
        wz = 0.0 if abs(wz) < self.deadband else wz

        linear_increment = np.array([vx, vy, vz]) * self.step_linear
        angular_increment = np.array([wx, wy, wz]) * self.step_angular
        self.latest_cmd = np.concatenate((linear_increment, angular_increment))

        if not np.allclose(self.latest_cmd, 0.0, atol=1e-6):
            self.has_new_input = True
            self.get_logger().info(f"Received movement command: linear {linear_increment}, angular {angular_increment}")

    def current_pose_callback(self, msg: PoseStamped):
        self.current_pose = msg
        if self.desired_pose is None:
            self.desired_pose = PoseStamped()
            self.desired_pose.header = msg.header
            self.desired_pose.pose.position = msg.pose.position
            self.desired_pose.pose.orientation = msg.pose.orientation

    def timer_callback(self):
        if not self.has_new_input or self.current_pose is None or self.desired_pose is None:
            return

        self.has_new_input = False  # Reset the flag until next twist input

        delta = self.latest_cmd

        current_desired_pos = np.array([
            self.desired_pose.pose.position.x,
            self.desired_pose.pose.position.y,
            self.desired_pose.pose.position.z
        ])
        new_pos = current_desired_pos + delta[0:3]

        smoothed_pos = (1 - self.filter_alpha) * current_desired_pos + self.filter_alpha * new_pos

        current_quat = np.array([
            self.desired_pose.pose.orientation.x,
            self.desired_pose.pose.orientation.y,
            self.desired_pose.pose.orientation.z,
            self.desired_pose.pose.orientation.w
        ])
        r_current = R.from_quat(current_quat)

        # Apply small angular increments
        new_euler = r_current.as_euler('xyz') + delta[3:6]
        r_new = R.from_euler('xyz', new_euler)

        # SLERP
        
        # SLERP requires key times and a Rotation object containing multiple rotations
        key_times = [0, 1]
        key_rots = R.concatenate([r_current, r_new])
        slerp = Slerp(key_times, key_rots)
        r_smoothed = slerp(self.filter_alpha)
        smoothed_quat = r_smoothed.as_quat()
        
        # Update and publish
        self.desired_pose.pose.position.x = smoothed_pos[0]
        self.desired_pose.pose.position.y = smoothed_pos[1]
        self.desired_pose.pose.position.z = smoothed_pos[2]
        self.desired_pose.pose.orientation.x = smoothed_quat[0]
        self.desired_pose.pose.orientation.y = smoothed_quat[1]
        self.desired_pose.pose.orientation.z = smoothed_quat[2]
        self.desired_pose.pose.orientation.w = smoothed_quat[3]

        self.desired_pose.header.stamp = self.get_clock().now().to_msg()
        self.pub_equilibrium_pose.publish(self.desired_pose)

        self.get_logger().info(f"Published new equilibrium pose:\n  Position: {smoothed_pos}\n  Orientation: {smoothed_quat}")


def main(args=None):
    rclpy.init(args=args)
    node = EquilibriumPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
