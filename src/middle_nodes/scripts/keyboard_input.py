#!/usr/bin/env python3

## @file keyboard_input.py
## @brief ROS2 node to control movement using keyboard input.
## @author JuanR5

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading
import time

class KeyboardControlNode(Node):
    """
    @class KeyboardControlNode
    @brief A ROS2 node that translates keyboard input into motion commands.
    
    Publishes geometry_msgs/Twist messages to control robot movement.
    """
    
    def __init__(self):
        """
        @brief Initialize the keyboard control node.
        """
        super().__init__('keyboard_control_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.publisher = self.create_publisher(Twist, 'controller_movement', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 10 Hz

        self.linear_velocity = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.active_keys = set()

        self.key_mapping = {
            'w': ('linear', 0, 1), 's': ('linear', 0, -1),
            'a': ('linear', 1, -1), 'd': ('linear', 1, 1),
            'q': ('linear', 2, -1), 'e': ('linear', 2, 1),
            'u': ('angular', 0, 1), 'j': ('angular', 0, -1),
            'i': ('angular', 1, 1), 'k': ('angular', 1, -1),
            'o': ('angular', 2, 1), 'l': ('angular', 2, -1),
        }

        self.get_logger().info("Starting keyboard listener...")

        self.running = True
        self.thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.thread.start()

    def get_key(self):
        """
        @brief Capture a single key press from stdin.
        @return str Pressed key character.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def keyboard_listener(self):
        """
        @brief Continuously listens to keyboard and handles key events.
        """
        while self.running:
            key = self.get_key()
            if key == '\x03':  # Ctrl+C
                break
            self.handle_key(key, True)
            time.sleep(0.5)
            self.handle_key(key, False)

    def handle_key(self, key, pressed):
        """
        @brief Handle a key press or release event.
        @param key The key character.
        @param pressed Boolean indicating press (True) or release (False).
        """
        key = key.lower()
        if key in self.key_mapping:
            vel_type, index, value = self.key_mapping[key]
            if pressed:
                self.active_keys.add(key)
                if vel_type == 'linear':
                    self.linear_velocity[index] = float(value) * 0.01
                else:
                    self.angular_velocity[index] = float(value) * 0.02
            else:
                self.active_keys.discard(key)
                if key not in self.active_keys:
                    if vel_type == 'linear':
                        self.linear_velocity[index] = 0.0
                    else:
                        self.angular_velocity[index] = 0.0

            self.get_logger().info(f"Active keys: {self.active_keys}, Linear: {self.linear_velocity}, Angular: {self.angular_velocity}")

        elif key == ' ':
            self.linear_velocity = [0.0, 0.0, 0.0]
            self.angular_velocity = [0.0, 0.0, 0.0]
            self.active_keys.clear()
            self.get_logger().info("Emergency stop: Reset all movements")
        else:
            self.get_logger().warning(f"Unmapped key '{key}' pressed, ignoring.")

    def timer_callback(self):
        """
        @brief Timer callback to publish current movement command.
        """
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = self.linear_velocity
        msg.angular.x, msg.angular.y, msg.angular.z = self.angular_velocity
        self.publisher.publish(msg)
        self.get_logger().debug(f"Published: Linear: {self.linear_velocity}, Angular: {self.angular_velocity}")

    def destroy_node(self):
        """
        @brief Stop the keyboard listener thread and cleanup.
        """
        self.running = False
        self.thread.join()
        super().destroy_node()


def main(args=None):
    """
    @brief Main entry point for the node.
    @param args ROS2 command-line arguments.
    """
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
