#!/usr/bin/env python3

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
    ROS2 Node to control movement using keyboard input.
    Listens for key presses and translates them into movement commands.
    """
    
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        # Set logger level
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Publisher to send movement commands
        self.publisher = self.create_publisher(Twist, 'controller_movement', 10)
        
        # Timer to publish movement at a fixed interval (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Initialize velocity values
        self.linear_velocity = [0.0, 0.0, 0.0]  # X, Y, Z
        self.angular_velocity = [0.0, 0.0, 0.0] # Roll, Pitch, Yaw
        self.active_keys = set()
        
        # Key mappings for movement
        self.key_mapping = {
            'w': ('linear', 0, 1),   # Forward (X+)
            's': ('linear', 0, -1),  # Backward (X-)
            'a': ('linear', 1, -1),  # Left (Y-)
            'd': ('linear', 1, 1),   # Right (Y+)
            'q': ('linear', 2, -1),  # Down (Z-)
            'e': ('linear', 2, 1),   # Up (Z+)
            'u': ('angular', 0, 1),  # Roll+
            'j': ('angular', 0, -1), # Roll-
            'i': ('angular', 1, 1),  # Pitch+
            'k': ('angular', 1, -1), # Pitch-
            'o': ('angular', 2, 1),  # Yaw+
            'l': ('angular', 2, -1), # Yaw-
        }
        
        self.get_logger().info("Starting keyboard listener...")
        
        # Start the keyboard listening thread
        self.running = True
        self.thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.thread.start()

    def get_key(self):
        """Read a single character from standard input."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def keyboard_listener(self):
        """Continuously listen for keyboard input."""
        while self.running:
            key = self.get_key()
            if key == '\x03':  # Ctrl+C to exit
                break
            
            self.handle_key(key, True)
            
            # Simulate key release after a short delay
            time.sleep(0.1)
            self.handle_key(key, False)

    def handle_key(self, key, pressed):
        """Process key presses and update movement values."""
        key = key.lower()
        
        if key in self.key_mapping:
            vel_type, index, value = self.key_mapping[key]
            
            if pressed:
                self.active_keys.add(key)
                if vel_type == 'linear':
                    self.linear_velocity[index] = float(value) 
                else:
                    self.angular_velocity[index] = float(value)
            else:
                self.active_keys.discard(key)
                if key not in self.active_keys:
                    if vel_type == 'linear':
                        self.linear_velocity[index] = 0.0
                    else:
                        self.angular_velocity[index] = 0.0
            
            self.get_logger().info(f"Active keys: {self.active_keys}, Linear: {self.linear_velocity}, Angular: {self.angular_velocity}")
        
        elif key == ' ':  # Emergency stop
            self.linear_velocity = [0.0, 0.0, 0.0]
            self.angular_velocity = [0.0, 0.0, 0.0]
            self.active_keys.clear()
            self.get_logger().info("Emergency stop: Reset all movements")
        else:
            self.get_logger().warning(f"Unmapped key '{key}' pressed, ignoring.")

    def timer_callback(self):
        """Publish the current state at regular intervals."""
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = self.linear_velocity
        msg.angular.x, msg.angular.y, msg.angular.z = self.angular_velocity
        self.publisher.publish(msg)
        self.get_logger().debug(f"Published: Linear: {self.linear_velocity}, Angular: {self.angular_velocity}")

    def destroy_node(self):
        """Stop keyboard listener and shutdown node."""
        self.running = False
        self.thread.join()
        super().destroy_node()


def main(args=None):
    """Main function to start the ROS2 node."""
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
