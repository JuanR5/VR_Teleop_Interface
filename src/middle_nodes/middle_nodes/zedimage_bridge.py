#!/usr/bin/env python3

"""
@file zed_image_bridge.py
@brief ROS2 node for processing raw ZED camera images and republishing them for Unity or other tools.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time


class ZEDImageBridge(Node):
    """
    @class ZEDImageBridge
    @brief A ROS2 node that subscribes to raw left/right images from a ZED camera,
           processes them (e.g., RGB conversion, optional resizing), and republishes them.
    """

    def __init__(self):
        """
        @brief Initializes the ZED image bridge node, sets up subscriptions and publishers.
        """
        super().__init__('zed_image_bridge')
        self.bridge = CvBridge()

        # Timestamps for FPS calculation
        self.last_received_time = {'left': None, 'right': None}
        self.last_published_time = {'left': None, 'right': None}

        # Timer for periodic actions (if needed)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_images)

        # Subscriptions for raw ZED camera images
        self.right_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/right_raw/image_raw_color',
            self.right_image_callback,
            30
        )

        self.left_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left_raw/image_raw_color',
            self.left_image_callback,
            30
        )

        # Publishers for processed images
        self.right_publisher = self.create_publisher(Image, '/zed2_unity/right_image', 30)
        self.left_publisher = self.create_publisher(Image, '/zed2_unity/left_image', 30)

        self.get_logger().info("ZED Image Bridge node initialized.")

    def calculate_fps(self, last_time, eye):
        """
        @brief Calculates FPS for incoming or outgoing frames.

        @param last_time Previous timestamp for that stream.
        @param eye Label indicating which eye/stream is being measured.

        @return New timestamp (now).
        """
        if last_time is None:
            return time.time()

        now = time.time()
        delta = now - last_time

        if delta > 0:
            fps = 1.0 / delta
            self.get_logger().debug(f"{eye.capitalize()} Camera FPS: {fps:.2f}")
        else:
            self.get_logger().debug(f"{eye.capitalize()} Camera FPS: Calculating...")

        return now

    def process_and_publish(self, msg: Image, publisher, eye: str):
        """
        @brief Converts, optionally resizes, and republishes a ZED image.

        @param msg The incoming raw ROS Image message.
        @param publisher The publisher to publish the processed image.
        @param eye A string identifier: 'left' or 'right'.
        """
        try:
            width, height = msg.width, msg.height
            self.last_received_time[eye] = self.calculate_fps(self.last_received_time[eye], f"{eye} input")
            self.get_logger().info(f"Received {eye} image - Resolution: {width}x{height}")

            # Convert to OpenCV format (RGB)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Optional resize step (disabled by default)
            # resized_image = cv2.resize(cv_image, (640, 360))

            # Convert back to ROS2 Image
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
            publisher.publish(ros_image)

            self.last_published_time[eye] = self.calculate_fps(self.last_published_time[eye], f"{eye} output")
            self.get_logger().info(f"Published processed {eye} image.")

        except Exception as e:
            self.get_logger().error(f"Error processing {eye} image: {str(e)}")

    def right_image_callback(self, msg: Image):
        """
        @brief Callback for receiving and processing the right camera image.

        @param msg The incoming ROS Image message.
        """
        self.process_and_publish(msg, self.right_publisher, "right")

    def left_image_callback(self, msg: Image):
        """
        @brief Callback for receiving and processing the left camera image.

        @param msg The incoming ROS Image message.
        """
        self.process_and_publish(msg, self.left_publisher, "left")

    def publish_images(self):
        """
        @brief Placeholder for timer-triggered behavior (e.g., diagnostics).
        Currently just logs a heartbeat if needed.
        """
        pass
        # self.get_logger().info("Publishing images at 30 FPS...")  # Optional heartbeat


def main(args=None):
    """
    @brief Main function to start the ZEDImageBridge node.
    """
    rclpy.init(args=args)
    node = ZEDImageBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
