#!/usr/bin/env python3

"""
@file zed_stream_receiver_node.py
@brief ROS2 node for receiving and publishing ZED camera video stream over network using ZED SDK.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyzed.sl as sl
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ZEDStreamReceiverNode(Node):
    """
    @class ZEDStreamReceiverNode
    @brief A ROS2 node that connects to a remote ZED camera stream, captures frames, and publishes them as ROS Image messages.
    """

    def __init__(self):
        """
        @brief Initializes the node, connects to the ZED stream, and sets up the publisher and timer.
        """
        super().__init__('zed_stream_receiver_node')

        # === ROS Image Publisher ===
        self.image_pub = self.create_publisher(Image, '/zed/left/image_raw', 10)
        self.bridge = CvBridge()

        # === ZED SDK Parameters ===
        self.init_params = sl.InitParameters()
        self.init_params.input.set_from_stream("145.126.7.157", 30000)  # IP and port of the ZED stream
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.depth_mode = sl.DEPTH_MODE.NONE  # Depth disabled; only image streaming

        # === ZED Camera Initialization ===
        self.zed = sl.Camera()
        status = self.zed.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to open ZED stream: {status}")
            exit(1)

        self.get_logger().info("Successfully connected to ZED stream")

        # === Frame Grabbing Timer (100 Hz) ===
        self.timer = self.create_timer(0.01, self.grab_and_publish_frame)

    def grab_and_publish_frame(self):
        """
        @brief Captures a frame from the ZED stream and publishes it as a ROS Image.
        """
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            image = sl.Mat()
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()  # NumPy array

            # Convert to ROS Image message and publish
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(msg)

            # Optional: Display in OpenCV window
            cv2.imshow("ZED Stream", frame)
            cv2.waitKey(1)
        else:
            self.get_logger().warn("Failed to grab frame from ZED stream")

    def destroy_node(self):
        """
        @brief Cleans up resources and shuts down ZED connection and OpenCV window.
        """
        self.zed.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """
    @brief Main entry point. Initializes and spins the ZED stream receiver node.
    """
    rclpy.init(args=args)
    node = ZEDStreamReceiverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
