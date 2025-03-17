import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyzed.sl as sl
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ZEDStreamReceiverNode(Node):
    def __init__(self):
        super().__init__('zed_stream_receiver_node')

        # Publisher for the image topic
        self.image_pub = self.create_publisher(Image, '/zed/left/image_raw', 10)
        self.bridge = CvBridge()

        # ZED SDK setup
        self.init_params = sl.InitParameters()
        self.init_params.input.set_from_stream("145.126.7.157", 30000)  # IP and port of the ZED stream
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.depth_mode = sl.DEPTH_MODE.NONE  # Only images for now

        self.zed = sl.Camera()
        status = self.zed.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to open ZED stream: {status}")
            exit(1)

        self.get_logger().info("Successfully connected to ZED stream")

        # Timer to continuously grab frames and publish
        self.timer = self.create_timer(0.01, self.grab_and_publish_frame)

    def grab_and_publish_frame(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            image = sl.Mat()
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()  # Get numpy array

            # Convert frame to ROS Image message and publish
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(msg)

            # Optional: Display the received frame
            cv2.imshow("ZED Stream", frame)
            cv2.waitKey(1)
        else:
            self.get_logger().warn("Failed to grab frame from ZED stream")

    def destroy_node(self):
        self.zed.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
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
