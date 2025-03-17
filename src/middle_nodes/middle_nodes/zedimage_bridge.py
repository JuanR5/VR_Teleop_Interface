import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class ZEDImageBridge(Node):
    """
    A ROS2 node that subscribes to ZED camera raw images (left and right),
    processes them (resizes and converts to RGB), and republishes them.
    """

    def __init__(self):
        super().__init__('zed_image_bridge')
        self.bridge = CvBridge()

        # Store last received timestamps for FPS calculation
        self.last_received_time = {'left': None, 'right': None}
        self.last_published_time = {'left': None, 'right': None}

        # Create a timer to control publishing frequency (30 FPS)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_images)

        # Subscribe to ZED left and right raw image topics with a larger queue size
        self.right_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/right_raw/image_raw_color',
            self.right_image_callback,
            30  # Increased queue size to 30
        )

        self.left_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left_raw/image_raw_color',
            self.left_image_callback,
            30  # Increased queue size to 30
        )

        # Publishers for processed RGB images
        self.right_publisher = self.create_publisher(Image, '/zed2_unity/right_image', 30)  # Queue size of 30
        self.left_publisher = self.create_publisher(Image, '/zed2_unity/left_image', 30)  # Queue size of 30

        self.get_logger().info("ZED Image Bridge node initialized.")

    def calculate_fps(self, last_time, eye):
        """Calculates FPS based on the time difference between messages."""
        if last_time is None:
            return None
        now = time.time()
        fps = 1.0 / (now - last_time) if (now - last_time) > 0 else None
        self.get_logger().debug(f"{eye.capitalize()} Camera FPS: {fps:.2f}" if fps else f"{eye.capitalize()} Camera FPS: Calculating...")
        return now

    def process_and_publish(self, msg: Image, publisher, eye: str):
        """
        Converts a ROS2 Image message to OpenCV format, resizes it, and republishes it.

        Args:
            msg (sensor_msgs.msg.Image): The incoming image message.
            publisher (rclpy.Publisher): The ROS2 publisher for the processed image.
            eye (str): Specifies whether the image is from the "left" or "right" camera.
        """
        try:
            # Extract resolution from message
            width = msg.width
            height = msg.height

            # Calculate input FPS
            self.last_received_time[eye] = self.calculate_fps(self.last_received_time[eye], f"{eye} input")

            self.get_logger().info(f"Received {eye} image - Resolution: {width}x{height}")

            # Convert ROS Image message to OpenCV format with RGB encoding
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Resize image to 1280x720
            #resized_image = cv2.resize(cv_image, (640, 360))

            # Convert the OpenCV image back to a ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')

            # Publish the processed image
            publisher.publish(ros_image)

            # Calculate output FPS
            self.last_published_time[eye] = self.calculate_fps(self.last_published_time[eye], f"{eye} output")

            self.get_logger().info(f"Published processed {eye} image.")

        except Exception as e:
            self.get_logger().error(f"Error processing {eye} image: {str(e)}")

    def right_image_callback(self, msg: Image):
        """Callback function for processing and publishing the right camera image."""
        self.process_and_publish(msg, self.right_publisher, "right")

    def left_image_callback(self, msg: Image):
        """Callback function for processing and publishing the left camera image."""
        self.process_and_publish(msg, self.left_publisher, "left")

    def publish_images(self):
        """Periodically publishes images at 30 FPS."""
        self.get_logger().info("Publishing images at 30 FPS...")

def main(args=None):
    """Main function to initialize and run the ZEDImageBridge node."""
    rclpy.init(args=args)
    node = ZEDImageBridge()
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
