import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import os
from cv_bridge import CvBridge
from datetime import datetime


class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'rob498_drone_9/recovery/detected_image',
            self.image_callback,
            1
        )
        self.save_folder = "saved_frames"

        self.get_logger().info("Image Display Node Initialized")


    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display the image
            cv2.imshow('Recovery Image', cv_image)
            cv2.waitKey(1)  # Required to refresh OpenCV window
            self.save_frame(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error displaying image: {str(e)}")
            # Save the frame to disk
        

    def save_frame(self, frame):
        # Generate filename based on timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = os.path.join(self.save_folder, f"frame_{timestamp}.jpg")

        # Save the image
        cv2.imwrite(filename, frame)
        self.get_logger().info(f"Saved frame: {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
