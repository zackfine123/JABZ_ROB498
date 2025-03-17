import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from time import sleep
from cv_bridge import CvBridge

class ImagePubNode(Node):
    def __init__(self):
        super().__init__('image_pub_node')
        self.image_pub = self.create_publisher(Image, 'rob498_drone_9/recovery/image', 10)
        self.timer = self.create_timer(0.2, self.process_frame)  # Capture an image every second

        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc wbmode=0 ! video/x-raw(memory:NVMM), width=640, height=360, format=(string)NV12, framerate=4/1 ! "
            "queue max-size-buffers=1 leaky=downstream ! "
            "nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=True",
            cv2.CAP_GSTREAMER
        )


        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            exit()

        self.get_logger().info("Image Node Initialized")
        self.bridge = CvBridge()

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image!")
            return

        self.publish_image(frame)
        self.get_logger().info("Published frame")
    
    def publish_image(self, frame):
        # Convert the frame to a ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
