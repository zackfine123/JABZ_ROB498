import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, PoseStamped
from cv_bridge import CvBridge

class TargetDetectionNode(Node):
    def __init__(self):
        super().__init__('target_detection_node')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image,'rob498_drone_9/recovery/image',  self.image_callback, 10)
        self.sub_vision_pose = self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.callback_vision_pose, 10)
        
        self.publisher = self.create_publisher(Bool, 'rob498_drone_9/recovery/target_detected', 10)
        self.image_publisher = self.create_publisher(Image, 'rob498_drone_9/recovery/detected_image', 10)
        self.control_publisher = self.create_publisher(Vector3, 'rob498_drone_9/recovery/servo_command', 10)
        self.get_logger().info("Target Detection Node Initialized")
    
    def callback_vision_pose(self, msg):
        self.vision_origin = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def get_current_vision_pose(self):
        return getattr(self, 'vision_origin', None)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detected, processed_image, control_cmd = self.detect_red_circle(cv_image)
        
        detection_msg = Bool()
        detection_msg.data = detected
        self.publisher.publish(detection_msg)
        
        # Publish the processed image with detected circles
        processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
        self.image_publisher.publish(processed_image_msg)

        # publish control command
        control_msg = Vector3()
        control_msg.x, control_msg.y, control_msg.z = control_cmd[0], control_cmd[1], 0.0  # No Z movement in 2D control
        self.control_publisher.publish(control_msg)
        
    def detect_red_circle(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Widen the lower and upper bounds for red in HSV
        lower_red1 = np.array([0, 100, 50])   # More tolerant to dark and faded reds
        upper_red1 = np.array([20, 255, 255]) # Increased upper hue to 15
        lower_red2 = np.array([165, 100, 50]) # Expanded lower hue to 165
        upper_red2 = np.array([180, 255, 255]) 

        # Get image dimensions
        height, width = image.shape[:2]
        mask = np.zeros((height, width), dtype=np.uint8)
        center_x, center_y = width // 2, height // 2
        cv2.circle(mask, (center_x, center_y), 180, 255, -1)
        masked_image = cv2.bitwise_and(image, image, mask=mask)

        # Create masks for both red ranges
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2  # Combine both masks

        # Apply mask to keep only red regions
        red_only = cv2.bitwise_and(masked_image, masked_image, mask=mask)

        # Convert masked image to grayscale
        gray = cv2.cvtColor(red_only, cv2.COLOR_BGR2GRAY)

        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        # Blur the grayscale image to reduce noise
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        current_position = self.get_current_vision_pose()
        if current_position is None:
            minr = 5
            maxr = 70
        else:
            minr = int(25/current_position[2] - 10)
            maxr = int(25/current_position[2] + 15)
        # Detect circles using Hough Transform
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
            param1=50, param2=30, minRadius=minr, maxRadius=maxr
        )

        detected = False
        control_cmd = (0.0,0.0)

        if circles is not None:
            detected = True
            circles = np.uint16(np.around(circles))  # Convert to integers
            lrad = 0
            largest_circle = np.zeros(3)  # Get the largest circle (x, y, r)
            for i in circles[0, :]:
                if i[2] > lrad:
                    largest_circle = i
                    lrad = i[2]
                cv2.circle(image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(image, (i[0], i[1]), 2, (0, 0, 255), 3)
            self.get_logger().info(f"radius: {lrad}")
            control_cmd = self.compute_control_command(image, largest_circle) # visual servoing
        #blurred = cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR)
        return detected, image, control_cmd

    def compute_control_command(self, frame, circle_center):
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2
        target_x = circle_center[0]
        target_y = circle_center[1]
        # Compute direction vector to move toward target
        dx = (target_x - center_x) / center_x  # Normalize to [-1, 1]
        dy = (target_y - center_y) / center_y
        return (-dx, dy)
    
def main(args=None):
    rclpy.init(args=args)
    node = TargetDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
