import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        self.subscription = self.create_subscription(
            Image,
            'rob498_drone_9/recovery/image',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Vector3, 'rob498_drone_9/recovery/servo_command', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Visual Servoing Node Initialized")

    def image_callback(self, msg):
        try:
            # Convert the compressed image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            direction = self.compute_control_command(frame)
            self.publish_command(direction)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def compute_control_command(self, frame):
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2
        
        # Convert to HSV and filter for red target
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Widen the lower and upper bounds for red in HSV
        lower_red1 = np.array([0, 100, 50])   # More tolerant to dark and faded reds
        upper_red1 = np.array([20, 255, 255]) # Increased upper hue to 15
        lower_red2 = np.array([165, 100, 50]) # Expanded lower hue to 165
        upper_red2 = np.array([180, 255, 255]) 

        # Create masks for both red ranges
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2  # Combine both masks
        
        # Find target contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                target_x = int(M['m10'] / M['m00'])
                target_y = int(M['m01'] / M['m00'])
                
                # Compute direction vector to move toward target
                dx = (target_x - center_x) / center_x  # Normalize to [-1, 1]
                dy = (target_y - center_y) / center_y
                return (dx, dy)
        
        return (0.0, 0.0)  # No target detected, no movement

    def publish_command(self, direction):
        msg = Vector3()
        msg.x, msg.y, msg.z = direction[0], direction[1], 0.0  # No Z movement in 2D control
        self.publisher.publish(msg)
        self.get_logger().info(f"Published servo command: ({msg.x}, {msg.y})")


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')

        self.publisher = self.create_publisher(Vector3, 'rob498_drone_9/recovery/servo_command', 10)
        self.circle_sub = self.create_subscription(Vector3, 'rob498_drone_9/recovery/circle_center', self.circle_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Visual Servoing Node Initialized")

    def circle_callback(self, msg):
        circle_center = [msg.x, msg.y]
        direction = self.compute_control_command(circle_center)
        self.publish_command(direction)


    def compute_control_command(self, circle_center):
        height = 360; width = 640
        center_x, center_y = width // 2, height // 2
        
        if circle_center != [0,0]:
            target_x = circle_center[0]
            target_y = circle_center[1]
            
            # Compute direction vector to move toward target
            dx = (target_x - center_x) / center_x  # Normalize to [-1, 1]
            dy = (target_y - center_y) / center_y
            return (dx, dy)
        else:
            return (0.0, 0.0)  # No target detected, no movement

    def publish_command(self, direction):
        msg = Vector3()
        msg.x, msg.y, msg.z = direction[0], direction[1], 0.0  # No Z movement in 2D control
        self.publisher.publish(msg)
        self.get_logger().info(f"Published servo command: ({msg.x}, {msg.y})")


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
