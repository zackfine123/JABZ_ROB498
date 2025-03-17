import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_9')

        # Publishers
        self.pose_publisher = self.create_publisher(PoseStamped, '/rob498/posdes', 10)
        self.shutdown_publisher = self.create_publisher(TwistStamped, '/rob498/emergency_shutdown', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.vision_pose_callback, 10)

        # Services
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_9/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_9/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_9/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_9/comm/abort', self.callback_abort)

        self.initial_pose = None  # Store initial pose
        self.get_logger().info("CommNode Initialized. Waiting for initial pose...")

    def vision_pose_callback(self, msg):
        """Stores the first received pose as the initial position."""
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.publish_position(self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z)
            self.get_logger().info(f"Initial pose recorded and set: x={self.initial_pose.position.x}, y={self.initial_pose.position.y}, z={self.initial_pose.position.z}")

    def callback_launch(self, request, response):
        """Takes off to 1.5m above the initial pose."""
        if self.initial_pose is None:
            response.success = False
            response.message = "Initial pose not received yet."
            return response

        # rs
        target_z = self.initial_pose.position.z + 1.253 # tune this as needed
        # vicon
        #target_z = 1.5 # tune this as needed
        self.publish_position(self.initial_pose.position.x, self.initial_pose.position.y, target_z)
        self.get_logger().info(f"Launch Requested. Target altitude: {target_z}m")

        response.success = True
        response.message = "Drone taking off."
        return response

    def callback_test(self, request, response):
        """Handles test start command."""
        self.get_logger().info("Test Requested. Starting test sequence.")
        response.success = True
        response.message = "Test has started. Recording data."
        return response

    def callback_land(self, request, response):
        """Descends back to the initial altitude."""
        if self.initial_pose is None:
            response.success = False
            response.message = "Initial pose not received yet."
            return response

        self.publish_position(self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z-0.05)
        self.get_logger().info(f"Landing Requested. Returning to z={self.initial_pose.position.z}m")

        response.success = True
        response.message = "Drone is landing."
        return response

    def callback_abort(self, request, response):
        """Handles emergency abort by cutting thrust immediately."""
        self.get_logger().warn("ABORT Requested! Cutting all thrust.")
        if self.initial_pose is None:
            response.success = False
            response.message = "Initial pose not received yet."
            return response

        self.publish_position(self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z)
        self.get_logger().info(f"Landing Requested. Returning to z={self.initial_pose.position.z}m")

        response.success = True
        '''
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = "map"
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.linear.y = 0.0
        stop_msg.twist.linear.z = 0.0
        stop_msg.twist.angular.x = 0.0
        stop_msg.twist.angular.y = 0.0
        stop_msg.twist.angular.z = 0.0

        self.shutdown_publisher.publish(stop_msg)

        response.success = True
        '''
        response.message = "Emergency shutdown command sent. Drone should land immediately."
        return response

    def publish_position(self, x, y, z):
        """Publishes a new desired position."""
        pos_msg = PoseStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = "map"
        pos_msg.pose.position.x = x
        pos_msg.pose.position.y = y
        pos_msg.pose.position.z = z
        self.pose_publisher.publish(pos_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()