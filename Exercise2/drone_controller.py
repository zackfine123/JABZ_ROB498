import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Initial position setpoint
        self.posdes = PoseStamped()
        self.posdes.pose.position.x = 0.0
        self.posdes.pose.position.y = 0.0
        self.posdes.pose.position.z = 0.0  # Will be updated

        # Publisher for position setpoints
        self.pose_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Publisher for emergency shutdown (zero thrust)
        self.vel_publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # Subscriber for setpoint updates
        self.subscription = self.create_subscription(
            PoseStamped,
            '/rob498/posdes',
            self.posdes_callback,
            10
        )

        # Subscriber for emergency shutdown
        self.subscription_shutdown = self.create_subscription(
            TwistStamped,
            '/rob498/emergency_shutdown',
            self.emergency_shutdown_callback,
            10
        )

        # Timer to continuously publish setpoints
        self.timer = self.create_timer(0.1, self.publish_setpoint)  # 10 Hz

        self.get_logger().info("Drone Controller Initialized.")

    def publish_setpoint(self):
        """Publishes the desired position at a steady rate."""
        self.posdes.header.stamp = self.get_clock().now().to_msg()
        self.posdes.header.frame_id = "map"
        self.pose_publisher.publish(self.posdes)

    def posdes_callback(self, msg):
        """Updates position setpoint when received from CommNode."""
        self.get_logger().info(f"Updating setpoint to: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})")
        self.posdes = msg  # Store new desired position

    def emergency_shutdown_callback(self, msg):
        """Receives zero-velocity emergency command and applies it."""
        self.get_logger().warn("Emergency shutdown received! Cutting power.")
        self.vel_publisher.publish(msg)  # Send zero thrust command

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
