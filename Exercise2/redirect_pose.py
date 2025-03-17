import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TransformToPoseRedirectNode(Node):
    def __init__(self):
        super().__init__('transform_to_pose_redirector')
        self.sys = 'rs'
        if self.sys == 'vicon':
            # Subscriber to "/vicon/ROB498_Drone/ROB498_Drone" (PoseStamped)
            self.subscription = self.create_subscription(
                PoseStamped,
                '/vicon/ROB498_Drone/ROB498_Drone',
                self.transform_callback,
                10  # Queue size
            )
        else:
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Force best effort mode
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
            self.subscription = self.create_subscription(
                Odometry,
                '/camera/pose/sample',
                self.transform_callback,
                qos_profile
                #100  # Queue size
                
            )
        self.subscription

        # switch to /camera/pose/sample for realsense
        # /vicon/ROB498_Drone/ROB498_Drone for vicon

        # Publisher to "/mavros/vision_pose/pose" (PoseStamped)
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            100  # Queue size
        )

        self.get_logger().info("Transform to Pose Redirect Node has started.")

    def transform_callback(self, msg):
        """Callback function to republish."""
        self.get_logger().info("Received transform, redirecting as PoseStamped")
        pose_msg = PoseStamped()
        pose_msg.header = msg.header  # Preserve timestamp and frame info
        # change from vicon timestamp to system timestamp later in case jetson is lagging
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        if self.sys == 'vicon':
            pose_msg.pose.position.x = msg.pose.position.x
            pose_msg.pose.position.y = msg.pose.position.y
            pose_msg.pose.position.z = msg.pose.position.z
            pose_msg.pose.orientation = msg.pose.orientation
        else:
            pose_msg.pose = msg.pose.pose

        #self.get_logger().info("Received transform, redirecting as PoseStamped")
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformToPoseRedirectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
