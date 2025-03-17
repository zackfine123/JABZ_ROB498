import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

STATE = 'Init'
WAYPOINTS = None
WAYPOINTS_RECEIVED = False
INITIAL_RECEIVED = False
OFFSET_RECEIVED = False
WAYPOINT_INDEX = 0
OFFSET = np.zeros(3)
INITIAL_POSITION = np.zeros(3)
SETPOINT_POSITION = np.zeros(3)

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_9')  # Change 00 to your team ID
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Service servers
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_9/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_9/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_9/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_9/comm/abort', self.callback_abort)
        
        # Subscribers
        self.sub_waypoints = self.create_subscription(PoseArray, 'rob498_drone_9/comm/waypoints', self.callback_waypoints, qos_profile)
        self.sub_vision_pose = self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.callback_vision_pose, qos_profile)
        # Subscriber to "/vicon/ROB498_Drone/ROB498_Drone" (PoseStamped)
        self.subscription = self.create_subscription(PoseStamped,'/vicon/ROB498_Drone/ROB498_Drone',self.get_offset,10)
        # Publisher for drone setpoints
        self.pub_setpoint = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)
        
        # Timer for execution loop
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Comm Node Initialized")
    
    def callback_launch(self, request, response):
        global STATE
        STATE = 'Launch'
        self.get_logger().info("Launch Requested.")
        response.success = True
        response.message = "Drone taking off."
        return response
    
    def get_offset(self, msg):
        global OFFSET, OFFSET_RECEIVED, STATE
        vicon_pose = msg.pose.position
        vision_origin = self.get_current_vision_pose()
        if (vision_origin is not None) and (STATE is not "Test"):
            if OFFSET_RECEIVED:
                OFFSET = (0.7*(np.array([vicon_pose.x, vicon_pose.y, vicon_pose.z]) - vision_origin) + 0.3*OFFSET) # weighted average
            else:
                OFFSET = np.array([vicon_pose.x, vicon_pose.y, vicon_pose.z]) - vision_origin
                OFFSET_RECEIVED = True
            self.get_logger().info(f"Computed Offset: {OFFSET}")

    def callback_test(self, request, response):
        global STATE
        STATE = 'Test'
        self.get_logger().info("Test Requested. Following waypoints.")
        response.success = True
        response.message = "Test Received."
        return response
    
    def callback_land(self, request, response):
        global STATE
        STATE = 'Land'
        self.get_logger().info("Land Requested.")
        response.success = True
        response.message = "Drone is landing."
        return response
    
    def callback_abort(self, request, response):
        global STATE
        STATE = 'Abort'
        self.get_logger().warn("Abort Requested. Emergency landing. Cutting power.")
        response.success = True
        response.message = "ABORT"
        return response
    
    def callback_waypoints(self, msg):
        global WAYPOINTS, WAYPOINTS_RECEIVED
        if WAYPOINTS_RECEIVED:
            return
        
        self.get_logger().info("Waypoints Received.")
        WAYPOINTS_RECEIVED = True
        WAYPOINTS = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in msg.poses])
    
    def callback_vision_pose(self, msg):
        global INITIAL_POSITION, INITIAL_RECEIVED
        self.vision_origin = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if not INITIAL_RECEIVED:
            self.get_logger().info("Initial Set.")
            INITIAL_POSITION = self.vision_origin
            INITIAL_RECEIVED = True
        
    
    def get_current_vision_pose(self):
        return getattr(self, 'vision_origin', None)
    
    def control_loop(self):
        global STATE, WAYPOINTS, WAYPOINT_INDEX, WAYPOINTS_RECEIVED, OFFSET, INITIAL_POSITION, SETPOINT_POSITION

        if STATE == 'Launch':
            SETPOINT_POSITION = INITIAL_POSITION + np.array([0, 0, 1.5])

        elif STATE == 'Test' and WAYPOINTS_RECEIVED:
            if WAYPOINT_INDEX < len(WAYPOINTS):
                SETPOINT_POSITION = WAYPOINTS[WAYPOINT_INDEX] - OFFSET
                self.get_logger().info(f"Moving to waypoint {WAYPOINT_INDEX + 1}/{len(WAYPOINTS)}: {SETPOINT_POSITION}")
                if self.reached_setpoint(SETPOINT_POSITION):
                    WAYPOINT_INDEX += 1
            else:
                self.get_logger().info("All waypoints reached. Holding position.")
                SETPOINT_POSITION = INITIAL_POSITION + np.array([0, 0, 1.5])
                WAYPOINT_INDEX = 0
                STATE = 'Hover'
        
        elif STATE == 'Land':
            current_position = self.get_current_vision_pose()
            SETPOINT_POSITION = np.array([current_position[0], current_position[1], -0.1])
        
        elif STATE == 'Abort':
            self.get_logger().error("Emergency Stop! Cutting power.")
            current_position = self.get_current_vision_pose()
            SETPOINT_POSITION = np.array([current_position[0], current_position[1], -0.1])

        elif STATE == 'Hover':
            current_position = self.get_current_vision_pose()
            SETPOINT_POSITION = np.array(current_position)

        self.publish_setpoint(SETPOINT_POSITION)
    
    def publish_setpoint(self, position):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        self.pub_setpoint.publish(pose_msg)
    
    def reached_setpoint(self, target_position, threshold=0.15):
        current_position = self.get_current_vision_pose()
        return (current_position is not None) and (np.linalg.norm(current_position - target_position) < threshold)

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
