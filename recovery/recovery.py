import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Vector3
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

STATE = 'Init'
WAYPOINTS = np.array([[0,0,1],
                    [1, 0, 1],
                    [1, 1, 1],
                    [-1.5, 1, 1],
                    [0,0,1],
                    [1.5, 0, 1],
                    [1.5, 1, 1],
                    [-1.5, 1, 1]]) # search pattern
WAYPOINTS_RECEIVED = True
INITIAL_RECEIVED = False
DETECTED = False
WAYPOINT_INDEX = 0
KP_rec = 0.05
KP_search = 0.05
INITIAL_POSITION = np.zeros(3)
SETPOINT_POSITION = np.zeros(3)
wait = 0

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
        self.sub_vision_pose = self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.callback_vision_pose, qos_profile)
        self.sub_vs_control = self.create_subscription(Vector3, 'rob498_drone_9/recovery/servo_command', self.callback_vs_dir, 10)
        self.sub_detect_target = self.create_subscription(Bool, 'rob498_drone_9/recovery/target_detected', self.callback_target_detect, 10)
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

    def callback_vision_pose(self, msg):
        global INITIAL_POSITION, INITIAL_RECEIVED
        self.vision_origin = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if not INITIAL_RECEIVED:
            self.get_logger().info("Initial Set.")
            INITIAL_POSITION = self.vision_origin
            INITIAL_RECEIVED = True

    def get_current_vision_pose(self):
        return getattr(self, 'vision_origin', None)
    
    def callback_vs_dir(self, msg):
        self.control_dir = np.array([msg.x, msg.y, 0])

    def get_current_control_dir(self):
        return getattr(self, 'control_dir', np.array([0,0,0]))
    
    def callback_target_detect(self, msg):
        global DETECTED
        DETECTED = msg.data
    
    def control_loop(self):
        global STATE, WAYPOINTS, WAYPOINT_INDEX, WAYPOINTS_RECEIVED, INITIAL_POSITION, SETPOINT_POSITION, KP_rec, KP_search, current_position, detected_pos, wait

        if STATE == 'Launch':
            SETPOINT_POSITION = np.array([0, 0, 1.5])

        elif STATE == 'Test':
            if DETECTED:
                self.get_logger().info("Target detected, attempting Flyover.")
                detected_pos = self.get_current_vision_pose()
                STATE = "Flyover"
            
            elif WAYPOINT_INDEX < len(WAYPOINTS):
                current_position = self.get_current_vision_pose()
                d = (WAYPOINTS[WAYPOINT_INDEX] - current_position)
                d = d/np.linalg.norm(d)
                SETPOINT_POSITION = current_position + KP_search * d
                if self.reached_setpoint(WAYPOINTS[WAYPOINT_INDEX]):
                    self.get_logger().info(f"Moving to waypoint {WAYPOINT_INDEX + 1}/{len(WAYPOINTS)}: {SETPOINT_POSITION}")
                    WAYPOINT_INDEX += 1
            else:
                self.get_logger().info("All waypoints reached, no target found.")
                WAYPOINT_INDEX = 0
                STATE = 'Land'

        elif STATE == "Flyover":
            if DETECTED:
                current_position = self.get_current_vision_pose()
                control = self.get_current_control_dir()
                if np.max(control) > 0.05: # tune to determine how close is close enough
                    SETPOINT_POSITION = current_position + KP_rec * control
                else:
                    SETPOINT_POSITION = current_position
                    self.get_logger().info("Flyover complete, hooking.")
                    current_position = self.get_current_vision_pose()
                    detected_pos = self.get_current_vision_pose()
                    STATE = "Hook"
                wait = 0
            else:
                self.get_logger().info("Target lost.")
                SETPOINT_POSITION = detected_pos
                if self.reached_setpoint(SETPOINT_POSITION):
                    wait += 1
                    if wait >= 40:
                        WAYPOINT_INDEX = WAYPOINT_INDEX - 1
                        wait = 0
                        STATE = "Test"

        elif STATE == "Hook":
            SETPOINT_POSITION = np.array([current_position[1]-0.2, current_position[2], 0.15])
            if self.reached_setpoint(SETPOINT_POSITION):
                self.get_logger().info("Hooked, recovering.")
                current_position = self.get_current_vision_pose()
                STATE = "Recover"

        elif STATE == "Recover":
            SETPOINT_POSITION = np.array([current_position[1]+0.5, current_position[2], 0.15])
            if self.reached_setpoint(SETPOINT_POSITION):
                current_position = self.get_current_vision_pose()
                self.get_logger().info("lifting.")
                STATE = "Lift"
        
        elif STATE == "Lift":
            SETPOINT_POSITION = np.array([current_position[1], current_position[2], 1])
            if self.reached_setpoint(SETPOINT_POSITION):
                self.get_logger().info("heading to drop off.")
                STATE = "Return"

        elif STATE == "Return":
            SETPOINT_POSITION = np.array([1, 0, 1])
            if self.reached_setpoint(SETPOINT_POSITION):
                self.get_logger().info("Dropping.")
                STATE = "Drop Off"
        
        elif STATE == "Drop Off":
            current_position = self.get_current_vision_pose()
            SETPOINT_POSITION = np.array([current_position[1], current_position[2], 0.15])
            if self.reached_setpoint(SETPOINT_POSITION):
                self.get_logger().info("Lowered, unhooking.")
                current_position = self.get_current_vision_pose()
                STATE = "Unhook"

        elif STATE == "Unhook":
            SETPOINT_POSITION = np.array([current_position[1]-0.5, current_position[2], 0.15])
            if self.reached_setpoint(SETPOINT_POSITION):
                current_position = self.get_current_vision_pose()
                self.get_logger().info("Returning to home.")
                STATE = "Land"

        elif STATE == "RTH":
            SETPOINT_POSITION = current_position + np.array([0, 0, 1.5])
            if self.reached_setpoint(SETPOINT_POSITION):
                self.get_logger().info("Landing.")
                STATE = "Land"

        elif STATE == 'Land':
            SETPOINT_POSITION = np.array([0, 0, -0.1])
        
        elif STATE == 'Abort':
            self.get_logger().error("Emergency Stop! landing.")
            current_position = self.get_current_vision_pose()
            SETPOINT_POSITION = np.array([current_position[0], current_position[1], -0.1])

        elif STATE == 'Hover':
            current_position = self.get_current_vision_pose()
            SETPOINT_POSITION = np.array(current_position)

        self.publish_setpoint(INITIAL_POSITION + SETPOINT_POSITION)
    
    def publish_setpoint(self, position):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        self.pub_setpoint.publish(pose_msg)
    
    def reached_setpoint(self, target_position, threshold=0.1):
        global INITIAL_POSITION
        target_position = target_position + INITIAL_POSITION
        current_position = self.get_current_vision_pose()
        return (current_position is not None) and (np.linalg.norm(current_position - target_position) < threshold)
    


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
