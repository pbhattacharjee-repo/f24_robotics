import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_msgs.msg import TFMessage
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

# Constants for movement and distance thresholds
LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
# Lidar scan index constants for direction analysis
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90

class RandomWalk(Node):
    def __init__(self):
        # Initialize the RandomWalk node
        super().__init__('random_walk_node')
        
        # Variables to track sensor data and robot state
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        
        # Publisher to send velocity commands to the robot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber to listen to LaserScan messages for obstacle detection
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # Subscriber to listen to Odometry messages for position tracking
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # Subscriber to listen for AprilTag detections
        self.apriltag_detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.apriltag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        # Initializing variables to hold sensor and control data
        self.laser_forward = 0
        self.odom_data = 0
        self.timer = self.create_timer(0.5, self.timer_callback)  # Timer for periodic actions
        self.pose_saved = ''
        self.cmd = Twist()  # To hold velocity commands
        self.apriltag_detected = False
        self.apriltag_id = None

    def listener_callback1(self, msg1):
        # Callback for processing LaserScan data
        scan = msg1.ranges
        self.scan_cleaned = []
        for reading in scan:
            if reading == float('Inf'):
                # Replace infinite readings with a large value
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                # Replace NaN readings with zero
                self.scan_cleaned.append(0.0)
            else:
                # Keep valid readings as-is
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        # Callback for processing Odometry data
        position = msg2.pose.pose.position
        self.pose_saved = position
        self.get_logger().info('Self position: {},{},{}'.format(
            position.x, position.y, position.z))

    def apriltag_callback(self, msg):
        # Callback for AprilTag detections
        if not msg.detections:
            self.get_logger().info('No AprilTag detected.')
            return

        for detection in msg.detections:
            tag_id = detection.id  # Extract AprilTag ID
            centre = detection.centre  # Tag center coordinates
            corners = detection.corners  # Tag corner coordinates

            self.get_logger().info(
                f'AprilTag Detected - ID: {tag_id}, Centre: [x: {centre.x}, y: {centre.y}]')
            self.apriltag_detected = True
            self.apriltag_id = tag_id

    def timer_callback(self):
        # Periodic callback for decision-making based on sensor data
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return
        
        # Determine minimum distances for left, right, and front lidar scans
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        # Behavior based on lidar readings
        if front_lidar_min < SAFE_STOP_DISTANCE:
            # Stop if an obstacle is very close
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Stopping')
            self.turtlebot_moving = False
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            # Slow down and turn if an obstacle is nearby
            self.cmd.linear.x = 0.07
            if right_lidar_min > left_lidar_min:
                # Turn left if the obstacle is closer on the right
                self.cmd.angular.z = -0.3
            else:
                # Turn right if the obstacle is closer on the left
                self.cmd.angular.z = 0.3
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Turning')
            self.turtlebot_moving = True
        else:
            # Move forward if the path is clear
            self.cmd.linear.x = 0.3
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True

        self.get_logger().info('Distance of the obstacle: %f' % front_lidar_min)

def main(args=None):
    # Main function to initialize and run the node
    rclpy.init(args=args)
    random_walk_node = RandomWalk()
    rclpy.spin(random_walk_node)  # Keep the node running
    random_walk_node.destroy_node()  # Clean up when done
    rclpy.shutdown()  # Shutdown the ROS client library

if __name__ == '__main__':
    main()
