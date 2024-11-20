import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_msgs.msg import TFMessage
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90

class RandomWalk(Node):

    def __init__(self):
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.apriltag_detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.apriltag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.laser_forward = 0
        self.odom_data = 0
        self.timer = self.create_timer(0.1, self.timer_callback)  # Faster callback interval
        self.pose_saved = ''
        self.cmd = Twist()
        self.apriltag_detected = False
        self.apriltag_id = None

    def listener_callback1(self, msg1):
        # Clean lidar data by replacing invalid values with a maximum range
        self.scan_cleaned = []
        for reading in msg1.ranges:
            if reading == float('Inf') or math.isnan(reading) or reading <= 0.05:
                self.scan_cleaned.append(3.5)  # Assume maximum range
            else:
                self.scan_cleaned.append(reading)

        # Log cleaned lidar data for debugging
        self.get_logger().info(f"Cleaned Lidar Data (first 10): {self.scan_cleaned[:10]}")

    def listener_callback2(self, msg2):
        # Log robot position from odometry
        position = msg2.pose.pose.position
        self.pose_saved = position
        self.get_logger().info(f'Self position: x={position.x}, y={position.y}, z={position.z}')

    def apriltag_callback(self, msg):
        # Handle AprilTag detections
        if not msg.detections:
            self.get_logger().info('No AprilTag detected.')
            return

        for detection in msg.detections:
            tag_id = detection.id
            centre = detection.centre
            self.get_logger().info(
                f'AprilTag Detected - ID: {tag_id}, Centre: [x: {centre.x}, y: {centre.y}]')
            self.apriltag_detected = True
            self.apriltag_id = tag_id

    def timer_callback(self):
        if len(self.scan_cleaned) == 0:
            self.get_logger().info("No lidar data available!")
            return

        # Calculate distances for front, left, and right regions
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])

        # Log distances for debugging
        self.get_logger().info(f"Front: {front_lidar_min}, Left: {left_lidar_min}, Right: {right_lidar_min}")

        # Obstacle avoidance logic
        if front_lidar_min < SAFE_STOP_DISTANCE:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Stopping: Obstacle too close')
        elif SAFE_STOP_DISTANCE <= front_lidar_min < LIDAR_AVOID_DISTANCE:
            self.cmd.linear.x = 0.07
            self.cmd.angular.z = -0.3 if right_lidar_min > left_lidar_min else 0.3
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Turning: Avoiding obstacle')
        else:
            self.cmd.linear.x = 0.3
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Moving Forward: Path is clear')

def main(args=None):
    rclpy.init(args=args)
    random_walk_node = RandomWalk()
    rclpy.spin(random_walk_node)
    random_walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
