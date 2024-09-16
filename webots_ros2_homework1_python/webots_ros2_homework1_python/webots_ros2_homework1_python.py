# import the ROS2 API python libraries
import rclpy
# import the ROS2 Node API python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import the Odometry module from nav_msgs interface
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

import math

LINEAR_VEL_FORWARD   = 0.15
LINEAR_VEL_TURNING   = 0.07
STOP_DISTANCE        = 0.2
LIDAR_ERROR          = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE   = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX     = 270
RIGHT_FRONT_INDEX    = 210
LEFT_FRONT_INDEX     = 150
LEFT_SIDE_INDEX      = 90

class WallFollower(Node):

    def __init__(self):
        # Initialize the publisher via parent class
        super().__init__('wall_follower_node')

        # Initialize values used for heuristic navigation
        self.scan_cleaned     = []
        self.stall            = False
        self.turtlebot_moving = False
        self.laser_forward    = 0
        self.odom_data        = 0
        self.pose_saved       = ''
        self.cmd              = Twist()

        # Initialize the publisher for velcity commands
        self.publisher_  = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Initialize the subscription to laser data
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Initialize the subscription to odometry data
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Amount of time between checks for updates
        timer_period = 0.5
        # The time that will cause the callback to execute
        self.timer   = self.create_timer(timer_period, self.timer_callback)

    def listener_callback1(self, msg1):
        '''
        Callback that executes upon new LaserScan data being received. Stores
        the new data in instance variables for use.
        Parameters:
        -----------
            msg1: The msg received via the LaserScan subscription.
        '''
        # Erase stale laser range data
        self.scan_cleaned = []

        # Parse the laser data from the message
        scan = msg1.ranges

        # Loop through range data for cleaning (assumes 360 degrees)
        for reading in scan:
            # If the reading is infiinty set to maximum range
            if float('Inf') == reading:
                self.scan_cleaned.append(3.5)
            # If the reading is NaN assume range is zero
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            # Otherwise assume reading is valid
            else:
                self.scan_cleaned.append(reading)

        return

    def listener_callback2(self, msg2):
        '''
        Callback that executes upon new Odometry data being received. Stores
        the new data in instance varaible for use.
        Parameters:
        -----------
            msg2: The msg revieved via the Odometry subscription.
        '''
        # Get the current position and orientation information
        position    = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation

        # Parse out the data from the vectors
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw)   = (orientation.x, orientation.y, orientation.z, orientation.w)

        # Save the current position data for future use
        self.pose_saved = position

        return

    def timer_callback(self):
        '''
        Callback function that will executed once during the specified time
        period. This will use the laser and odometry data to determine what
        the robot should do.
        '''
        # If we have not yey received any laser data exit immediantely
        if (len(self.scan_cleaned) == 0):
            self.turtlebot_moving = False
            return

        # Get the minimum detected range from the front left region
        left_lidar_min  = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        # Get the minimum detected range from the front right region
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        # Get the minimum detected range from the front region
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        # If something is detected directly in front of the robot stop
        if front_lidar_min < SAFE_STOP_DISTANCE:

            if self.turtlebot_moving == True:

                self.get_logger().info('Stopping')

                self.turtlebot_moving = False
                self.cmd.linear.x     = 0.0
                self.cmd.angular.z    = 0.0
                self.publisher_.publish(self.cmd)

                return

        # If something is detected distantly in front of the robot, attempt
        # to avoid it
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:

            self.get_logger().info('Turning')

            self.turtlebot_moving = True
            self.cmd.linear.x     = LINEAR_VEL_TURNING

            if (right_lidar_min > left_lidar_min):
                self.cmd.angular.z = -0.3
            else:
                self.cmd.angular.z = 0.3

            self.publisher_.publish(self.cmd)

        # If nothing is detected, simply move forward
        else:

            self.get_logger().info('Continue Moving')

            self.turtlebot_moving = True
            self.cmd.linear.x     = LINEAR_VEL_FORWARD
            self.cmd.linear.z     = 0.0
            self.publisher_.publish(self.cmd)

        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I received: "%s"' % str(self.odom_data))

        if self.stall == True:
            self.get_logger().info('Stall reported')

        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_follower_node = WallFollower()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(wall_follower_node)
    # Explicity destroy the node
    wall_follower_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
