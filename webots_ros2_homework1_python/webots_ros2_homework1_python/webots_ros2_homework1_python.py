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
from   enum import Enum, auto

# the number of duplicate positions before entering error mode
MAX_DUP_POSITIONS                  = 10

# distance at which to detect walls and objects
FORWARD_DETECT_RANGE               = 0.6
LEFT_DETECT_RANGE                  = 0.2
RIGHT_DETECT_RANGE                 = 0.6
BACKWARD_DETECT_RANGE              = 0.2

# forward speed configuration
NORMAL_FORWARD_LINEAR_SPEED        = 0.1
NORMAL_FORWARD_ANGULAR_SPEED       = 0.0

# turning left speed configuration
NORMAL_TURNING_LEFT_LINEAR_SPEED   = 0.1
NORMAL_TURNING_LEFT_ANGULAR_SPEED  = 0.5

# turning right speed configuration
NORMAL_TURNING_RIGHT_LINEAR_SPEED  = 0.1
NORMAL_TURNING_RIGHT_ANGULAR_SPEED = -0.5

# backward speed configuration
NORMAL_BACKWARD_LINEAR_SPEED       = -0.1
NORMAL_BACKWARD_ANGULAR_SPEED      = 0.0

# struck speed configuration
STUCK_LINEAR_SPEED                 = 0.0
STUCK_ANGULAR_SPEED                = 0.0

# error detection forward speed configuration
ERROR_FORWARD_LINEAR_SPEED         = 0.1
ERROR_FORWARD_ANGULAR_SPEED        = 0.0

# error detection left speed configuration
ERROR_LEFT_LINEAR_SPEED           = 0.075
ERROR_LEFT_ANGULAR_SPEED          = 0.5

# error detection backward speed configuration
ERROR_BACKWARD_LINEAR_SPEED       = -0.1
ERROR_BACKWARD_ANGULAR_SPEED      = 0.0

# The indexes of various position in the laser scan array
INITIAL_BACK_INDEX = 0
BACK_LEFT_INDEX    = 45
LEFT_INDEX         = 90
FRONT_LEFT_INDEX   = 135
FRONT_INDEX        = 180
FRONT_RIGHT_INDEX  = 225
RIGHT_INDEX        = 270
BACK_RIGHT_INDEX   = 315
FINAL_BACK_INDEX   = 359

class WallFollowingStates(Enum):
    # nothing is detected, following wall, or moving past object
    STATE_MOVE_FORWARD = auto()
    # turning around a corner due to wall or object
    STATE_TURN_RIGHT    = auto()
    # reached a corner or moving around an object
    STATE_TURN_LEFT     = auto()
    # reached a dead end only option is to go backward
    STATE_MOVE_BACKWARD = auto()
    # attempted to go backward unsuccessfully, robot is stuck
    STATE_STUCK         = auto()

class WallFollower(Node):

    def __init__(self):
        '''
        The initializing function of the wall follower node.
        '''
        # Initialize the publisher via parent class
        super().__init__('wall_follower_node')

        # Initialize values used for heuristic navigation
        self.scan_cleaned  = []
        self.pose          = ''

        self.original_duplicate_pose = ''
        self.original_duplicate_scan = []

        self.duplicate_count = 0
        self.recovery_cycles = 0

        self.cmd = Twist()

        self.state = WallFollowingStates.STATE_MOVE_FORWARD

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
        Returns:
        --------
            None
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
        Returns:
        --------
            None
        '''

        # Get the current position and orientation information
        position    = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation

        # Parse out the data from the vectors
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw)   = (orientation.x, orientation.y, orientation.z, orientation.w)

        # Save the current position data for future use
        self.pose = position

        return

    def object_detected(self, min_foward_lidar, min_left_lidar, min_right_lidar, min_backward_lidar):
        '''
        Determines if an object is detected within the specified range or not.
        '''
        b_forward_object  = min_foward_lidar   < FORWARD_DETECT_RANGE
        b_left_object     = min_left_lidar     < LEFT_DETECT_RANGE
        b_right_object    = min_right_lidar    < RIGHT_DETECT_RANGE
        b_backward_object = min_backward_lidar < BACKWARD_DETECT_RANGE

        return (b_forward_object, b_left_object, b_right_object, b_backward_object)

    def in_same_position(self):
        '''
        Checks to see if the position reported by the robot is the same as
        previously reported.

        Returns:
        --------
            bool: Whether or not the position is the same
        '''
        duplicate_x          = False
        duplicate_y          = False
        duplicate_laser_scan = False

        x_diff = math.fabs(self.pose.x - self.original_duplicate_pose.x)
        y_diff = math.fabs(self.pose.y - self.original_duplicate_pose.y)

        for idx in range(len(self.scan_cleaned)):
            if math.fabs(self.scan_cleaned[idx] - self.original_duplicate_scan[idx]) < 0.01:
                duplicate_laser_scan = True

        if duplicate_laser_scan:
            self.get_logger().info(f'Duplicate LaserScan value detecteds')
        if x_diff < 0.3:
            self.get_logger().info(f'Duplicate x value detected: {x_diff}')
            duplicate_x = True
        if y_diff < 0.3:
            self.get_logger().info(f'Duplicate y value detected: {y_diff}')
            duplicate_y = True

        if duplicate_x and duplicate_y and duplicate_laser_scan:
            return False

        return False

    def get_state(self):
        '''
        Determines what state the robot is in based on the laser scan data.

        @note This functions assumes the lidar data stored in instance variable
              is up to date and accurate.

        Returns:
        --------
            WallFollowingStates:
                An enum variable representing which state that the robot is
                currently in.
        '''
        # Get the lidar data for the different regions
        left_lidar_data       = self.scan_cleaned[BACK_LEFT_INDEX:FRONT_LEFT_INDEX]
        right_lidar_data      = self.scan_cleaned[FRONT_RIGHT_INDEX:BACK_RIGHT_INDEX]
        front_lidar_data      = self.scan_cleaned[FRONT_LEFT_INDEX:FRONT_RIGHT_INDEX]
        back_left_lidar_data  = self.scan_cleaned[INITIAL_BACK_INDEX:BACK_LEFT_INDEX]
        back_right_lidar_data = self.scan_cleaned[BACK_RIGHT_INDEX:FINAL_BACK_INDEX]
        back_lidar_data       = back_left_lidar_data + back_right_lidar_data

        # Get the minimum data to determine robot's state
        min_left_lidar_data  = min(left_lidar_data)
        min_right_lidar_data = min(right_lidar_data)
        min_front_lidar_data = min(front_lidar_data)
        min_back_lidar_data  = min(back_lidar_data)

        # Determine where objects are detected
        b_forward_object, b_left_object, b_right_object, b_backward_object = self.object_detected(
            min_front_lidar_data,
            min_left_lidar_data,
            min_right_lidar_data,
            min_back_lidar_data
        )

        # Obstacle in all regions
        if b_left_object and b_right_object and b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in all regions')
            return WallFollowingStates.STATE_STUCK

        # Obstacle in no regions
        if not b_left_object and not b_right_object and not b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in no regions')
            return WallFollowingStates.STATE_MOVE_FORWARD

        # Obstacle in left, right, and front regions
        elif b_left_object and b_right_object and b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in left, right, and front regions')
            return WallFollowingStates.STATE_MOVE_BACKWARD

        # Obstacle in left, right, and back regions
        elif b_left_object and b_right_object and not b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in left, right, and back regions')
            return WallFollowingStates.STATE_MOVE_FORWARD

        # Obstacle in right, front, and back regions
        elif not b_left_object and b_right_object and b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in right, front, and back regions')
            return WallFollowingStates.STATE_TURN_LEFT

        # Obstacle in left, front, and back regions
        elif b_left_object and not b_right_object and b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in left, front, and back regions')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in left and right region
        elif b_left_object and b_right_object and not b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in left and right regions')
            return WallFollowingStates.STATE_MOVE_FORWARD

        # Obstacle in front and right region
        elif not b_left_object and b_right_object and b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in front and right regions')
            return WallFollowingStates.STATE_TURN_LEFT

        # Obstacle in front and left region
        elif b_left_object and not b_right_object and b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in front and left regions')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in front and back region
        elif not b_left_object and not b_right_object and b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in front and back regions')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in back and left region
        elif b_left_object and not b_right_object and not b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in back and left region')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in back and right region
        elif not b_left_object and b_right_object and not b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in back and right region')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in front region only
        elif not b_left_object and not b_right_object and b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in front region')
            return WallFollowingStates.STATE_TURN_LEFT

        # Obstacle in back region only
        elif not b_left_object and not b_right_object and not b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in back region')
            return WallFollowingStates.STATE_MOVE_FORWARD

        # Obstacle in right region only
        elif not b_left_object and b_right_object and not b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in right region')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in left region only
        elif b_left_object and not b_right_object and not b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in left region')
            return WallFollowingStates.STATE_TURN_RIGHT

        return WallFollowingStates.STATE_STUCK

    def timer_callback(self):
        '''
        Callback function that will execute once during the specified time
        period. This will collect the laser and odometry data before calling
        the current state's function.
        '''
        # if we have not yey received any laser data exit immediantely
        if (len(self.scan_cleaned) == 0) or (self.pose == ''):
            return

        # if the same position has not been detected 10 times, continue execution
        # as normal
        if self.duplicate_count < 10:

            # If there is not currently an original position dupilicate, set it
            # to the current position value
            if self.original_duplicate_pose == '':
                self.original_duplicate_pose = self.pose

            # if there is not currently an original laser scan duplcate, set it
            # to the current laser scan
            if self.original_duplicate_scan == []:
                self.original_duplicate_scan = self.scan_cleaned

            # check if we are in the same position based on the position and
            # laser scan values.
            if self.in_same_position():
                self.get_logger().info('Duplicate Position Detected')
                self.recovery_cycles  = 20
                self.duplicate_count += 1

            # if we are not in the same position reset the stored duplicate
            # values and count
            else:
                self.get_logger().info('Reseting Duplicate Counter')
                self.original_duplicate_pose = ''
                self.original_duplicate_scan = []
                self.duplicate_count         = 0

            # determine which state we are in
            self.state = self.get_state()

            if WallFollowingStates.STATE_MOVE_FORWARD == self.state:
                self.get_logger().info('Entering Move Forward State')
                self.cmd.linear.x  = NORMAL_FORWARD_LINEAR_SPEED
                self.cmd.angular.z = NORMAL_FORWARD_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            elif WallFollowingStates.STATE_MOVE_BACKWARD == self.state:
                self.get_logger().info('Entering Move Backward State')
                self.cmd.linear.x  = NORMAL_BACKWARD_LINEAR_SPEED
                self.cmd.angular.z = NORMAL_BACKWARD_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            elif WallFollowingStates.STATE_TURN_RIGHT == self.state:
                self.get_logger().info('Entering Turn Right State')
                self.cmd.linear.x  = NORMAL_TURNING_RIGHT_LINEAR_SPEED
                self.cmd.angular.z = NORMAL_TURNING_RIGHT_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            elif WallFollowingStates.STATE_TURN_LEFT == self.state:
                self.get_logger().info('Entering Turn Left State')
                self.cmd.linear.x  = NORMAL_TURNING_LEFT_LINEAR_SPEED
                self.cmd.angular.z = NORMAL_TURNING_LEFT_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            elif WallFollowingStates.STATE_STUCK == self.state:
                self.get_logger().info('Entering Stuck State')
                self.cmd.linear.x  = STUCK_LINEAR_SPEED
                self.cmd.angular.z = STUCK_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            else:
                self.get_logger().info('Invalid State Detected!')

        # 10 duplicate positions have been detected, enter revovery mode
        else:
            self.recovery_cycles -= 1

            if self.recovery_cycles == 0:
                self.get_logger().info('Exiting Recovery State')
                self.duplicate_count         = 0
                self.original_duplicate_pose = ''
                self.original_duplicate_scan = []

            elif self.recovery_cycles > 15:
                self.get_logger().info('Move Backward to Attempt to Free Robot')
                self.cmd.linear.x  = ERROR_BACKWARD_LINEAR_SPEED
                self.cmd.angular.z = ERROR_BACKWARD_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            elif self.recovery_cycles > 10:
                self.get_logger().info('Turn Left to Attempt to Free Robot')
                self.cmd.linear.x  = ERROR_LEFT_LINEAR_SPEED
                self.cmd.angular.z = ERROR_LEFT_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            else:
                self.get_logger().info('Move Forward to Attempt to Free Robot')
                self.cmd.linear.x  = ERROR_FORWARD_LINEAR_SPEED
                self.cmd.angular.z = ERROR_FORWARD_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

        return

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
