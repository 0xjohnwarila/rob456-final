import rospy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class GlobalPlanner:
    """
    Driver for global exploration and planning, with A* path finding.
    """

    def __init__(self):
        """
        Initialize SLAM object's publishers, subscribers, and internal state
        """
        # Setup Subscribers

        # SLAM map subscriber
        self.gmap_sub_ = rospy.Subscriber(
            'gmap',
            OccupancyGrid,
            self.gmap_callback
        )
        # LIDAR scan
        self.lidar_sub_ = rospy.Subscriber(
            '/scan',
            LaserScan,
            self.lidar_callback
        )
        # Odometry
        self.odom_sub_ = rospy.Subscriber(
            '/odom',
            Odometry,
            self.odom_callback
        )

        # Setup Publishers

        # Commanded Velocity Publisher
        self.twist_pub_ = rospy.Publisher('/cmd_vel', Twist, queuesize=10)

        # Setup Internal State
        map_ = {}

        # [(x, y)]
        waypoints_ = []

        # (x, y)
        curr_waypoint_ = None

        # (x, y)
        target_ = None

        # (x, y, yaw)
        odom_ = None
    
    def lidar_callback(self, msg):
        """
        Callback for the LIDAR subscriber.
        Reads in msg and tries to continue on to the currently selected waypoint
        :param: msg: lidar msg
        :returns: None
        """

        # Check if we have a waypoint to navigate to
        if self.curr_waypoint_ == None:
            # Get the next waypoint
            self.get_new_waypoint()

        command = Twist()

        # Fill in the fields.  Field values are unspecified 
        # until they are actually assigned. The Twist message 
        # holds linear and angular velocities.
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0

        # Lidar properties (unpacked for your ease of use)
        # find current laser angle, max scan length, distance array for all
        # scans, and number of laser scans
        max_angle = msg.angle_max
        min_angle = msg.angle_min
        angle_increment = msg.angle_increment

        max_scan_length = msg.range_max
        distances = msg.ranges
        num_scans = len(distances)

        x_dist = self.curr_waypoint_[0] - self.odom_[0]
        y_dist = self.curr_waypoint_[1] - self.odom_[1]
        total_distance = np.sqrt(x_dist ** 2 + y_dist ** 2)

        # I subtracted the odometer angle from the arctan angle so I can use
        # that to only turn as much as I need to
        # (PS: Why oh why would the numpy people write a function that flipped
        # x and y like that? It was the root of all my problems for so long)
        #
        # ~ Pretty sure it is to maintain the same function signature as the C
        #   method atan()  - John
        angle = np.arctan2(y_dist, x_dist) - self.odom_[2]

        # If the target point is behind the robot then it will only turn, once
        # it's <90 degrees from the front it will start driving forward
        if angle > np.pi / 2 or angle < - np.pi / 2:
            command.linear.x = 0
            command.angular.z = angle * .3
        else:
            command.linear.x = 0.1
            command.angular.z = angle * .3
        # Stop the robot when position is reached
        if total_distance < 0.1:
            command.linear.x = 0
            command.linear.z = 0
            self.curr_waypoint_ = None

        current_laser_theta = min_angle
        for i, scan in enumerate(distances):
            half_angle = 25
            # This if statement basically checks to see if we're close to a
            # wall, then checks if that wall is within a 50 degree angle in
            # front of us and we're moving towards it. If all conditions are
            # satisfied then we do a hard left or right depending on whether
            # the wall is closer to the left or right front
            if scan < 0.5:
                if 360 - half_angle < i < 360 and command.linear.x > 0:
                    command.linear.x = 0
                    command.angular.z = 1
                elif 0 <= i < 0 + half_angle and command.linear.x > 0:
                    command.linear.x = 0
                    command.angular.z = -1
            current_laser_theta = current_laser_theta + angle_incr
    
    pub.publish(command)
    
    def odom_callback(self, msg):
        """
        Callback for the odom subscriber.
        Reads msgs and saves to the internal state.
        
        :param: msg: Odometry message
        "returns: None
        """
        position = msg.pose.position
        ori = msg.pose.pose.orientation
        (r, p, yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.odom_ = (position.x, position.y, yaw)

    def gmap_callback(self, gmap):
        pass

if __name__ == '__main__':
    rospy.init_node('global_planner')
    GlobalPlanner()
    # ROS go brrrr
    rospy.spin()
