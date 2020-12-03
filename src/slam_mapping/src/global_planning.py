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
        waypoints = []

        # (x, y)
        curr_waypoint = None

        # (x, y)
        target = None

        # (x, y, yaw)
        odom_ = None
    
    def lidar_callback(self, scan):
        """
        Callback for the LIDAR subscriber.
        Reads in msg and tries to continue on to the currently selected waypoint
        """


    
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
