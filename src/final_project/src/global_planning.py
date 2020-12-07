#!/usr/bin/env python2

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from points_to_rviz import draw_points


import astar
import mapreading


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
            '/map',
            OccupancyGrid,
            self.map_callback
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
        self.viz_pub_ = rospy.Publisher('/marker', Marker, queue_size=2)

        # Commanded Velocity Publisher
        self.twist_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Setup Internal State

        # [(x, y)]
        self.waypoints_ = []

        # (x, y)
        self.curr_waypoint_ = None

        # (x, y)
        self.target_ = (188, 174)

        # (x, y, yaw) in meters
        self.odom_ = None

        # (x, y) in map space
        self.coords_ = (90, 115)

    def lidar_callback(self, msg):
        """
        Callback for the LIDAR subscriber.
        Reads in msg and tries to continue on to the currently selected
        waypoint.
        :param: msg: lidar msg
        :returns: None
        """

        # Check if we have a waypoint to navigate to
        if self.curr_waypoint_ is None:
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
        # Still hate it - Matt
        angle = np.arctan2(y_dist, x_dist) - self.odom_[2]

        # Accounts for situations where the angle reads positive but really
        # should be negative (or vice versa)
        if angle > np.pi:
            angle = angle - 2 * np.pi
        elif angle < -np.pi:
            angle = angle + 2 * np.pi

        # If the target point is behind the robot then it will only turn, once
        # it's <90 degrees from the front it will start driving forward
        if angle > np.pi / 2 or angle < - np.pi / 2:
            command.linear.x = 0
            command.angular.z = angle * .3
        else:
            command.linear.x = 0.1
            command.angular.z = angle * .3
        # Stop the robot when position is reached
        if total_distance < 0.2:
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
            current_laser_theta = current_laser_theta + angle_increment

        self.twist_pub_.publish(command)

    def odom_callback(self, msg):
        """
        Callback for the odom subscriber.
        Reads msgs and saves to the internal state.

        :param: msg: Odometry message
        "returns: None
        """
        position = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        (r, p, yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.odom_ = (position.x, position.y, yaw)
        x = position.x * self.resolution_
        y = position.y * self.resolution_

        self.coords_ = (int(x), int(y))

    def map_callback(self, msg):
        """
        Callback for the mab subcriber.
        Gets an occupancy grid and converts it to a usable grid map.

        :param: msg: Occupancy Grid
        :returns: None
        """
        # data: int8[] in row-major order
        data = msg.data
        c = msg.info.width
        r = msg.info.height
        self.resolution_ = msg.info.resolution

        data = np.flip(np.reshape(data, (r, c)), 0)

        self.map_ = mapreading.live_threshold(data)

    def convert_point_np(self, point):
        """
        Convert a point from (x, y) to (y, x) or the other way.
        :param: point: (x, y)
        :returns: (y, x)
        """
        return (point[1], point[0])

    def convert_point_rviz(self, point):
        """
        Converts points from numpy array form to rviz coordinates with (0, 0) at the center of the map
        :param point: (y, x) coordinate
        :return: (x, y) coordinate shifted and flipped along the y axis
        """
        x_shift = np.size(self.map_, axis=1) / 2 - 1
        y_shift = np.size(self.map_, axis=0) / 2 - 1
        new_x = point[1] - x_shift
        new_y = point[0] - y_shift
        return (new_x, -new_y)

    def path_plan(self, start, target):
        """
        Plan a new path with A* from start to target
        :param: start: (x,y), target: (x,y)
        :returns: None
        """
        # Make sure to convert to np style first
        s = self.convert_point_np(start)
        t = self.convert_point_np(target)

        points, parents, size = astar.astar(self.map_, s, t)

        # Do stuff with the path, like get the waypoints
        path = []
        child = t
        i = 0
        while child != s:
            if i % 10 == 0:
                path.append(child)
            child = parents[child]
            i += 1

        print len(path)
        # display with rviz
        for i, point in enumerate(path):
            tmp = self.convert_point_rviz(point)
            tmp = (tmp[0] * self.resolution_, tmp[1] * self.resolution_)
            path[i] = tmp

        print "drawing points"
        print path
        draw_points(path, self.viz_pub_)
        self.waypoints_ = path

    def get_new_waypoint(self):
        """
        Retrieve the next way point. If there are no waypoints left, get new ones.
        :returns: (x, y)
        """

        if len(self.waypoints_) == 0:
            # If there are no remaining waypoints, get some new ones
            self.path_plan(self.coords_, self.target_)
        self.curr_waypoint_ = self.waypoints_.pop(0)

        return self.curr_waypoint_


if __name__ == '__main__':
    rospy.init_node('global_planner')
    GlobalPlanner()
    # ROS go brrrr
    rospy.spin()
