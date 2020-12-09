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
        self.target_ = None

        # (x, y, yaw) in meters
        self.odom_ = None

        # (x, y) in map space
        self.coords_ = None

        # Counter for number of times obstacle avoidance is triggered (>10 triggers replan)
        self.redirect_ = 0

        self.count_ = 0
        self.far_ = False

        self.three_point_ = 0

    def lidar_callback(self, msg):
        """
        Callback for the LIDAR subscriber.
        Reads in msg and tries to continue on to the currently selected
        waypoint.
        :param: msg: lidar msg
        :returns: None
        """
        self.redirect_ += 1
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
        # it's <45 degrees from the front it will start driving forward
        if angle > np.pi / 4 or angle < - np.pi / 4:
            command.linear.x = 0
            command.angular.z = angle * .9
        else:
            command.linear.x = 0.2
            command.angular.z = angle * .5

        # Stop the robot when position is reached
        if total_distance < 1.0:
            self.redirect_ = 0
            self.get_new_waypoint()
            self.wait_ = 0

        current_laser_theta = min_angle
        for i, scan in enumerate(distances):
            half_angle = 46
            # This if statement basically checks to see if we're close to a
            # wall, then checks if that wall is within a 50 degree angle in
            # front of us and we're moving towards it. If all conditions are
            # satisfied then we do a hard left or right depending on whether
            # the wall is closer to the left or right front
            if scan < 0.35:
                # Obstacle in front
                if 360 - half_angle < i < 360 and command.linear.x > 0:
                    print "***********DIVERT**********"
                    command.linear.x = -0.2
                    command.angular.z = 1.0
                    self.redirect_ += 1
                elif 0 <= i < 0 + half_angle and command.linear.x > 0:
                    print "***********DIVERT**********"
                    command.linear.x = -0.2
                    command.angular.z = -1.0
                    self.redirect_ += 1.0
                # Obstacle to the left
                elif half_angle < i < 180 - half_angle:
                    command.angular.z = -0.5
                # Obstacle to the right
                elif 180 + half_angle < i < 360 - half_angle:
                    command.angular.z = 0.5
                # Obstacle behind
                elif 180 - half_angle < 180 + half_angle:
                    command.linear.x = .2
                    self.redirect_ += 1
            current_laser_theta = current_laser_theta + angle_increment

        if self.redirect_ > 75:
            if self.three_point_ == 2:
                command.linear.x = 0.4
                command.angular.z = 0.0
                self.redirect_ = 0
                self.far_ = not self.far_
                self.waypoints_ = []
                self.curr_waypoint_ = None
                self.redirect_ = 0
                self.three_point_ = 0
            elif self.three_point_ == 1:
                command.linear.x = 0.0
                command.angular.z = .5
                self.three_point_ = 2
            else:
                command.linear.x = -0.6
                command.angular.z = 0.0
                self.three_point_ = 1
        
        """
        if self.count_ > 100:
            self.waypoints_ = []
            self.curr_waypoint_ = None
        """

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

        self.coords_ = self.rviz_to_np((position.x, position.y))


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
        

        # using 3 for the half radius cuz I don't want to calc that rn
        self.map_ = mapreading.live_threshold(data)
        # Trigger replan on new map
        # self.waypoints_ = []

    def convert_point_np(self, point):
        """
        Convert a point from (x, y) to (y, x) or the other way.
        :param: point: (x, y)
        :returns: (y, x)
        """
        return (point[1], point[0])

    def np_to_rviz(self, point):
        """
        Converts points from numpy array form to rviz coordinates with (0, 0) at the center of the map
        :param point: (r, c) coordinate
        :return: (x, y) coordinate shifted and flipped along the y axis
        """
        x_shift = np.size(self.map_, axis=1) / 2
        y_shift = np.size(self.map_, axis=0) / 2
        new_x = (point[1] - 1 - x_shift) * self.resolution_
        new_y = (point[0] - 1 - y_shift) * self.resolution_
        return (new_x, -new_y)

    def rviz_to_np(self, point):
        """
        Converts point from rviz frame to numpy frame (r, c), with (0, 0) at the top left of the map
        :param point: rviz coordinate (x, y)
        :return: np coordinate (r, c)
        """
        r_shift = np.size(self.map_, axis=0) / 2
        c_shift = np.size(self.map_, axis=1) / 2
        new_r = -point[1] / self.resolution_ + r_shift
        new_c = point[0] / self.resolution_ + c_shift
        return (int(np.ceil(new_r)), int(np.ceil(new_c)))

    def path_plan(self, start, target):
        """
        Plan a new path with A* from start to target
        :param: start: (x,y), target: (x,y)
        :returns: None
        """
        # I really don't get it but apparently these are reversed? At least it works when I switch them
        s = start
        t = target

        points, parents, size = astar.astar(self.map_, s, t)

        # Do stuff with the path, like get the waypoints
        path = []
        child = s
        i = 0
        while child != t:
            if i % 5 == 0 or i == 0:
                path.append(child)
            child = parents[child]
            i += 1

        # display with rviz
        for i, point in enumerate(path):
            tmp = self.np_to_rviz(point)
            path[i] = tmp

        draw_points(path, self.viz_pub_)
        self.waypoints_ = path

    def get_new_waypoint(self):
        """
        Retrieve the next way point. If there are no waypoints left, get new ones.
        :returns: (x, y)
        """

        if len(self.waypoints_) == 0:
            self.get_new_target()
            # If there are no remaining waypoints, get some new ones
            self.path_plan(self.coords_, self.target_)
        self.curr_waypoint_ = self.waypoints_.pop(0)

        return self.curr_waypoint_

    def get_new_target(self):
        """
        Gets list of boundary pixels and targets the one that's furthest from the current positions
        :return: Target coordinates (r, c)
        """

        """
        # Distance only target finding
        boundary = mapreading.get_boundary_pixels(self.map_)
        distance = np.abs(boundary[:, 0] - self.coords_[0]) + np.abs(boundary[:, 1] - self.coords_[1])
        # Get the first target that is more than 50 away
        distance = np.sort(distance)
        for i, dist in enumerate(distance):
            if dist > 50:
                self.target_ = (boundary[i, 0], boundary[i, 1])
        """

        # Distance and density target finding
        boundary = mapreading.get_weighted_boundary_pixels(self.map_)
        distance = np.abs(boundary[:, 0] - self.coords_[0]) + np.abs(boundary[:, 1] - self.coords_[1])
        distance = np.sort(distance)
        goal_distance = 2
        if self.far_:
            goal_distance = 10
        for i, dist in enumerate(distance):
            if boundary[i, 2] > 4 and dist > goal_distance:
                self.target_ = (boundary[i, 0], boundary[i, 1])

        return self.target_


if __name__ == '__main__':
    rospy.init_node('global_planner')
    GlobalPlanner()
    # ROS go brrrr
    rospy.spin()
