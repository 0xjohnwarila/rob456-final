#!/usr/bin/env python

import rospy
import numpy as np

# Sensor message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# the velocity command message
from geometry_msgs.msg import Twist

GOAL = (0, 1.5)
ODOM = None

def lidar_callback(scan_msg):
    global GOAL, ODOM
    # Let's make a new twist message
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
    # find current laser angle, max scan length, distance array for all scans, and number of laser scans
    maxAngle = scan_msg.angle_max
    minAngle = scan_msg.angle_min
    angleIncrement = scan_msg.angle_increment

    maxScanLength = scan_msg.range_max
    distances = scan_msg.ranges
    numScans = len(distances)

    print ODOM
    # Problem 1: move the robot toward the goal
    # Finding coordinate differences between goal and odometer readings
    x_distance = GOAL[0] - ODOM[0]
    y_distance = GOAL[1] - ODOM[1]
    total_distance = np.sqrt(x_distance ** 2 + y_distance ** 2)
    # I subtracted the odometer angle from the arctan angle so I can use that to only turn as much as I need to
    # (PS: Why oh why would the numpy people write a function that flipped x and y like that? It was the root of all my
    # problems for so long)
    angle = np.arctan2(y_distance, x_distance) - ODOM[2]
    # If the target point is behind the robot then it will only turn, once it's <90 degrees from the front it will
    # start driving forward
    if angle > np.pi / 2 or angle < - np.pi / 2:
        command.linear.x = 0
        command.angular.z = angle * .3
    else:
        command.linear.x = .1
        command.angular.z = angle * .3
    # Condition for stopping the robot
    if total_distance < 0.1:
        command.linear.x = 0
        command.linear.z = 0
    # End problem 1

    currentLaserTheta = minAngle
    # for each laser scan
    for i, scan in enumerate(distances):
        # for each laser scan, the angle is currentLaserTheta, the index is i, and the distance is scan
        # Problem 2: avoid obstacles based on laser scan readings
        half_angle = 25
        # This if statement basically checks to see if we're close to a wall, then checks if that wall is within a 50
        # degree angle in front of us and we're moving towards it. If all conditions are satisfied then we do a hard
        # left or right depending on whether the wall is closer to the left or right front
        if scan < .5:
            if 360 - half_angle < i < 360 and command.linear.x > 0:
                command.linear.x = 0
                command.angular.z = 1
            elif 0 <= i < 0 + half_angle and command.linear.x > 0:
                command.linear.x = 0
                command.angular.z = -1
        # End problem 2
        # After this loop is done, we increment the currentLaserTheta
        currentLaserTheta = currentLaserTheta + angleIncrement

    pub.publish(command)


def odom_callback(msg):
    """
    Subscribes to the odom message, unpacks and transforms the relevent information, and places it in the global variable ODOM
    ODOM is structured as follows:
    ODOM = (x, y, yaw)

    :param: msg: Odometry message
    :returns: None
    """
    global ODOM
    position = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    (r, p, yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    ODOM = (position.x, position.y, yaw)


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('lab2', log_level=rospy.DEBUG)

    # subscribe to sensor messages
    lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

    # publish twist message
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Turn control over to ROS
    rospy.spin()
