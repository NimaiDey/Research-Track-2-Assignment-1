#!/usr/bin/env python3

"""
.. module:: bug_as
   :platform: Unix
   :synopsis: ROS node implementing the Bug algorithm for robot navigation.

This node:
    - Subscribes to the `/odom` topic for odometry data
    - Subscribes to the `/scan` topic for laser scan data
    - Publishes velocity commands to the `/cmd_vel` topic to control robot movement
    - Implements logic for navigating toward a desired position using the Bug algorithm

Subscribes to:
    - `/odom` (nav_msgs/Odometry): Odometry data for robot's position and orientation.
    - `/scan` (sensor_msgs/LaserScan): Laser scan data for obstacle detection.

Publishes to:
    - `/cmd_vel` (geometry_msgs/Twist): Velocity commands to control robot movement.

Dependencies:
    - ``geometry_msgs``
    - ``nav_msgs``
    - ``sensor_msgs``
"""

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Try to import the 'Point' message from assignment_2_2022
# If import fails, define a simple class to hold x and y values
try:
    from assignment_2_2022.msg import Point
except ImportError:
    class Point:
        x = 0.0
        y = 0.0

# === GLOBAL VARIABLES ===

active_ = False
"""bool: Flag to track if the robot is actively moving or not."""

pub_ = None
"""Publisher: ROS publisher for sending velocity commands to the robot."""

regions_ = None
"""dict: Stores the distance measurements from the laser scanner in different directions."""

position_ = Point()
"""Point: Stores the current position of the robot in the environment."""

yaw_ = 0
"""float: Yaw angle (orientation) of the robot, calculated from odometry data."""

yaw_error_allowed_ = math.pi / 90  # ±2 degrees
"""float: The acceptable error in yaw angle for robot movement."""

state_ = 0
"""int: Holds the current state of the robot. Used for state machine logic."""

desired_position_ = Point()
"""Point: Stores the target position that the robot aims to reach."""

def clbk_odom(msg):
    """
    Callback function to update the robot's position and orientation based on odometry data.

    Args:
        msg (Odometry): The Odometry message containing position and orientation data.
    """
    global position_, yaw_

    # Update the robot's position
    position_ = msg.pose.pose.position

    # Calculate yaw (orientation) from quaternion representation
    orientation_q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
    cosy_cosp = 1.0 - 2.0 * (orientation_q.y ** 2 + orientation_q.z ** 2)
    yaw_ = math.atan2(siny_cosp, cosy_cosp)

def clbk_laser(msg):
    """
    Callback function to update laser scan data for obstacle detection.

    Args:
        msg (LaserScan): The LaserScan message containing range data from the laser sensor.
    """
    global regions_

    # Assign laser scan readings to different regions (right, front, left, etc.)
    regions_ = {
        'right': min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front': min(min(msg.ranges[288:431]), 10),
        'fleft': min(min(msg.ranges[432:575]), 10),
        'left': min(min(msg.ranges[576:713]), 10),
    }

def change_state(state):
    """
    Changes the current state of the robot if the new state is different.

    Args:
        state (int): The new state to transition into.
    """
    global state_

    # Check if the state has changed and log the new state
    if state != state_:
        rospy.loginfo('State changed to [%s]', state)
        state_ = state

def normalize_angle(angle):
    """
    Normalizes the angle to be within the range [-pi, pi].

    Args:
        angle (float): The angle to normalize.

    Returns:
        float: The normalized angle.
    """
    if abs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / abs(angle)
    return angle

def main():
    """
    Initializes and runs the Bug algorithm for robot navigation.

    Sets up publishers and subscribers, and enters ROS spin loop.
    """
    global pub_, active_

    # Initialize the ROS node
    rospy.init_node('bug_as')

    # Publisher to send velocity commands to robot
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Subscriber to receive odometry data
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    # Subscriber to receive laser scan data
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    # Enter the ROS spin loop to keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

