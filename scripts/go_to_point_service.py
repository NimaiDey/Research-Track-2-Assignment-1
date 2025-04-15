#!/usr/bin/env python3

"""
.. module:: go_to_point_service
   :platform: Unix
   :synopsis: ROS node for navigating the robot to a specified target point.

This node:
    - Subscribes to the `/odom` topic for odometry data
    - Publishes velocity commands to the `/cmd_vel` topic to control the robot's movement
    - Moves the robot towards a desired point by correcting yaw (orientation) and position

Subscribes to:
    - `/odom` (nav_msgs/Odometry): Odometry data for robot's position and orientation.

Publishes to:
    - `/cmd_vel` (geometry_msgs/Twist): Velocity commands to control robot movement.

Parameters:
    - `des_pos_x` (float): Desired x-coordinate of the target position.
    - `des_pos_y` (float): Desired y-coordinate of the target position.

Dependencies:
    - ``geometry_msgs``
    - ``nav_msgs``
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

# Try to import the 'Point' message from assignment_2_2022
# If import fails, define a simple class to hold x and y values
try:
    from assignment_2_2022.msg import Point
except ImportError:
    class Point:
        x = 0.0
        y = 0.0

pub_ = None
"""Publisher: ROS publisher for sending velocity commands to the robot."""

active_ = False
"""bool: Flag to track if the robot is actively moving or not."""

position_ = Point()
"""Point: Stores the current position of the robot in the environment."""

desired_position_ = Point()
"""Point: Stores the target position that the robot aims to reach."""

yaw_ = 0
"""float: Yaw angle (orientation) of the robot, calculated from odometry data."""

yaw_precision_ = math.pi / 90  # ±2 deg
"""float: The acceptable error in yaw angle for robot movement."""

dist_precision_ = 0.1
"""float: The acceptable distance error to consider robot has reached the target."""

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

def fix_yaw():
    """
    Corrects the robot's yaw to align with the desired target orientation.

    If the yaw error exceeds the allowed precision, adjusts angular velocity to correct the orientation.
    """
    global yaw_, pub_

    # Calculate the desired yaw based on the target position
    desired_yaw = math.atan2(desired_position_.y - position_.y,
                             desired_position_.x - position_.x)

    # Calculate the yaw error and normalize the angle
    err_yaw = normalize_angle(desired_yaw - yaw_)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3

    pub_.publish(twist_msg)

def go_straight_ahead():
    """
    Moves the robot straight towards the target position.

    If the robot is not within the desired distance of the target, moves it forward.
    """
    global pub_

    # Calculate the desired yaw based on the target position
    desired_yaw = math.atan2(desired_position_.y - position_.y,
                             desired_position_.x - position_.x)

    # Calculate the error in yaw and the distance to the target
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt((desired_position_.y - position_.y) ** 2 +
                        (desired_position_.x - position_.x) ** 2)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3  # Move forward at 0.3 m/s
        pub_.publish(twist_msg)

def normalize_angle(angle):
    """
    Normalizes the angle to be within the range [-pi, pi].

    Args:
        angle (float): The angle to normalize.

    Returns:
        float: The normalized angle.
    """
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / abs(angle)
    return angle

def done():
    """
    Stops the robot by publishing a zero velocity command.
    """
    twist_msg = Twist()
    pub_.publish(twist_msg)

def main():
    """
    Initializes and runs the go-to-point service for robot navigation.

    Sets up publishers and subscribers, and enters ROS spin loop.
    """
    global pub_, desired_position_

    # Initialize the ROS node
    rospy.init_node('go_to_point_service')

    # Retrieve desired position from parameters (default to 0,0 if not set)
    desired_position_.x = rospy.get_param('des_pos_x', 0.0)
    desired_position_.y = rospy.get_param('des_pos_y', 0.0)

    # Publisher to send velocity commands to robot
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Subscriber to receive odometry data
    rospy.Subscriber('/odom', Odometry, clbk_odom)

    # Enter the ROS spin loop to keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

