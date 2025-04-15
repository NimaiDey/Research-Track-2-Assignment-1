#!/usr/bin/env python3

"""
.. module:: wall_follow_service
   :platform: Unix
   :synopsis: ROS node for wall-following behavior using laser scan data.

This node:
    - Subscribes to the `/scan` topic to receive laser scan data
    - Publishes velocity commands to the `/cmd_vel` topic to control robot movement
    - Uses laser scan data to follow walls by adjusting the robot's linear and angular velocities

Subscribes to:
    - `/scan` (sensor_msgs/LaserScan): Laser scan data for obstacle detection and wall-following behavior.

Publishes to:
    - `/cmd_vel` (geometry_msgs/Twist): Velocity commands to control robot movement.

Dependencies:
    - ``geometry_msgs``
    - ``sensor_msgs``
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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

regions_ = None
"""dict: Stores laser scan data for different regions around the robot (right, front, left, etc.)."""

def clbk_laser(msg):
    """
    Callback function that processes laser scan data and determines the robot's action based on obstacles.

    Args:
        msg (LaserScan): The LaserScan message containing range data from the laser sensor.
    """
    global regions_

    # Process laser scan data to extract the minimum distance in each region (right, front, left, etc.)
    regions_ = {
        'right': min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front': min(min(msg.ranges[288:431]), 10),
        'fleft': min(min(msg.ranges[432:575]), 10),
        'left': min(min(msg.ranges[576:713]), 10),
    }

    # Call take_action() to make the robot take appropriate action based on the laser scan data
    take_action()

def take_action():
    """
    Determines the robot's behavior based on laser scan data and publishes appropriate velocity commands.

    The robot adjusts its linear and angular velocities depending on detected obstacles.
    """
    global regions_, pub_

    # Create a Twist message to set linear and angular velocity commands
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    # Define the minimum safe distance for obstacle detection
    d = 1.0

    # Case 1: No obstacles in front, left, or right
    if regions_['front'] > d and regions_['fleft'] > d and regions_['fright'] > d:
        state_description = 'case 1 - nothing'
        linear_x = 0.5  # Move forward
        angular_z = 0   # No turning

    # Case 2: Obstacle detected in front, but clear left and right
    elif regions_['front'] < d and regions_['fleft'] > d and regions_['fright'] > d:
        state_description = 'case 2 - front'
        linear_x = 0   # Stop moving forward
        angular_z = 0.3  # Turn clockwise to avoid the obstacle

    # Case 3: Obstacle detected in front and right, but clear left
    elif regions_['front'] < d and regions_['fleft'] > d and regions_['fright'] < d:
        state_description = 'case 3 - fright + front'
        linear_x = 0   # Stop moving forward
        angular_z = 0.3  # Turn clockwise to avoid the obstacle

    # Case 4: Unknown situation (should not happen if other cases are properly handled)
    else:
        state_description = 'unknown case'

    rospy.loginfo(state_description)  # Log the current action
    msg.linear.x = linear_x  # Set linear velocity
    msg.angular.z = angular_z  # Set angular velocity
    pub_.publish(msg)  # Publish velocity commands to the robot

def main():
    """
    Initializes and runs the wall-following service for the robot.

    Sets up publishers and subscribers, and enters the ROS spin loop.
    """
    global pub_

    # Initialize the ROS node
    rospy.init_node('wall_follow_service')

    # Publisher to send velocity commands to robot
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Subscriber to receive laser scan data
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    # Enter the ROS spin loop to keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

