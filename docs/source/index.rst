.. ResearchTrack2-Assignment-1 documentation master file, created by
   sphinx-quickstart on Sat Apr 12 13:09:57 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to ResearchTrack2-Assignment-1's documentation!
=======================================================
## Overview  

This package implements a set of ROS nodes designed to facilitate robot navigation. It includes the following components:

1. **Bug Algorithm Node (bug_as.py)**: A ROS node that implements the Bug algorithm for robot navigation. This node allows the robot to navigate around obstacles while following a target position.

2. **Go-to-Point Service Node (go_to_point_service.py)**: A service node that allows the robot to move towards a specified target position by calculating the required orientation and distance. This node tracks and adjusts the robot's yaw and position until it reaches the target.

3. **Wall-Following Node (wall_follow_service.py)**: A node that enables the robot to follow a wall by adjusting its movement based on laser scan data. The robot uses the scan data to navigate around obstacles and maintain a safe distance from walls.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   bug_as
   go_to_point_service
   wall_follow_service
   modules

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
