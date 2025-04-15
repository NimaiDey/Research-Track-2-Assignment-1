# ResearchTrack2-Assignment-1

## Project Overview

This project involves building a ROS (Robot Operating System) system that utilizes various modules to enable a robot to perform specific tasks such as following a wall, moving towards a target point, and managing a bug's behavior in a simulated environment. The system leverages ROS packages for communication, service handling, and message passing.

### Key Features
- **ROS-based system** for robotic control
- **Modules for wall following, bug movement, and navigation**
- **Uses Turtlesim for simulation** and visualization
- **Integrates ROS services and topics** for communication
- **Python and C++ integration** for efficient node handling

## Project Structure

The project is divided into the following main directories and files:

### 1. `scripts/`
Contains the Python scripts that define the logic of different ROS nodes.

#### **`bug_as.py`**:
This script implements the logic for the "bug" algorithm. It handles the movement of a robot in a simulated environment by detecting obstacles and adjusting its path accordingly. The algorithm simulates a behavior where the robot explores the environment while avoiding obstacles.

Key Functions:
- Continuously moves the robot forward while checking for obstacles.
- If an obstacle is detected, the robot uses the "bug" algorithm to avoid it and continue moving towards the goal.
- It also integrates ROS topics and services for communication between the robot and other systems.

#### **`go_to_point_service.py`**:
This script provides a ROS service that allows a robot to move to a specified point in a 2D space. The service listens for parameters that define the target position and moves the robot accordingly.

Key Functions:
- Subscribes to ROS parameters to get the desired target position.
- Moves the robot towards the target location.
- Integrates with ROS services for dynamic control and feedback.

#### **`wall_follow_service.py`**:
This script implements wall-following behavior for the robot. It allows the robot to follow the walls of an environment, using its sensors to maintain a set distance from the wall while navigating.

Key Functions:
- Constantly adjusts the robot's movement to keep a consistent distance from the wall.
- Uses ROS services and topics for control, ensuring that the robot follows the wall even if the path changes.
- It also ensures that the robot navigates correctly by adjusting its orientation and speed based on environmental feedback.

### 2. `msg/`
Contains message files used for communication between nodes.

### 3. `CMakeLists.txt` and `package.xml`
These files define the dependencies, compilation instructions, and the ROS package configuration.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/NimaiDey/ResearchTrack2-Assignment-1.git
#### 2.Install dependencies (ensure ROS is installed):
cd ~/catkin_ws
catkin_make
##### Source the workspace:

source devel/setup.bash
Run the ROS nodes as per the requirements.

## Usage
Running the ROS Nodes:
Start the ROS master:

## roscore
Run individual nodes:

## Go To Point Service:


rosrun assignment_2_2022-main go_to_point_service.py
## Wall Follow Service:


rosrun assignment_2_2022-main wall_follow_service.py
## Bug Algorithm:


rosrun assignment_2_2022-main bug_as.py
Use rosservice to call services or test the system's behavior.

#### Documentation
For detailed documentation about each module and the overall system architecture, visit the project documentation.
 ## home/nimailinux/assignment_2_2022-main/docs/build/html/index.html

Dependencies
ROS Noetic (or appropriate version)

geometry_msgs, turtlesim, and other common ROS packages

Python 2 or 3 compatible for ROS scripting

License
This project is licensed under the MIT License - see the LICENSE file for details.
