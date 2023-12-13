# Home-Service-Robot
 Home Service Robot
 
This is a project in a course on Udacity.
In this project, learners will learn issues related to robotics
1. Design the robot's environment with the Building Editor in Gazebo.
2. Using Teleoperate the robot and manually test SLAM.
3. Use the ROS navigation stack and manually command the robot using the 2D Nav Goal arrow in Rviz to move to 2 different desired positions and orientations.
4. Write a pick_objects node that commands the robot to move to the desired pickup and drop-off zones.
5. Write an add_markers node that subscribes to the robot odometry and publishes pick-up and drop-off markers to Rviz.
6. Modify file pick_objects_node and add_markers_node to establish communication between them, to complete desired home service robot implementation

# In the Home Service Robot project, several key packages are utilized to enable localization, mapping, and navigation functionalities. Let's break down each aspect:

## Localization:
AMCL (Adaptive Monte Carlo Localization): AMCL is a probabilistic localization algorithm that uses a particle filter to estimate the robot's pose (position and orientation) within a known map. It enables the robot to determine its location concerning the environment.

## Mapping: Gmapping (Grid-based FastSLAM): 
Gmapping is a popular algorithm for constructing grid maps from laser range sensor data. It employs a variant of the FastSLAM algorithm, allowing the robot to build a map of its environment while simultaneously localizing itself within that map.

## Navigation: ROS Navigation Stack: 
The ROS Navigation Stack is a collection of packages and algorithms that facilitate robot autonomous navigation. It includes modules such as the global planner, local planner, and costmap that work together to plan a path from the robot's current location to a specified goal while avoiding obstacles.

## Move Base: 
Move Base is a high-level navigation system that integrates with the ROS Navigation Stack. It takes input from the global planner and local planner to guide the robot safely through its environment. It handles path planning, obstacle avoidance, and velocity control tasks.

## ROS Navigation Messages: 
These are standardized message types used for communication between different components of the navigation stack.

Combining these packages, the Home Service Robot project achieves a comprehensive set of capabilities. The robot can accurately localize itself, create and update maps of its surroundings, and autonomously navigate through the environment to reach specified goals while avoiding obstacles. These packages leverage the Robot Operating System (ROS) framework, providing a modular and extensible architecture for building complex robotic systems.

# Follow all steps below to build this project:
1. Git clone my project.
   ```git clone ```
2. Within your home directory, execute the following:
Note Execute the following commands in the workspace terminal to set it up for the project:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
sudo apt-get update
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/slam_gmapping
git clone https://github.com/turtlebot/turtlebot
git clone https://github.com/turtlebot/turtlebot_interactions
git clone https://github.com/turtlebot/turtlebot_simulator
cd ~/catkin_ws/
source devel/setup.bash
rosdep -i install gmapping
rosdep -i install turtlebot_teleop
rosdep -i install turtlebot_rviz_launchers
rosdep -i install turtlebot_gazebo
catkin_make
source devel/setup.bash
```
