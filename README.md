# Tentabot: A 3D Reactive Navigation Algorithm for Mobile Robots by Using Tentacle-Based Sampling

[![teaser](tentabot.gif)](https://youtu.be/5vZSEuWUXe4)

### 1. Related Papers

Hundelshausen, Felix V., et al. "Driving with tentacles-integral structures for sensing and motion." The DARPA Urban Challenge. Springer, Berlin, Heidelberg, 2009. 393-440.

### 2 Installation

The system has been tested with Ubuntu 16.04 (ROS Kinetic).

Follow the tutorials to [install ROS Kinetic](http://wiki.ros.org/ROS/Installation) and to [set up catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

#### Install required packages into the src folder:
```
cd ~/catkin_ws/src
```

Install [octomap_ros](https://wiki.ros.org/octomap_ros) for the the ROS Kinetic branch.
```
git clone https://github.com/OctoMap/octomap_ros.git
```

Install [octomap_msgs](http://wiki.ros.org/octomap_msgs) for the the ROS Kinetic branch.
```
git clone https://github.com/OctoMap/octomap_msgs.git
```

Follow the instructions and install [ewok](https://github.com/VladyslavUsenko/ewok/tree/ubuntu_16_04_kinetic) for the ROS Kinetic branch.

### 3. Simulation example

In separate terminal windows:

Start roscore:
```
roscore
```

Start visualization in Rviz:
```
roscd tentabot/rviz/
rviz -d simulation.rviz
```

Launch the system and Gazebo simulator:
```
roslaunch tentabot tentabot_go_cylinder.launch
```
OR
```
roslaunch tentabot tentabot_go_forest.launch
```

### 6. Credentials
Tentabot was developed at the [RIVeR Lab, Northeastern University](http://robot.neu.edu/).
