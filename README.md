# Tentabot: A 3D Reactive Navigation Algorithm for Mobile Robots by Using Tentacle-Based Sampling

[![teaser](tentabot_cylinders0.gif)](https://youtu.be/5vZSEuWUXe4) [![teaser](tentabot_forest1.gif)](https://youtu.be/5vZSEuWUXe4)

### 1. Related Papers

* N. Ü. Akmandor and T. Padir, "A 3D Reactive Navigation Algorithm for Mobile Robots by Using Tentacle-Based Sampling," 2020 Fourth IEEE International Conference on Robotic Computing (IRC), Taichung, Taiwan, 2020, pp. 9-16, [[DOI:10.1109/IRC.2020.00009]](https://doi.org/10.1109/IRC.2020.00009). [[arXiv:1703.01416]](https://arxiv.org/abs/1703.01416).
* Von Hundelshausen, Felix, et al. "**Driving with tentacles: Integral structures for sensing and motion**." Journal of Field Robotics 25.9 (2008): 640-673, [[DOI:10.1002/rob.20256]](https://doi.org/10.1002/rob.20256).
* Usenko, Vladyslav, et al. "**Real-time trajectory replanning for MAVs using uniform B-splines and a 3D circular buffer**." 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2017. [[DOI:10.1109/IROS.2017.8202160]](https://doi.org/10.1109/IROS.2017.8202160) [[arXiv:1703.01416]](https://arxiv.org/abs/1703.01416).

### 2. Videos

* [[IRC 2020 - supplementary video]](https://www.youtube.com/watch?v=5vZSEuWUXe4&t)
* [[IRC 2020 - presentation]](https://youtu.be/Y5FCiJPXmlo)

### 3 Installation

The system has been tested with both Ubuntu 16.04 (ROS Kinetic) and 18.04 (ROS Melodic).

Follow the tutorials to
- either [install ROS Kinetic](http://wiki.ros.org/ROS/Installation) or [install ROS Melodic](http://wiki.ros.org/ROS/Installation),
- [set up catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

#### 3.a Install required packages into the src folder:
```
cd ~/catkin_ws/src
```

Install [octomap_ros](https://wiki.ros.org/octomap_ros) based on the ROS branch (Kinetic or Melodic).
```
git clone https://github.com/OctoMap/octomap_ros.git
```

Install [octomap_msgs](http://wiki.ros.org/octomap_msgs) based on the ROS branch (Kinetic or Melodic).
```
git clone https://github.com/OctoMap/octomap_msgs.git
```

Follow the instructions and install [ewok](https://github.com/VladyslavUsenko/ewok/tree/master) based on the ROS branch (Kinetic or Melodic).

#### 3.b Install tentabot into the src folder and build the catkin workspace:
```
git clone https://github.com/RIVeR-Lab/tentabot.git
cd ~/catkin_ws
catkin build
```

### 4. Simulation example

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

### 5. Credentials
Tentabot was developed at the [RIVeR Lab, Northeastern University](http://robot.neu.edu/).
