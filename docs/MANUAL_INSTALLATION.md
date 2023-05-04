## Manual Installation
The system has been tested with 20.04 (ROS Noetic).

Follow the tutorials to
- [install ROS](http://wiki.ros.org/ROS/Installation) based on the Ubuntu version.
- [set up catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

### Tentabot depends on the following libraries/packages:

#### Install [catkin-simple](https://github.com/catkin/catkin_simple) package into the src folder.
#### Install [mav_comm](https://github.com/ethz-asl/mav_comm) package into the src folder.
#### Install [forest_gen](https://github.com/ethz-asl/forest_gen) package into the src folder.
#### Install [octomap_rviz_plugins](https://github.com/OctoMap/octomap_rviz_plugins) package into the src folder.
#### Install [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin.git) package into the src folder.
#### Install [stretch_ros](https://github.com/hello-robot/stretch_ros) package into the src folder.
#### Install [jackal](https://github.com/jackal/jackal.git) package into the src folder.
#### Install [husky](https://github.com/husky/husky.git) package into the src folder.

#### Install 'noetic-akmandor' branch of [rotors_simulator](https://github.com/RIVeR-Lab/rotors_simulator.git) package into the src folder.
#### Install 'noetic-akmandor' branch of [pedsim_ros](https://github.com/RIVeR-Lab/pedsim_ros.git) package into the src folder.
#### Install 'noetic-akmandor' branch of [turtlebot3](https://github.com/RIVeR-Lab/turtlebot3.git) package into the src folder.
#### Install 'noetic-akmandor' branch of [LMS1xx](https://github.com/RIVeR-Lab/LMS1xx.git) package into the src folder.
#### Install 'noetic-akmandor' branch of [geometry2](https://github.com/RIVeR-Lab/geometry2.git) package into the src folder.

#### Install [openai-ros](https://github.com/RIVeR-Lab/openai_ros) package into the src folder.
#### Install [tentabot](https://github.com/RIVeR-Lab/tentabot) package into the src folder.


```
cd ~/catkin_ws/src
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/forest_gen.git
git clone https://github.com/OctoMap/octomap_rviz_plugins.git
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
git clone https://github.com/hello-robot/stretch_ros
git clone https://github.com/jackal/jackal.git
git clone https://github.com/husky/husky.git
git clone https://github.com/RIVeR-Lab/rotors_simulator.git 		#'noetic-akmandor' branch*
git clone https://github.com/RIVeR-Lab/pedsim_ros.git 				#'noetic-akmandor' branch*
git clone https://github.com/RIVeR-Lab/turtlebot3.git 				#'noetic-akmandor' branch*
git clone https://github.com/RIVeR-Lab/LMS1xx.git 					#'noetic-akmandor' branch*
git clone https://github.com/RIVeR-Lab/geometry2.git 				#'noetic-akmandor' branch*
git clone https://github.com/RIVeR-Lab/openai_ros.git
git clone https://github.com/RIVeR-Lab/tentabot.git
```

##### *Switch to 'noetic-akmandor' branch in the package (turtlebot3,LMS1xx,geometry2):
```
git checkout noetic-akmandor
```

#### (Required for Tentabot-DRL) Install [stable-baselines3](https://stable-baselines3.readthedocs.io/en/master/guide/install.html#stable-release) Python package following their instructions.

#### Install ROS dependencies using rosdep tool:

Follow the instructions under 'Prerequisites' and install rosdep tool based on the ROS version: [Noetic rosdep](http://wiki.ros.org/noetic/Installation/Source).

ROS Noetic example of installing dependencies using rosdep tools:
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

#### Install Python dependencies
```
pip install stable-baselines3[extra]
pip install GitPython
pip install squaternion
```
