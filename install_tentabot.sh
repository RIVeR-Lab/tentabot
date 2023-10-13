#!/bin/bash

# Author: Neset Unver Akmandor (NUA)
#         Gary M. Lvov (GML)     
#         Hongyu Li (LHY) 
# E-Mail: akmandor.n@northeastern.edu
#         lvov.g@northeastern.edu
#         li.hongyu1@northeastern.edu
# TODO:
#
# NOTES:
# 1) Not required right now: Install joystick drivers into src folder
#   git clone https://github.com/ros-drivers/joystick_drivers.git
#   cd joystick_drivers
#   git checkout kinetic-devel
#   cd ..
# 2) Not required: install navrep 
#   git clone https://github.com/ethz-asl/navrep.git # need to run setup.py install

cd ..

# 3.1 Install outsourced ROS packages.
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/forest_gen.git
git clone https://github.com/OctoMap/octomap_rviz_plugins.git
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
git clone https://github.com/hello-robot/stretch_ros.git

# 3.2 Install edited ROS packages (checkout to the specified branch is required).
git clone https://github.com/RIVeR-Lab/rotors_simulator.git #checkout noetic-akmandor
cd rotors_simulator
git checkout noetic-akmandor
cd ..

git clone https://github.com/RIVeR-Lab/pedsim_ros.git #checkout noetic-akmandor
cd pedsim_ros
git checkout noetic-akmandor
cd ..

git clone git@github.com:RIVeR-Lab/turtlebot2.git
cd turtlebot2
git checkout noetic-river-v0
sh turtlebot_noetic.sh
cd ..

git clone https://github.com/RIVeR-Lab/turtlebot3.git #checkout noetic-akmandor
cd turtlebot3
git checkout noetic-akmandor
cd ..

git clone https://github.com/RIVeR-Lab/LMS1xx.git #checkout noetic-akmandor
cd LMS1xx
git checkout noetic-akmandor
cd ..

git clone https://github.com/RIVeR-Lab/geometry2.git #checkout noetic-akmandor
cd geometry2
git checkout noetic-akmandor
cd ..

# 3.3 Install other developed ROS packages.
git clone https://github.com/RIVeR-Lab/openai_ros.git
cd ..

# 3.4 Install Python dependencies (required for tentabot drl).
pip install stable-baselines3[extra]
pip install GitPython
pip install squaternion

# 3.5 Install other ROS dependencies using rosdep tool:
wait
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 3.6 Build the catkin workspace:
#catkin build
