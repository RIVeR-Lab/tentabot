#!/bin/bash

# Author: Neset Unver Akmandor (NUA)
#         Gary M. Lvov (GML)     
#         Hongyu Li (LHY) 
# E-Mail: akmandor.n@northeastern.edu
#         lvov.g@nor4theastern.edu
#         li.hongyu1@northeastern.edu

# GML TODO: verify new auto install

echo -e "What is the absolute file path to your user from root? (for example: /home/YOUR_USERNAME_HERE/)"
read abs_path
echo -e "What is the absolute path to your catkin workspace? (for example:  home/YOUR_USERNAME_HERE/catkin_ws)"
read catkin_path

echo -e "What build command do you use? (catkin_make or catkin build)"
read catkin_command

echo -e "Do you already have FCL (the Flexible Collision Library)"
echo -e "and it's dependencies installed? (y/n)"
read fcl

cd abs_path

if [[ "$fcl" == "n" ]]; then
    # 3.1 Install flexible-collision-library/fcl following their instructions using CMake.
    #------ Install libcdd sub-dep
    cd abs_path
    mkdir tentabot_deps
    cd tentabot_deps

    git clone https://github.com/danfis/libccd.git
    cd libccd
    mkdir build 
    cd build
    cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON ..
    make 
    make install
    cd ..
    cd ..

    #------ Install eigen sub-dep
    apt-get install libeigen3-dev

    #------ Install fcl
    git clone https://github.com/flexible-collision-library/fcl.git
    cd fcl
    mkdir build 
    cd build
    cmake ..
    make install
    cd ..
    cd ..
fi

eval cd "$catkin_path"/src

# 3.3 Install rotors_simulator package into the src folder.
git clone https://github.com/ethz-asl/rotors_simulator.git

# 3.4 Install 'noetic-akmandor' branch of turtlebot3 package into the src folder.
git clone https://github.com/RIVeR-Lab/turtlebot3.git  #'noetic-akmandor' branch*
cd turtlebot3
git checkout noetic-akmandor
cd ..

# 3.5 Install 'noetic-akmandor' branch of LMS1xx package into the src folder.
git clone https://github.com/RIVeR-Lab/LMS1xx.git  #'noetic-akmandor' branch*
cd LMS1xx
git checkout noetic-akmandor
cd ..

# 3.6 Install 'noetic-akmandor' branch of geometry2 package into the src folder.
git clone https://github.com/RIVeR-Lab/geometry2.git  #'noetic-akmandor' branch*
cd geometry2
git checkout noetic-akmandor
cd ..

# 3.7 Install catkin-simple package into the src folder.
git clone https://github.com/catkin/catkin_simple.git

# 3.8 Install forest_gen package into the src folder.
git clone https://github.com/ethz-asl/forest_gen.git

# 3.9 Install mav_comm package into the src folder.
git clone https://github.com/ethz-asl/mav_comm.git

# 3.10 Install octomap_rviz_plugins package into the src folder.
git clone https://github.com/OctoMap/octomap_rviz_plugins.git

# 3.11 Install pedsim_ros package into the src folder.
git clone https://github.com/srl-freiburg/pedsim_ros.git

# 3.12 Install openai-ros package into the src folder.
git clone https://github.com/RIVeR-Lab/openai_ros.git

# 3.13 Install tentabot package into the src folder.
# git clone https://github.com/RIVeR-Lab/tentabot.git

# Install stretch_ros package into src folder
git clone https://github.com/hello-robot/stretch_ros.git

# Install realsense package into src folder
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git

# Install jackal package into src folder
git clone https://github.com/jackal/jackal.git


# Install husky package into src folder
git clone https://github.com/husky/husky.git

# Install joystick drivers into ssrc folder
git clone https://github.com/ros-drivers/joystick_drivers.git
cd joystick_drivers
git checkout kinetic-devel
cd ..

# install navrep 
git clone https://github.com/ethz-asl/navrep.git # need to run setup.py install


# 3.15 Install ROS dependencies
apt-get update
apt-get install libsuitesparse-dev
apt-get install libnlopt-dev
apt-get install ros-noetic-octomap-msgs
apt-get install ros-noetic-octomap-ros
apt-get install ros-noetic-turtlebot3-msgs
apt-get upgrade python3-catkin-pkg
apt-get install libgflags-dev
apt-get install libgoogle-glog-dev
apt-get install protobuf-compiler libprotobuf-dev

# 3.16 Install other ROS dependencies using rosdep tool:
cd ..
wait
source /opt/ros/noetic/setup.bash
rosdep install -i --from-path src --rosdistro noetic -y

# 3.17 Install Python dependencies
pip install stable-baselines3[extra] #only neeeded for tentabot drl
pip install GitPython
pip install squaternion

# 3.18 Build the catkin workspace:
$catkin_command
