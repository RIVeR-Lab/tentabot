FROM osrf/ros:noetic-desktop-full
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
SHELL ["/bin/bash", "-c"]

RUN apt update
RUN apt upgrade -y
RUN apt install python3-catkin-tools -y
RUN apt install git -y
RUN apt install python3-pip -y
RUN apt install libsuitesparse-dev -y
RUN apt install libnlopt-dev -y
RUN apt install python-is-python3 -y
RUN pip install --upgrade numpy

# Install dependencies
RUN mkdir -p /home/tent_deps
WORKDIR /home/tent_deps

# Install libccd
RUN git clone https://github.com/danfis/libccd.git
WORKDIR /home/tent_deps/libccd
RUN mkdir build
WORKDIR /home/tent_deps/libccd/build
RUN cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON ..
RUN make
RUN make install

# Install libeigen3
RUN apt install libeigen3-dev

# Install fcl
WORKDIR /home/tent_deps
RUN git clone https://github.com/flexible-collision-library/fcl.git
WORKDIR /home/tent_deps/fcl
RUN mkdir build
WORKDIR /home/tent_deps/fcl/build
RUN cmake ..
RUN make
RUN make install

# Install ROS Pkgs
RUN mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws/src
RUN git clone https://github.com/ethz-asl/rotors_simulator.git
RUN git clone https://github.com/RIVeR-Lab/turtlebot3.git -b noetic-akmandor  #'noetic-akmandor' branch*
RUN git clone https://github.com/RIVeR-Lab/LMS1xx.git -b noetic-akmandor #'noetic-akmandor' branch*
RUN git clone https://github.com/RIVeR-Lab/geometry2.git -b noetic-akmandor #'noetic-akmandor' branch*
RUN git clone https://github.com/catkin/catkin_simple.git
RUN git clone https://github.com/ethz-asl/forest_gen.git
RUN git clone https://github.com/ethz-asl/mav_comm.git
RUN git clone https://github.com/OctoMap/octomap_rviz_plugins.git
RUN git clone https://github.com/srl-freiburg/pedsim_ros.git
RUN git clone https://github.com/hello-robot/stretch_ros.git
RUN git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git

# Install Python dep
WORKDIR /home/catkin_ws
RUN source /opt/ros/noetic/setup.sh
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro noetic -y

# install DRL dep

RUN pip install torch==1.8.2+cu111 -f https://download.pytorch.org/whl/lts/1.8/torch_lts.html
RUN pip install stable-baselines3[extra]
RUN pip install GitPython
RUN pip install squaternion

RUN apt install screen -y

# fix bugs for Intel tigerlake arch
RUN apt install gcc-10 g++-10 -y
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 --slave /usr/bin/g++ g++ /usr/bin/g++-10 --slave /usr/bin/gcov gcov /usr/bin/gcov-10
RUN update-alternatives --config gcc

RUN apt install ros-noetic-perception -y
# build what we have
WORKDIR /home/catkin_ws
RUN catkin config --extend /opt/ros/noetic
RUN catkin build

# install tentabot
WORKDIR /home/catkin_ws/src
COPY tentabot-main.zip .
RUN unzip tentabot-main.zip
COPY openai_ros-main.zip .
RUN unzip openai_ros-main.zip
WORKDIR /home/catkin_ws
RUN catkin build; exit 0
RUN catkin build; exit 0
RUN catkin build

RUN apt install nano -y
RUN apt install net-tools -y
RUN apt install libqt5gui5 -y
RUN strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so.5
RUN apt install python3-tk -y