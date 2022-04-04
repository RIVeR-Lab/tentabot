cmake_minimum_required(VERSION 3.0.2)
project(tentabot)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
add_definitions(-std=c++14 -g -O3 -ftree-vectorize -march=native)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy 
  std_msgs
  message_generation
  geometry_msgs 
  pcl_msgs 
  pcl_conversions 
  pcl_ros
  octomap_ros
  octomap_msgs
  sensor_msgs
  mav_msgs
  eigen_conversions
  tf
  tf_conversions
  cv_bridge
  forest_gen
  rotors_control
  laser_geometry
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
#find_package(octomap REQUIRED)
find_package(CCD REQUIRED)

find_library(EIGEN_CDDLIB_LIB eigen-cddlib)

if(EXISTS "${CMAKE_SOURCE_DIR}/cmake/FindFCL.cmake")
  find_package(FCL REQUIRED)
else()
  set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/")
  find_package(FCL REQUIRED)
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   rl_step.srv
   update_goal.srv
   reset_map_utility.srv
   get_laser_image.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   nav_msgs
   mav_msgs
#   tentabot
#   rl_step
#   update_goal
#   reset_map_utility
#   get_laser_image
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#   LIBRARIES tentabot mat
   CATKIN_DEPENDS roscpp rospy message_runtime std_msgs geometry_msgs octomap_ros octomap_msgs sensor_msgs pcl_ros
   DEPENDS FCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR}
#  mav_comm
#  rotors_simulator
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tentabot.cpp
# )

add_library(trajectory_sampling_utility src/trajectory_sampling_utility.cpp)
add_library(goal_utility src/goal_utility.cpp)
add_library(map_utility src/map_utility.cpp)
add_library(tentabot src/tentabot.cpp)
add_library(laser_to_image src/laser_to_image.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(map_utility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tentabot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(laser_to_image ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/tentabot_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(publish_robot_pose_command src/publish_robot_pose_command.cpp)
target_link_libraries(publish_robot_pose_command ${catkin_LIBRARIES})

add_executable(map_utility_server src/map_utility_server.cpp)
#add_dependencies(map_utility_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_utility_server ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${CHOLMOD_LIBRARY} nlopt ${TF_LIBRARIES} map_utility ${FCL_LIBRARIES})

add_executable(laser_to_image_server src/laser_to_image_server.cpp)
#add_dependencies(laser_to_image_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_to_image_server ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${CHOLMOD_LIBRARY} ${TF_LIBRARIES} laser_to_image)

add_executable(tentabot_server src/tentabot_server.cpp)
#add_dependencies(tentabot_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(tentabot_server ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${CHOLMOD_LIBRARY} nlopt ${TF_LIBRARIES} trajectory_sampling_utility goal_utility tentabot)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tentabot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
