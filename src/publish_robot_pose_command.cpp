// LAST UPDATE: 2022.03.18
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

// --EXTERNAL LIBRARIES--
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
#include <thread>

using namespace std;

int main(int argc, char** argv)
{
    // INITIALIZE ROS
    ros::init(argc, argv, "publish_robot_pose_command");
    
    // INITIALIZE THE MAIN ROS NODE HANDLE
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    string frame_name, robot_pose_control_msg;
    double robot_pose_x, robot_pose_y, robot_pose_z, robot_pose_yaw;

    pnh.param<string>("frame_name", frame_name, "");
    pnh.param<string>("robot_pose_control_msg", robot_pose_control_msg, "");
    pnh.param("robot_pose_x", robot_pose_x, 0.0);
    pnh.param("robot_pose_y", robot_pose_y, 0.0);
    pnh.param("robot_pose_z", robot_pose_z, 0.0);
    pnh.param("robot_pose_yaw", robot_pose_yaw, 0.0);

    tf::Quaternion robot_orientation;
    robot_orientation.setRPY(0.0, 0.0, robot_pose_yaw);


    // START THE GAZEBO
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);

    unsigned int i = 0;

    while (i <= 10 && !unpaused) 
    {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
    }

    if (!unpaused) 
    {
        ROS_FATAL("Could not wake up Gazebo.");
        return -1;
    }
    else 
    {
        ROS_INFO("Unpaused the Gazebo simulation.");
    }

    ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>(robot_pose_control_msg, 100);

    ros::Duration(1.0).sleep();

    geometry_msgs::PoseStamped p;
    
    p.pose.position.x = robot_pose_x;
    p.pose.position.y = robot_pose_y;
    p.pose.position.z = robot_pose_z;
    
    p.pose.orientation.x = robot_orientation.x();
    p.pose.orientation.y = robot_orientation.y();
    p.pose.orientation.z = robot_orientation.z();
    p.pose.orientation.w = robot_orientation.w();
    
    p.header.seq++;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = frame_name;

    trajectory_pub.publish(p);

    cout << "publish_robot_pose_command::main -> Published!" << endl;

    /*
    cout << "publish_robot_pose_command::main -> frame_name: " << frame_name << endl;
    cout << "publish_robot_pose_command::main -> robot_pose_control_msg: " << robot_pose_control_msg << endl;
    cout << "publish_robot_pose_command::main -> robot_pose_x: " << robot_pose_x << endl;
    cout << "publish_robot_pose_command::main -> robot_pose_y: " << robot_pose_y << endl;
    cout << "publish_robot_pose_command::main -> robot_pose_z: " << robot_pose_z << endl;
    cout << "publish_robot_pose_command::main -> robot_pose_yaw: " << robot_pose_yaw << endl;
    */
    
    ros::spin();
}