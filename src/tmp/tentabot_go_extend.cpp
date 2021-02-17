// LAST UPDATE: 2020.02.09
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION:
//

#include "tentabot_go_extend.h"

//#include <ewok/polynomial_3d_optimization.h>
//#include <ewok/uniform_bspline_3d_optimization.h>

int main(int argc, char** argv)
{
    // INITIALIZE ROS
    ros::init(argc, argv, "tentabot_go_extend");
    
    // INITIALIZE THE MAIN ROS NODE HANDLE
    ros::NodeHandle nh;

    // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
    ros::NodeHandle pnh("~");

    
    // INITIALIZE TRANSFORM LISTENER
    tf::TransformListener* listener = new tf::TransformListener;

    //// SET THE PARAMETERS
    // INITIALIZE AND SET WORLD FRAME NAME
    string world_frame_name;
    pnh.param<string>("world_frame_name", world_frame_name, "world");

    // INITIALIZE AND SET GOAL PARAMETERS
    double start_x, start_y, start_z, start_yaw;
    pnh.param("start_x", start_x, 0.0);
    pnh.param("start_y", start_y, 0.0);
    pnh.param("start_z", start_z, 0.0);
    pnh.param("start_yaw", start_yaw, 0.0);
    tf::Quaternion start_orientation;
    start_orientation.setRPY(0.0, 0.0, start_yaw);
    
    double goal1_x, goal1_y, goal1_z, goal1_yaw;
    pnh.param("goal1_x", goal1_x, 0.0);
    pnh.param("goal1_y", goal1_y, 0.0);
    pnh.param("goal1_z", goal1_z, 0.0);+
    pnh.param("goal1_yaw", goal1_yaw, 0.0);

    double goal2_x, goal2_y, goal2_z, goal2_yaw;
    pnh.param("goal2_x", goal2_x, 0.0);
    pnh.param("goal2_y", goal2_y, 0.0);
    pnh.param("goal2_z", goal2_z, 0.0);
    pnh.param("goal2_yaw", goal2_yaw, 0.0);

    GoalUtility gu(nh, world_frame_name, "twoGoal");
    gu.addGoalPoint(goal1_x, goal1_y, goal1_z);
    gu.addGoalPoint(goal2_x, goal2_y, goal2_z);

    // INITIALIZE AND SET MAP PARAMETERS
    double resolution;
    pnh.param("resolution", resolution, 0.15);
    string map_name;
    pnh.param<string>("world_name", map_name, "globalMap");
    MapUtility mu(nh, gu, resolution, world_frame_name, map_name);

    // INITIALIZE AND SET PROCESS PARAMETERS
    ProcessParams pp;
    pnh.param<bool>("visu_flag", pp.visu_flag, true);
    pnh.param<bool>("online_tuning_flag", pp.online_tuning_flag, false);
    pnh.param<double>("time_limit", pp.time_limit, 90);
    pnh.param<double>("dt", pp.nav_dt, 0.1);
    pnh.param<bool>("navexit_flag", pp.navexit_flag, false);
    pnh.param<double>("goal_close_threshold", pp.goal_close_threshold, 0.5);

    // INITIALIZE AND SET ROBOT PARAMETERS
    RobotParams rp;
    pnh.param<double>("robot_width", rp.width, 0.5);
    pnh.param<double>("robot_length", rp.length, 0.5);
    pnh.param<double>("robot_height", rp.height, 0.4);
    pnh.param<double>("dummy_max_lat_velo", rp.dummy_max_lat_velo, 5);
    pnh.param<double>("dummy_max_lat_acc", rp.dummy_max_lat_acc, 5);
    pnh.param<double>("dummy_max_yaw_velo", rp.dummy_max_yaw_velo, 2);
    pnh.param<double>("dummy_max_yaw_acc", rp.dummy_max_yaw_acc, 5);
    pnh.param<double>("dummy_max_pitch_velo", rp.dummy_max_pitch_velo, 1);
    pnh.param<double>("dummy_max_roll_velo", rp.dummy_max_roll_velo, 1);
    pnh.param<double>("init_robot_pos_x", rp.init_robot_pose.position.x, start_x);
    pnh.param<double>("init_robot_pos_y", rp.init_robot_pose.position.y, start_y);
    pnh.param<double>("init_robot_pos_z", rp.init_robot_pose.position.z, start_z);
    pnh.param<double>("init_robot_ori_x", rp.init_robot_pose.orientation.x, start_orientation.x());
    pnh.param<double>("init_robot_ori_y", rp.init_robot_pose.orientation.y, start_orientation.y());
    pnh.param<double>("init_robot_ori_z", rp.init_robot_pose.orientation.z, start_orientation.z());
    pnh.param<double>("init_robot_ori_w", rp.init_robot_pose.orientation.w, start_orientation.w());
    pnh.param<string>("robot_frame_name", rp.robot_frame_name, "firefly/base_link");
    pnh.param<string>("mav_name", rp.robot_name, "tentabot");
    pnh.param<double>("nav_sensor_freq", rp.nav_sensor.freq, 30);
    pnh.param<double>("nav_sensor_resolution", rp.nav_sensor.resolution, resolution);
    pnh.param<double>("nav_sensor_range_x_min", rp.nav_sensor.range_x[0], 0);
    pnh.param<double>("nav_sensor_range_x_max", rp.nav_sensor.range_x[1], 10);
    pnh.param<double>("nav_sensor_range_y_min", rp.nav_sensor.range_y[0], 0);
    pnh.param<double>("nav_sensor_range_y_max", rp.nav_sensor.range_y[1], 10);
    pnh.param<double>("nav_sensor_range_z_min", rp.nav_sensor.range_z[0], 0);
    pnh.param<double>("nav_sensor_range_z_max", rp.nav_sensor.range_z[1], 10);
    pnh.param<double>("nav_sensor_pos_x", rp.nav_sensor.pose_wrt_robot.position.x, 0.0);
    pnh.param<double>("nav_sensor_pos_y", rp.nav_sensor.pose_wrt_robot.position.y, 0.0);
    pnh.param<double>("nav_sensor_pos_z", rp.nav_sensor.pose_wrt_robot.position.z, 0.1);
    pnh.param<double>("nav_sensor_ori_x", rp.nav_sensor.pose_wrt_robot.orientation.x, 0.0);
    pnh.param<double>("nav_sensor_ori_y", rp.nav_sensor.pose_wrt_robot.orientation.y, 0.0);
    pnh.param<double>("nav_sensor_ori_z", rp.nav_sensor.pose_wrt_robot.orientation.z, 0.0);
    pnh.param<double>("nav_sensor_ori_w", rp.nav_sensor.pose_wrt_robot.orientation.w, 1);
    pnh.param<string>("nav_sensor_frame_name", rp.nav_sensor.frame_name, "firefly/camera_depth_optical_frame");

    // INITIALIZE AND SET OFFLINE TUNING PARAMETERS FOR GRID AND TENTACLES
    OffTuningParams offtp;
    pnh.param<int>("tyaw_cnt", offtp.tyaw_cnt, 31);
    pnh.param<int>("tpitch_cnt", offtp.tpitch_cnt, 21);
    pnh.param<int>("troll_cnt", offtp.troll_cnt, 0);
    pnh.param<int>("tsamp_cnt", offtp.tsamp_cnt, 30);
    pnh.param<int>("tlat_velo_cnt", offtp.tlat_velo_cnt, 1);
    pnh.param<int>("tyaw_velo_cnt", offtp.tyaw_velo_cnt, 1);
    pnh.param<int>("tpitch_velo_cnt", offtp.tpitch_velo_cnt, 1);
    pnh.param<int>("troll_velo_cnt", offtp.troll_velo_cnt, 1);
    pnh.param<double>("tlen", offtp.tlen, 10);
    pnh.param<double>("tyaw", offtp.tyaw, 0.3*PI);
    pnh.param<double>("tpitch", offtp.tpitch, 0.2*PI);
    pnh.param<double>("troll", offtp.troll, 0);
    pnh.param<string>("tentacle_type", offtp.tentacle_type, "linear");
    pnh.param<string>("tyaw_samp_type", offtp.tyaw_samp_type, "lin");
    pnh.param<string>("tpitch_samp_type", offtp.tpitch_samp_type, "lin");
    pnh.param<string>("troll_samp_type", offtp.troll_samp_type, "lin");
    pnh.param<double>("pdist", offtp.pdist, 0.4);
    pnh.param<double>("sdist", offtp.sdist, 1);
    pnh.param<double>("sweight_max", offtp.sweight_max, 1);
    pnh.param<double>("sweight_scale", offtp.sweight_scale, 10);
    pnh.param<double>("egrid_vdim", offtp.egrid_vdim, 0.1);
    pnh.param<int>("egrid_vnumx", offtp.egrid_vnumx, 200);
    pnh.param<int>("egrid_vnumy", offtp.egrid_vnumy, 200);
    pnh.param<int>("egrid_vnumz", offtp.egrid_vnumz, 200);

    // INITIALIZE AND SET ONLINE TUNING PARAMETERS FOR GRID AND TENTACLES
    OnTuningParams ontp;
    pnh.param<double>("crash_dist", ontp.crash_dist, 8);
    pnh.param<double>("clear_scale", ontp.clear_scale, 50);
    pnh.param<double>("flat_scale", ontp.flat_scale, 100);
    pnh.param<double>("close_scale", ontp.close_scale, 80);
    pnh.param<double>("smooth_scale", ontp.smooth_scale, 0.5);
    ontp.tbin_window = 1;
    ontp.tbin_obs_cnt_threshold = 1;

    
    // START THE GAZEBO
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);

    unsigned int i = 0;

    while (i <= 10 && !unpaused) 
    {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");           // Trying to unpause Gazebo for 10 seconds.
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

    ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 10);           // NUA TODO: Why does this only work when it is before the sleep?
    //ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::Point>("command/point", 10);

    ros::Duration(5.0).sleep();                                                         // Wait for 5 seconds to let the Gazebo GUI show up.

    geometry_msgs::PoseStamped p;
    p.header.seq++;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = world_frame_name;

    p.pose.position.x = start_x;
    p.pose.position.y = start_y;
    p.pose.position.z = start_z;

    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;

    geometry_msgs::Point ppp;
    ppp.x = start_x;
    ppp.y = start_y;
    ppp.z = start_z;

    trajectory_pub.publish(p);

    // INITIALIZE TENTABOT OBJECT
    Tentabot tbot(nh, listener, pp, rp, offtp, ontp, mu);

    // SUBSCRIBE TO THE ODOMETRY DATA
    ros::Subscriber sub = nh.subscribe("/firefly/odometry_sensor1/pose", 10, &Tentabot::odometryCallback, &tbot);
    
    // SUBSCRIBE TO THE DEPTH SENSOR DATA
    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
    depth_image_sub_.subscribe(nh, "camera/depth/image_raw", 10);
    tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, world_frame_name, 10);
    tf_filter_.registerCallback(&Tentabot::depthImageCallback, &tbot);
    
    ros::Duration(5.0).sleep();

    // NAV LOOP
    ros::Timer timer = nh.createTimer(ros::Duration(pp.nav_dt), &Tentabot::sendCommandCallback, &tbot);

    ros::spin();
}