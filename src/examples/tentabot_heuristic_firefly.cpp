// LAST UPDATE: 2021.10.07
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

// --CUSTOM LIBRARIES--
#include "trajectory_sampling_utility.h"
#include "tentabot.h"

int main(int argc, char** argv)
{
    // INITIALIZE ROS
    ros::init(argc, argv, "tentabot_go");
    
    // INITIALIZE THE MAIN ROS NODE HANDLE
    ros::NodeHandle nh;

    // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
    ros::NodeHandle pnh("~");

    // INITIALIZE TRANSFORM LISTENER
    tf::TransformListener* listener = new tf::TransformListener;

    //// SET THE PARAMETERS

    // INITIALIZE AND SET GOAL PARAMETERS
    string world_frame_name;
    pnh.param<string>("world_frame_name", world_frame_name, "");
    GoalUtility gu(nh, world_frame_name, "twoGoal");
    double goal_x, goal_y, goal_z, goal_yaw;
    
    pnh.param("goal1_x", goal_x, 0.0);
    pnh.param("goal1_y", goal_y, 0.0);
    pnh.param("goal1_z", goal_z, 0.0);
    pnh.param("goal1_yaw", goal_yaw, 0.0);
    gu.addGoalPoint(goal_x, goal_y, goal_z);

    pnh.param("goal2_x", goal_x, 0.0);
    pnh.param("goal2_y", goal_y, 0.0);
    pnh.param("goal2_z", goal_z, 0.0);
    pnh.param("goal2_yaw", goal_yaw, 0.0);
    gu.addGoalPoint(goal_x, goal_y, goal_z);

    // INITIALIZE AND SET ROBOT PARAMETERS
    Tentabot::RobotParams rp;
    pnh.param<string>("world_name", rp.world_name, "");
    pnh.param<string>("robot_name", rp.robot_name, "");
    pnh.param<string>("robot_frame_name", rp.robot_frame_name, "");
    pnh.param<double>("robot_bbx_x_max", rp.robot_bbx_x_max, 0.0);
    pnh.param<double>("robot_bbx_x_min", rp.robot_bbx_x_min, 0.0);
    pnh.param<double>("robot_bbx_y_max", rp.robot_bbx_y_max, 0.0);
    pnh.param<double>("robot_bbx_y_min", rp.robot_bbx_y_min, 0.0);
    pnh.param<double>("robot_bbx_z_max", rp.robot_bbx_z_max, 0.0);
    pnh.param<double>("robot_bbx_z_min", rp.robot_bbx_z_min, 0.0);
    pnh.param<double>("robot_max_lat_velo", rp.dummy_max_lat_velo, 0.0);
    pnh.param<double>("robot_max_lat_acc", rp.dummy_max_lat_acc, 0.0);
    pnh.param<double>("robot_max_yaw_velo", rp.dummy_max_yaw_velo, 0.0);
    pnh.param<double>("robot_max_yaw_acc", rp.dummy_max_yaw_acc, 0.0);
    pnh.param<double>("robot_init_pos_x", rp.init_robot_pose.position.x, 0.0);
    pnh.param<double>("robot_init_pos_y", rp.init_robot_pose.position.y, 0.0);
    pnh.param<double>("robot_init_pos_z", rp.init_robot_pose.position.z, 0.0);
    double init_robot_yaw;
    pnh.param<double>("robot_init_yaw", init_robot_yaw, 0.0);
    tf::Quaternion init_robot_orientation;
    init_robot_orientation.setRPY(0.0, 0.0, init_robot_yaw);
    pnh.param<double>("robot_init_rot_x", rp.init_robot_pose.orientation.x, init_robot_orientation.x());
    pnh.param<double>("robot_init_rot_y", rp.init_robot_pose.orientation.y, init_robot_orientation.y());
    pnh.param<double>("robot_init_rot_z", rp.init_robot_pose.orientation.z, init_robot_orientation.z());
    pnh.param<double>("robot_init_rot_w", rp.init_robot_pose.orientation.w, init_robot_orientation.w());
    pnh.param<string>("robot_pose_control_msg", rp.robot_pose_control_msg, "command/pose");
    pnh.param<string>("robot_velo_control_msg", rp.robot_velo_control_msg, "/turtlebot0/mobile_base/commands/velocity");
    pnh.param<string>("odometry_msg", rp.odometry_msg, "/firefly/odometry_sensor1/pose");
    pnh.param<string>("map_msg", rp.map_msg, "/Tentabot_octomap");
    //pnh.param<string>("occupancy_sensor_msg", rp.occupancy_sensor_msg, "/cam1d435/camera/depth/color/points");

    // INITIALIZE AND SET PROCESS PARAMETERS
    Tentabot::ProcessParams pp;
    pnh.param<bool>("visu_flag", pp.visu_flag, true);
    pnh.param<double>("time_limit", pp.time_limit, 0);
    pnh.param<double>("dt", pp.nav_dt, 0.0);
    pnh.param<double>("goal_close_threshold", pp.goal_close_threshold, 0.0);

    // INITIALIZE AND GENERATE TRAJECTORY SAMPLING
    string traj_data_path;
    pnh.param<string>("trajectory_data_path", traj_data_path, "");
    TrajectorySamplingUtility tsu(nh, rp.robot_frame_name);
    if (traj_data_path != "")
    {
        tsu.read_trajectory_data(traj_data_path);
    }
    else
    {
        double tlen, tyaw, tpitch;
        int tsamp_cnt, tyaw_cnt, tpitch_cnt;
        string traj_samp_dataset_path;

        pnh.param<string>("trajectory_sampling_dataset_path", traj_samp_dataset_path, "");
        pnh.param<double>("tlen", tlen, 10);
        pnh.param<double>("tyaw", tyaw, 0.3*PI);
        pnh.param<double>("tpitch", tpitch, 0.2*PI);
        pnh.param<int>("tsamp_cnt", tsamp_cnt, 30);
        pnh.param<int>("tyaw_cnt", tyaw_cnt, 31);
        pnh.param<int>("tpitch_cnt", tpitch_cnt, 21);

        tsu.set_trajectory_sampling_dataset_path(traj_samp_dataset_path);
        tsu.set_trajectory_length(tlen);
        tsu.set_trajectory_yaw(tyaw);
        tsu.set_trajectory_pitch(tpitch);
        tsu.set_trajectory_sampling_count(tsamp_cnt);
        tsu.set_trajectory_yaw_sampling_count(tyaw_cnt);
        tsu.set_trajectory_pitch_sampling_count(tpitch_cnt);

        tsu.construct_trajectory_data_by_geometry(true);
        tsu.fill_trajectory_sampling_visu();
        tsu.save_trajectory_data();
    }

    // INITIALIZE AND SET OFFLINE TUNING PARAMETERS FOR GRID AND TENTACLES
    Tentabot::OffTuningParams offtp;
    offtp.tentacle_data_path = tsu.get_trajectory_data_path();
    offtp.tentacle_data = tsu.get_trajectory_data();
    pnh.param<double>("max_occupancy_belief_value", offtp.max_occupancy_belief_value, 100);
    pnh.param<double>("pdist_x_max", offtp.pdist_x_max, 0.2);
    pnh.param<double>("pdist_x_min", offtp.pdist_x_min, 0.2);
    pnh.param<double>("pdist_y_max", offtp.pdist_y_max, 0.2);
    pnh.param<double>("pdist_y_min", offtp.pdist_y_min, 0.2);
    pnh.param<double>("pdist_z_max", offtp.pdist_z_max, 0.2);
    pnh.param<double>("pdist_z_min", offtp.pdist_z_min, 0.2);
    pnh.param<double>("sdist_x_max", offtp.sdist_x_max, 0.4);
    pnh.param<double>("sdist_x_min", offtp.sdist_x_min, 0.4);
    pnh.param<double>("sdist_y_max", offtp.sdist_y_max, 0.4);
    pnh.param<double>("sdist_y_min", offtp.sdist_y_min, 0.4);
    pnh.param<double>("sdist_z_max", offtp.sdist_z_max, 0.4);
    pnh.param<double>("sdist_z_min", offtp.sdist_z_min, 0.4);
    pnh.param<double>("sweight_max", offtp.sweight_max, 1);
    pnh.param<double>("sweight_scale", offtp.sweight_scale, 10);
    pnh.param<double>("egrid_vdim", offtp.egrid_vdim, 0.1);

    // INITIALIZE AND SET ONLINE TUNING PARAMETERS FOR GRID AND TENTACLES
    Tentabot::OnTuningParams ontp;
    pnh.param<int>("tbin_obs_cnt_threshold", ontp.tbin_obs_cnt_threshold, 1);
    pnh.param<double>("crash_dist_scale", ontp.crash_dist_scale, 0.3);
    pnh.param<double>("clear_scale", ontp.clear_scale, 0);
    pnh.param<double>("clutter_scale", ontp.clutter_scale, 20);
    pnh.param<double>("close_scale", ontp.close_scale, 120);
    pnh.param<double>("smooth_scale", ontp.smooth_scale, 2);
    
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

    // NUA TODO: Why does this only work when it is before the sleep?
    ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>(rp.robot_pose_control_msg, 10);

    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(5.0).sleep();

    geometry_msgs::PoseStamped p;
    p.header.seq++;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = world_frame_name;
    p.pose = rp.init_robot_pose;
    trajectory_pub.publish(p);

    // INITIALIZE TENTABOT
    string nav_data_path;
    pnh.param<string>("nav_data_path", nav_data_path, "");
    Tentabot tbot(nh, listener, gu, rp, pp, offtp, ontp, nav_data_path);

    // SUBSCRIBE TO THE ODOMETRY DATA
    ros::Subscriber sub_odom = nh.subscribe(rp.odometry_msg, 1000, &Tentabot::robotPoseCallback, &tbot);

    // SUBSCRIBE TO THE MAP DATA (Octomap)
    ros::Subscriber sub_map = nh.subscribe(rp.map_msg, 1000, &Tentabot::mapCallback, &tbot);

    ros::Duration(5.0).sleep();

    // NAV LOOP
    ros::Timer timer = nh.createTimer(ros::Duration(pp.nav_dt), &Tentabot::mainCallback, &tbot);

    ros::spin();
}