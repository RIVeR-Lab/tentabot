// LAST UPDATE: 2021.10.08
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:

// --CUSTOM LIBRARIES--
#include "trajectory_sampling_utility.h"
#include "tentabot.h"

int main(int argc, char** argv)
{
    // INITIALIZE ROS
    ros::init(argc, argv, "tentabot_rl_service");
    
    // INITIALIZE THE MAIN ROS NODE HANDLE
    ros::NodeHandle nh;

    // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
    ros::NodeHandle pnh("~");

    //ros::AsyncSpinner spinner(0);
    //spinner.start();

    // INITIALIZE TRANSFORM LISTENER
    tf::TransformListener* listener = new tf::TransformListener;

    //// SET THE PARAMETERS

    string world_frame_name;
    pnh.param<string>("world_frame_name", world_frame_name, "");

    // INITIALIZE AND SET GOAL PARAMETERS
    string goal_msg;
    pnh.param<string>("goal_msg", goal_msg, "");
    GoalUtility gu(nh, world_frame_name, "subGoal", goal_msg);

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
    pnh.param<double>("robot_max_lat_velo", rp.dummy_max_lat_velo, 0);
    pnh.param<double>("robot_max_lat_acc", rp.dummy_max_lat_acc, 0);
    pnh.param<double>("robot_max_yaw_velo", rp.dummy_max_yaw_velo, 0);
    pnh.param<double>("robot_max_yaw_acc", rp.dummy_max_yaw_acc, 0);
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
    pnh.param<string>("robot_pose_control_msg", rp.robot_pose_control_msg, "");
    pnh.param<string>("robot_velo_control_msg", rp.robot_velo_control_msg, "");
    pnh.param<string>("odometry_msg", rp.odometry_msg, "");
    pnh.param<string>("map_msg", rp.map_msg, "");

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
        tsu.read_velocity_control_data(traj_data_path);
        tsu.save_input_data();
    }
    else
    {
        double ttime, tlen, tyaw, tpitch;
        int tsamp_cnt, lat_velo_samp_cnt, tyaw_cnt, tpitch_cnt;
        string traj_samp_dataset_path, traj_gen_type;

        pnh.param<string>("trajectory_sampling_dataset_path", traj_samp_dataset_path, "");
        pnh.param<string>("trajectory_gen_type", traj_gen_type, "");
        pnh.param<double>("ttime", ttime, 0.0);
        pnh.param<double>("tlen", tlen, 0.0);
        pnh.param<double>("tyaw", tyaw, 0.0);
        pnh.param<double>("tpitch", tpitch, 0.0);
        pnh.param<int>("tsamp_cnt", tsamp_cnt, 0);
        pnh.param<int>("lat_velo_samp_cnt", lat_velo_samp_cnt, 0);
        pnh.param<int>("tyaw_cnt", tyaw_cnt, 0);
        pnh.param<int>("tpitch_cnt", tpitch_cnt, 0);

        tsu.set_trajectory_sampling_dataset_path(traj_samp_dataset_path);
        tsu.set_trajectory_time(ttime);
        tsu.set_trajectory_length(tlen);
        tsu.set_trajectory_yaw(tyaw);
        tsu.set_trajectory_pitch(tpitch);
        tsu.set_trajectory_sampling_count(tsamp_cnt);
        tsu.set_lateral_velocity_sampling_count(lat_velo_samp_cnt);
        tsu.set_trajectory_yaw_sampling_count(tyaw_cnt);
        tsu.set_trajectory_pitch_sampling_count(tpitch_cnt);

        if (traj_gen_type == "geometry")
        {
            tsu.construct_trajectory_data_by_geometry();
        }
        else if(traj_gen_type == "kinematic")
        {
            tsu.construct_trajectory_data_by_simple_car_model(rp.dummy_max_lat_velo, rp.dummy_max_yaw_velo, 0.01);
        }

        tsu.fill_trajectory_sampling_visu();
        tsu.save_trajectory_data();
    }

    // INITIALIZE AND SET OFFLINE TUNING PARAMETERS FOR GRID AND TENTACLES
    Tentabot::OffTuningParams offtp;
    offtp.tentacle_data_path = tsu.get_trajectory_data_path();
    offtp.tentacle_data = tsu.get_trajectory_data();
    offtp.velocity_control_data = tsu.get_velocity_control_data();

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

    // INITIALIZE TENTABOT
    string nav_bench_path;
    pnh.param<string>("nav_bench_path", nav_bench_path, "");
    Tentabot tbot(nh, listener, gu, rp, pp, offtp, ontp, nav_bench_path);

    // SUBSCRIBE TO THE ODOMETRY DATA
    ros::Subscriber sub_odom;
    if (rp.robot_name == "firefly")
    {
        sub_odom = nh.subscribe(rp.odometry_msg, 1000, &Tentabot::robotPoseCallback, &tbot);
    }
    else if (rp.robot_name == "turtlebot0")
    {
        sub_odom = nh.subscribe(rp.odometry_msg, 1000, &Tentabot::odometryCallback, &tbot);
    }

    // SUBSCRIBE TO THE MAP DATA (Octomap)
    ros::Subscriber sub_map = nh.subscribe(rp.map_msg, 1000, &Tentabot::mapCallback, &tbot);

    // NAV SERVICE
    ros::ServiceServer service_rl_step = nh.advertiseService("rl_step", &Tentabot::rl_step, &tbot);
    ros::ServiceServer service_update_goal = nh.advertiseService("update_goal", &Tentabot::update_goal, &tbot);

    ros::spin();
    //ros::waitForShutdown();
}