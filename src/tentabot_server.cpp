// LAST UPDATE: 2022.04.03
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
    ros::init(argc, argv, "tentabot_server");
    
    // INITIALIZE THE MAIN ROS NODE HANDLE
    ros::NodeHandle nh;

    // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
    ros::NodeHandle pnh("~");

    // INITIALIZE TRANSFORM LISTENER
    tf::TransformListener* listener = new tf::TransformListener;

    //// SET PARAMETERS

    // INITIALIZE AND SET WORLD PARAMETERS
    string world_frame_name;
    pnh.param<string>("/world_frame_name", world_frame_name, "");
    cout << "tentabot_server::main -> world_frame_name: " << world_frame_name << endl;

    // INITIALIZE AND SET GOAL PARAMETERS
    string goal_msg;
    pnh.param<string>("/goal_msg", goal_msg, "");
    GoalUtility gu(nh, world_frame_name, "/nav_goal", goal_msg);
    
    if(goal_msg == "")
    {
        vector<double> goal_vector_x;
        vector<double> goal_vector_y;
        vector<double> goal_vector_z;
        int n_goal;
        string goal_name, goal_name_x;
        double goal_x, goal_y, goal_z;

        pnh.param<int>("/n_goal", n_goal, 0);
        cout << "tentabot_server::main -> n_goal: " << n_goal << endl;

        for (int i = 1; i <= n_goal; ++i)
        {
            goal_name = "/goal" + to_string(i);

            pnh.param<double>(goal_name + "_x", goal_x, 0.0);
            pnh.param<double>(goal_name + "_y", goal_y, 0.0);
            pnh.param<double>(goal_name + "_z", goal_z, 0.0);

            /*
            cout << "tentabot_server::main -> " << goal_name << "_x: " << goal_x << endl;
            cout << "tentabot_server::main -> " << goal_name << "_y: " << goal_y << endl;
            cout << "tentabot_server::main -> " << goal_name << "_z: " << goal_z << endl;
            */
            
            gu.addGoalPoint(goal_x, goal_y, goal_z);
        }
    }

    // INITIALIZE AND SET ROBOT PARAMETERS
    Tentabot::RobotParams rp;
    pnh.param<string>("/local_map_msg", rp.local_map_msg, "");
    pnh.param<string>("/world_name", rp.world_name, "");
    pnh.param<string>("/robot_name", rp.robot_name, "");
    pnh.param<string>("/robot_frame_name", rp.robot_frame_name, "");
    pnh.param<double>("/robot_bbx_x_min", rp.robot_bbx_x_min, 0.0);
    pnh.param<double>("/robot_bbx_x_max", rp.robot_bbx_x_max, 0.0);
    pnh.param<double>("/robot_bbx_y_min", rp.robot_bbx_y_min, 0.0);
    pnh.param<double>("/robot_bbx_y_max", rp.robot_bbx_y_max, 0.0);
    pnh.param<double>("/robot_bbx_z_min", rp.robot_bbx_z_min, 0.0);
    pnh.param<double>("/robot_bbx_z_max", rp.robot_bbx_z_max, 0.0);
    pnh.param<double>("/robot_max_lat_velo", rp.dummy_max_lat_velo, 0.0);
    pnh.param<double>("/robot_max_lat_acc", rp.dummy_max_lat_acc, 0.0);
    pnh.param<double>("/robot_max_yaw_velo", rp.dummy_max_yaw_velo, 0.0);
    pnh.param<double>("/robot_max_yaw_acc", rp.dummy_max_yaw_acc, 0.0);
    pnh.param<string>("/robot_odometry_msg", rp.robot_odometry_msg, "");
    pnh.param<string>("/robot_pose_control_msg", rp.robot_pose_control_msg, "");
    pnh.param<string>("/robot_velo_control_msg", rp.robot_velo_control_msg, "");

    // INITIALIZE AND SET GENERAL PARAMETERS
    string nav_data_path;
    pnh.param<string>("/heuristic_data_path", nav_data_path, "");

    // INITIALIZE AND SET PROCESS PARAMETERS
    Tentabot::ProcessParams pp;
    pnh.param<bool>("/visu_flag", pp.visu_flag, false);
    pnh.param<double>("/time_limit", pp.time_limit, 0);
    pnh.param<double>("/dt", pp.nav_dt, 0.0);
    pnh.param<double>("/goal_close_threshold", pp.goal_close_threshold, 0.0);
    pnh.param<bool>("/ground_collision_flag", pp.ground_collision_flag, false);
    pnh.param<double>("/ground_collision_threshold", pp.ground_collision_threshold, 0.0);
    pnh.param<bool>("/drl_service_flag", pp.drl_service_flag, false);

    // INITIALIZE AND GENERATE TRAJECTORY SAMPLING
    string traj_data_path;
    pnh.param<string>("/trajectory_data_path", traj_data_path, "");
    TrajectorySamplingUtility tsu(nh, rp.robot_frame_name);
    double ttime, tlen, tyaw, tpitch;
    int tsamp_cnt, tyaw_cnt, tpitch_cnt, lat_velo_samp_cnt, ang_velo_samp_cnt;
    string traj_samp_dataset_path, trajectory_gen_type;

    if (traj_data_path != "")
    {
        tsu.read_trajectory_data(traj_data_path);
        tsu.read_velocity_control_data(traj_data_path);
    }
    else
    {
        pnh.param<string>("/trajectory_sampling_dataset_path", traj_samp_dataset_path, "");
        pnh.param<double>("/tlen", tlen, 0);
        pnh.param<int>("/tsamp_cnt", tsamp_cnt, 0);
        pnh.param<string>("/trajectory_gen_type", trajectory_gen_type, "");

        tsu.set_trajectory_sampling_dataset_path(traj_samp_dataset_path);
        tsu.set_trajectory_length(tlen);
        tsu.set_trajectory_sampling_count(tsamp_cnt);

        if (trajectory_gen_type == "geometric")
        {
            pnh.param<double>("/tyaw", tyaw, 0.0);
            pnh.param<double>("/tpitch", tpitch, 0.0);
            pnh.param<int>("/tyaw_cnt", tyaw_cnt, 0);
            pnh.param<int>("/tpitch_cnt", tpitch_cnt, 0);

            tsu.set_trajectory_yaw(tyaw);
            tsu.set_trajectory_pitch(tpitch);
            tsu.set_trajectory_yaw_sampling_count(tyaw_cnt);
            tsu.set_trajectory_pitch_sampling_count(tpitch_cnt);

            tsu.construct_trajectory_data_by_geometry();
        }
        else if (trajectory_gen_type == "kinematic")
        {
            pnh.param<double>("/ttime", ttime, 0);
            pnh.param<int>("/lat_velo_samp_cnt", lat_velo_samp_cnt, 0);
            pnh.param<int>("/ang_velo_samp_cnt", ang_velo_samp_cnt, 0);

            tsu.set_trajectory_time(ttime);
            tsu.set_lateral_velocity_sampling_count(lat_velo_samp_cnt);
            tsu.set_angular_velocity_sampling_count(ang_velo_samp_cnt);

            tsu.construct_trajectory_data_by_simple_car_model(rp.dummy_max_lat_velo, rp.dummy_max_yaw_velo, 0.01);
        }
        else
        {
            cout << "tentabot_server::main -> ERROR: trajectory_gen_type is not defined!" << endl;
        }
        
        tsu.fill_trajectory_sampling_visu();
        tsu.save_trajectory_data();
    }

    // INITIALIZE AND SET OFFLINE TUNING PARAMETERS FOR GRID AND TENTACLES
    Tentabot::OffTuningParams offtp;
    offtp.tentacle_data_path = tsu.get_trajectory_data_path();
    offtp.tentacle_data = tsu.get_trajectory_data();
    offtp.velocity_control_data = tsu.get_velocity_control_data();

    pnh.param<double>("/max_occupancy_belief_value", offtp.max_occupancy_belief_value, 100);
    pnh.param<double>("/sweight_max", offtp.sweight_max, 0);
    pnh.param<double>("/sweight_scale", offtp.sweight_scale, 0);
    pnh.param<double>("/egrid_vdim", offtp.egrid_vdim, 0.0);
    pnh.param<double>("/pdist_x_max", offtp.pdist_x_max, 0.0);
    pnh.param<double>("/pdist_x_min", offtp.pdist_x_min, 0.0);
    pnh.param<double>("/pdist_y_max", offtp.pdist_y_max, 0.0);
    pnh.param<double>("/pdist_y_min", offtp.pdist_y_min, 0.0);
    pnh.param<double>("/pdist_z_max", offtp.pdist_z_max, 0.0);
    pnh.param<double>("/pdist_z_min", offtp.pdist_z_min, 0.0);
    pnh.param<double>("/sdist_x_max", offtp.sdist_x_max, 0.0);
    pnh.param<double>("/sdist_x_min", offtp.sdist_x_min, 0.0);
    pnh.param<double>("/sdist_y_max", offtp.sdist_y_max, 0.0);
    pnh.param<double>("/sdist_y_min", offtp.sdist_y_min, 0.0);
    pnh.param<double>("/sdist_z_max", offtp.sdist_z_max, 0.0);
    pnh.param<double>("/sdist_z_min", offtp.sdist_z_min, 0.0);

    // INITIALIZE AND SET ONLINE TUNING PARAMETERS FOR GRID AND TENTACLES
    Tentabot::OnTuningParams ontp;
    pnh.param<int>("/tbin_obs_cnt_threshold", ontp.tbin_obs_cnt_threshold, 0);
    pnh.param<double>("/crash_dist_scale", ontp.crash_dist_scale, 0.0);
    pnh.param<double>("/clear_scale", ontp.clear_scale, 0);
    pnh.param<double>("/clutter_scale", ontp.clutter_scale, 0);
    pnh.param<double>("/occupancy_scale", ontp.occupancy_scale, 0);
    pnh.param<double>("/close_scale", ontp.close_scale, 0);
    pnh.param<double>("/smooth_scale", ontp.smooth_scale, 0);

    /*
    cout << "tentabot_server::main -> local_map_msg: " << rp.local_map_msg << endl;
    cout << "tentabot_server::main -> world_name: " << rp.world_name << endl;
    cout << "tentabot_server::main -> robot_name: " << rp.robot_name << endl;
    cout << "tentabot_server::main -> robot_frame_name: " << rp.robot_frame_name << endl;
    cout << "tentabot_server::main -> robot_bbx_x_min: " << rp.robot_bbx_x_min << endl;
    cout << "tentabot_server::main -> robot_bbx_x_max: " << rp.robot_bbx_x_max << endl;
    cout << "tentabot_server::main -> robot_bbx_y_min: " << rp.robot_bbx_y_min << endl;
    cout << "tentabot_server::main -> robot_bbx_y_max: " << rp.robot_bbx_y_max << endl;
    cout << "tentabot_server::main -> robot_bbx_z_min: " << rp.robot_bbx_z_min << endl;
    cout << "tentabot_server::main -> robot_bbx_z_max: " << rp.robot_bbx_z_max << endl;
    cout << "tentabot_server::main -> robot_max_lat_velo: " << rp.dummy_max_lat_velo << endl;
    cout << "tentabot_server::main -> robot_max_lat_acc: " << rp.dummy_max_lat_acc << endl;
    cout << "tentabot_server::main -> robot_max_yaw_velo: " << rp.dummy_max_yaw_velo << endl;
    cout << "tentabot_server::main -> robot_max_yaw_acc: " << rp.dummy_max_yaw_acc << endl;
    cout << "tentabot_server::main -> robot_pose_control_msg: " << rp.robot_pose_control_msg << endl;
    cout << "tentabot_server::main -> robot_velo_control_msg: " << rp.robot_velo_control_msg << endl;
    cout << "tentabot_server::main -> robot_odometry_msg: " << rp.robot_odometry_msg << endl;
    cout << "tentabot_server::main -> nav_data_path: " << nav_data_path << endl;
    cout << "tentabot_server::main -> visu_flag: " << pp.visu_flag << endl;
    cout << "tentabot_server::main -> time_limit: " << pp.time_limit << endl;
    cout << "tentabot_server::main -> dt: " << pp.nav_dt << endl;
    cout << "tentabot_server::main -> goal_close_threshold: " << pp.goal_close_threshold << endl;
    cout << "tentabot_server::main -> ground_collision_flag: " << pp.ground_collision_flag << endl;
    cout << "tentabot_server::main -> ground_collision_threshold: " << pp.ground_collision_threshold << endl;
    cout << "tentabot_server::main -> drl_service_flag: " << pp.drl_service_flag << endl;
    cout << "tentabot_server::main -> traj_data_path: " << traj_data_path << endl;
    cout << "tentabot_server::main -> traj_samp_dataset_path: " << traj_samp_dataset_path << endl;
    cout << "tentabot_server::main -> tlen: " << tlen << endl;
    cout << "tentabot_server::main -> tyaw: " << tyaw << endl;
    cout << "tentabot_server::main -> tpitch: " << tpitch << endl;
    cout << "tentabot_server::main -> tsamp_cnt: " << tsamp_cnt << endl;
    cout << "tentabot_server::main -> tyaw_cnt: " << tyaw_cnt << endl;
    cout << "tentabot_server::main -> tpitch_cnt: " << tpitch_cnt << endl;
    cout << "tentabot_server::main -> max_occupancy_belief_value: " << offtp.max_occupancy_belief_value << endl;
    cout << "tentabot_server::main -> sweight_max: " << offtp.sweight_max << endl;
    cout << "tentabot_server::main -> sweight_scale: " << offtp.sweight_scale << endl;
    cout << "tentabot_server::main -> egrid_vdim: " << offtp.egrid_vdim << endl;
    cout << "tentabot_server::main -> tbin_obs_cnt_threshold: " << ontp.tbin_obs_cnt_threshold << endl;
    cout << "tentabot_server::main -> crash_dist_scale: " << ontp.crash_dist_scale << endl;
    cout << "tentabot_server::main -> pdist_x_min: " << offtp.pdist_x_min << endl;
    cout << "tentabot_server::main -> pdist_x_max: " << offtp.pdist_x_max << endl;
    cout << "tentabot_server::main -> pdist_y_min: " << offtp.pdist_y_min << endl;
    cout << "tentabot_server::main -> pdist_y_max: " << offtp.pdist_y_max << endl;
    cout << "tentabot_server::main -> pdist_z_min: " << offtp.pdist_z_min << endl;
    cout << "tentabot_server::main -> pdist_z_max: " << offtp.pdist_z_max << endl;
    cout << "tentabot_server::main -> sdist_x_min: " << offtp.sdist_x_min << endl;
    cout << "tentabot_server::main -> sdist_x_max: " << offtp.sdist_x_max << endl;
    cout << "tentabot_server::main -> sdist_y_min: " << offtp.sdist_y_min << endl;
    cout << "tentabot_server::main -> sdist_y_max: " << offtp.sdist_y_max << endl;
    cout << "tentabot_server::main -> sdist_z_min: " << offtp.sdist_z_min << endl;
    cout << "tentabot_server::main -> sdist_z_max: " << offtp.sdist_z_max << endl;
    cout << "tentabot_server::main -> clear_scale: " << ontp.clear_scale << endl;
    cout << "tentabot_server::main -> clutter_scale: " << ontp.clutter_scale << endl;
    cout << "tentabot_server::main -> occupancy_scale: " << ontp.occupancy_scale << endl;
    cout << "tentabot_server::main -> close_scale: " << ontp.close_scale << endl;
    cout << "tentabot_server::main -> smooth_scale: " << ontp.smooth_scale << endl;
    */

    // INITIALIZE TENTABOT
    Tentabot tbot(nh, listener, gu, rp, pp, offtp, ontp, nav_data_path);

    ros::Duration(2.0).sleep();

    ros::Timer timer;

    if (!pp.drl_service_flag)
    {
        // NAV LOOP
        timer = nh.createTimer(ros::Duration(pp.nav_dt), &Tentabot::mainCallback, &tbot);
    }

    ros::spin();
}
