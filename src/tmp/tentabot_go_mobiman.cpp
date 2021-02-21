// LAST UPDATE: 2021.02.19
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION:
//

#include "tentabot_go_mobiman.h"

int main(int argc, char** argv)
{
    // INITIALIZE ROS
    ros::init(argc, argv, "tentabot_go2");
    
    // INITIALIZE THE MAIN ROS NODE HANDLE
    ros::NodeHandle nh;

    // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
    ros::NodeHandle pnh("~");

    
    // INITIALIZE TRANSFORM LISTENER
    tf::TransformListener* listener = new tf::TransformListener;

    //// SET THE PARAMETERS
    // INITIALIZE AND SET WORLD FRAME NAME
    string world_frame_name;
    pnh.param<string>("world_frame_name", world_frame_name, "map");

    // INITIALIZE AND SET GOAL PARAMETERS
    GoalUtility gu(nh, world_frame_name, "goal");
    gu.addGoalPoint(5, 0, 0);

    // INITIALIZE AND SET MAP PARAMETERS
    double resolution;
    pnh.param("resolution", resolution, 0.15);
    string map_name;
    pnh.param<string>("world_name", map_name, "map");
    MapUtility mu(nh, resolution, world_frame_name, map_name);

    // INITIALIZE AND SET PROCESS PARAMETERS
    ProcessParams pp;
    pnh.param<bool>("visu_flag", pp.visu_flag, true);
    pnh.param<bool>("online_tuning_flag", pp.online_tuning_flag, false);
    pnh.param<double>("time_limit", pp.time_limit, 90);
    pnh.param<double>("dt", pp.nav_dt, 0.1);
    pnh.param<bool>("navexit_flag", pp.navexit_flag, false);
    pnh.param<double>("goal_close_threshold", pp.goal_close_threshold, 0.5);

    // INITIALIZE AND SET ROBOT PARAMETERS
    double start_x, start_y, start_z, start_yaw;
    pnh.param("start_x", start_x, 0.0);
    pnh.param("start_y", start_y, 0.0);
    pnh.param("start_z", start_z, 0.0);
    pnh.param("start_yaw", start_yaw, 0.0);
    tf::Quaternion start_orientation;
    start_orientation.setRPY(0.0, 0.0, start_yaw);
    
    RobotParams rp;
    pnh.param<double>("robot_width", rp.width, 0.8);
    pnh.param<double>("robot_length", rp.length, 1.3);
    pnh.param<double>("robot_height", rp.height, 2.0);
    pnh.param<double>("max_lat_velo", rp.max_lat_velo, 1);
    pnh.param<double>("max_yaw_velo", rp.max_yaw_velo, 2);
    pnh.param<double>("max_pitch_velo", rp.max_pitch_velo, 1);
    pnh.param<double>("max_roll_velo", rp.max_roll_velo, 1);
    pnh.param<double>("init_robot_pos_x", rp.init_robot_pose.position.x, start_x);
    pnh.param<double>("init_robot_pos_y", rp.init_robot_pose.position.y, start_y);
    pnh.param<double>("init_robot_pos_z", rp.init_robot_pose.position.z, start_z);
    pnh.param<double>("init_robot_ori_x", rp.init_robot_pose.orientation.x, start_orientation.x());
    pnh.param<double>("init_robot_ori_y", rp.init_robot_pose.orientation.y, start_orientation.y());
    pnh.param<double>("init_robot_ori_z", rp.init_robot_pose.orientation.z, start_orientation.z());
    pnh.param<double>("init_robot_ori_w", rp.init_robot_pose.orientation.w, start_orientation.w());
    pnh.param<string>("robot_frame_name", rp.robot_frame_name, "base_link");
    pnh.param<string>("mav_name", rp.robot_name, "tentabot");
    pnh.param<double>("nav_sensor_freq", rp.nav_sensor.freq, 30);
    pnh.param<double>("nav_sensor_resolution", rp.nav_sensor.resolution, resolution);
    pnh.param<double>("nav_sensor_range_x_min", rp.nav_sensor.range_x[0], 0.3);
    pnh.param<double>("nav_sensor_range_x_max", rp.nav_sensor.range_x[1], 5);
    pnh.param<double>("nav_sensor_range_y_min", rp.nav_sensor.range_y[0], 0.3);
    pnh.param<double>("nav_sensor_range_y_max", rp.nav_sensor.range_y[1], 5);
    pnh.param<double>("nav_sensor_range_z_min", rp.nav_sensor.range_z[0], 0.3);
    pnh.param<double>("nav_sensor_range_z_max", rp.nav_sensor.range_z[1], 5);
    pnh.param<string>("nav_sensor_frame_name", rp.nav_sensor.frame_name, "firefly/camera_depth_optical_frame");
    
    string cloud_in_topic1 = "/d435_cam1d435/camera/depth_registered/points";
    string cloud_in_topic2 = "/d435_cam2d435/camera/depth_registered/points";
    //pnh.getParam("/cloud_in_topic1", cloud_in_topic1);
    //pnh.getParam("/cloud_in_topic2", cloud_in_topic2);

    // INITIALIZE AND SET OFFLINE TUNING PARAMETERS FOR GRID AND TENTACLES
    OffTuningParams offtp;
    pnh.param<int>("tyaw_cnt", offtp.tyaw_cnt, 11);
    pnh.param<int>("tpitch_cnt", offtp.tpitch_cnt, 1);
    pnh.param<int>("troll_cnt", offtp.troll_cnt, 0);
    pnh.param<int>("tsamp_cnt", offtp.tsamp_cnt, 5);
    pnh.param<int>("tlat_velo_cnt", offtp.tlat_velo_cnt, 1);
    pnh.param<int>("tyaw_velo_cnt", offtp.tyaw_velo_cnt, 1);
    pnh.param<int>("tpitch_velo_cnt", offtp.tpitch_velo_cnt, 1);
    pnh.param<int>("troll_velo_cnt", offtp.troll_velo_cnt, 1);
    pnh.param<double>("tlen", offtp.tlen, 5);
    pnh.param<double>("tyaw", offtp.tyaw, 0.3*PI);
    pnh.param<double>("tpitch", offtp.tpitch, 0.0);
    pnh.param<double>("troll", offtp.troll, 0.0);
    pnh.param<string>("tentacle_type", offtp.tentacle_type, "linear");
    pnh.param<string>("tyaw_samp_type", offtp.tyaw_samp_type, "lin");
    pnh.param<string>("tpitch_samp_type", offtp.tpitch_samp_type, "lin");
    pnh.param<string>("troll_samp_type", offtp.troll_samp_type, "lin");
    pnh.param<double>("pdist", offtp.pdist, 0.4);
    pnh.param<double>("sdist", offtp.sdist, 1);
    pnh.param<double>("sweight_max", offtp.sweight_max, 1);
    pnh.param<double>("sweight_scale", offtp.sweight_scale, 10);
    pnh.param<double>("egrid_vdim", offtp.egrid_vdim, 0.1);
    pnh.param<int>("egrid_vnumx", offtp.egrid_vnumx, 50);
    pnh.param<int>("egrid_vnumy", offtp.egrid_vnumy, 50);
    pnh.param<int>("egrid_vnumz", offtp.egrid_vnumz, 50);

    // INITIALIZE AND SET ONLINE TUNING PARAMETERS FOR GRID AND TENTACLES
    OnTuningParams ontp;
    pnh.param<double>("crash_dist", ontp.crash_dist, 8);
    pnh.param<double>("clear_scale", ontp.clear_scale, 50);
    pnh.param<double>("flat_scale", ontp.flat_scale, 100);
    pnh.param<double>("close_scale", ontp.close_scale, 80);
    pnh.param<double>("smooth_scale", ontp.smooth_scale, 0.5);
    ontp.tbin_window = 1;
    ontp.tbin_obs_cnt_threshold = 1;

    // INITIALIZE TENTABOT OBJECT
    Tentabot tbot(nh, listener, pp, rp, offtp, ontp, mu, gu);

    // SUBSCRIBE TO THE ODOMETRY DATA
    //ros::Subscriber sub = nh.subscribe("/firefly/odometry_sensor1/pose", 10, &Tentabot::odometryCallback, &tbot);
    ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 10, &Tentabot::odometryCallback2, &tbot);
    
    // SUBSCRIBE TO THE DEPTH SENSOR DATA
    //message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
    //depth_image_sub_.subscribe(nh, "camera/depth/image_raw", 10);
    //tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, world_frame_name, 10);
    //tf_filter_.registerCallback(&Tentabot::depthImageCallback, &tbot);
    //ros::Subscriber pc1_sub = nh.subscribe("/d435_cam1d435/camera/depth_registered/points", 10, &Tentabot::pcCallback1, &tbot);
    //ros::Subscriber pc2_sub = nh.subscribe("/d435_cam2d435/camera/depth_registered/points", 10, &Tentabot::pcCallback2, &tbot);
    //ros::Subscriber pc_sub = nh.subscribe("/concat_pc", 10, &Tentabot::pcCallback, &tbot);

    // Create a ROS subscriber for the input point cloud
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_in_topic1_sub(nh, cloud_in_topic1, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_in_topic2_sub(nh, cloud_in_topic2, 1);

    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(cloud_in_topic1_sub, cloud_in_topic2_sub, 10);
    sync.registerCallback(boost::bind(&Tentabot::pc_callback, &tbot, _1, _2));

    // NAV LOOP
    ros::Timer timer = nh.createTimer(ros::Duration(pp.nav_dt), &Tentabot::sendCommandCallback, &tbot);

    ros::spin();
}
