// LAST UPDATE: 2021.10.15
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] N. Ü. Akmandor and T. Padir, "A 3D Reactive Navigation Algorithm 
//     for Mobile Robots by Using Tentacle-Based Sampling," 2020 Fourth 
//     IEEE International Conference on Robotic Computing (IRC), Taichung, 
//     Taiwan, 2020, pp. 9-16, doi: 10.1109/IRC.2020.00009.
// [2] F. von Hundelshausen, M. Himmelsbach, F. Hecker, A. Mueller, and 
//     H.-J. Wuensche. Driving with tentacles: Integral structures for 
//     sensing and motion. Journal of Field Robotics, 25(9):640–673, 2008.
//
// NUA TODO:

// --CUSTOM LIBRARIES--
#include "tentabot.h"

Tentabot::Tentabot( NodeHandle& nh,
                    tf::TransformListener* listener,
                    GoalUtility& gu,
                    RobotParams& rp,
                    ProcessParams& pp,
                    OffTuningParams& offtp,
                    OnTuningParams& ontp,
                    string data_path)
{
  cout << "Welcome to Tentabot Navigation Simulation! I hope you'll enjoy the experience..." << endl;

  tflistener = new tf::TransformListener;
  tflistener = listener;

  goal_util = gu;
  setRobotParams(rp);
  setProcessParams(pp);
  setOffTuningParams(offtp);
  setOnTuningParams(ontp);
  setDataPath(data_path);

  initialize(nh);
}

Tentabot::~Tentabot()                                // destructor
{
  //ROS_INFO( "Calling Destructor for Tentabot..." );
  delete tflistener;
  mid_data_stream.close();
  post_data_stream.close();
} 

// GET FUNCTIONS
Tentabot::ProcessParams Tentabot::getProcessParams()
{
  return process_param;
}

Tentabot::RobotParams Tentabot::getRobotParams()
{
  return robot_param;
}

Tentabot::OffTuningParams Tentabot::getOffTuningParams()
{
  return off_tuning_param;
}

Tentabot::OnTuningParams Tentabot::getOnTuningParams()
{
  return on_tuning_param;
}

Tentabot::HeuristicParams Tentabot::getHeuristicParams()
{
  return heuristic_param;
}

Tentabot::StatusParams Tentabot::getStatusParams()
{
  return status_param;
}

Tentabot::VisuParams Tentabot::getVisuParams()
{
  return visu_param;
}

// SET FUNCTIONS
void Tentabot::setProcessParams(Tentabot::ProcessParams new_process_param)
{
  process_param.visu_flag = new_process_param.visu_flag;
  process_param.time_limit = new_process_param.time_limit;
  process_param.nav_dt = new_process_param.nav_dt;
  process_param.goal_close_threshold = new_process_param.goal_close_threshold;
}

void Tentabot::setRobotParams(Tentabot::RobotParams new_robot_param)
{
  robot_param.world_name = new_robot_param.world_name;
  robot_param.robot_name = new_robot_param.robot_name;
  robot_param.robot_frame_name = new_robot_param.robot_frame_name;
  robot_param.sensor_frame_name = new_robot_param.sensor_frame_name;
  robot_param.robot_bbx_x_max = new_robot_param.robot_bbx_x_max;
  robot_param.robot_bbx_x_min = new_robot_param.robot_bbx_x_min;
  robot_param.robot_bbx_y_max = new_robot_param.robot_bbx_y_max;
  robot_param.robot_bbx_y_min = new_robot_param.robot_bbx_y_min;
  robot_param.robot_bbx_z_max = new_robot_param.robot_bbx_z_max;
  robot_param.robot_bbx_z_min = new_robot_param.robot_bbx_z_min;
  robot_param.dummy_max_lat_velo = new_robot_param.dummy_max_lat_velo;
  robot_param.dummy_max_lat_acc = new_robot_param.dummy_max_lat_acc;
  robot_param.dummy_max_yaw_velo = new_robot_param.dummy_max_yaw_velo; 
  robot_param.dummy_max_yaw_acc = new_robot_param.dummy_max_yaw_acc;
  robot_param.init_robot_pose = new_robot_param.init_robot_pose;
  robot_param.robot_pose_control_msg = new_robot_param.robot_pose_control_msg;
  robot_param.robot_velo_control_msg = new_robot_param.robot_velo_control_msg;
  robot_param.odometry_msg = new_robot_param.odometry_msg;
  robot_param.map_msg = new_robot_param.map_msg;
}

void Tentabot::setOffTuningParams(Tentabot::OffTuningParams new_off_tuning_param)
{
  off_tuning_param.tentacle_data_path = new_off_tuning_param.tentacle_data_path;
  off_tuning_param.tentacle_data = new_off_tuning_param.tentacle_data;
  off_tuning_param.velocity_control_data = new_off_tuning_param.velocity_control_data;
  off_tuning_param.max_occupancy_belief_value = new_off_tuning_param.max_occupancy_belief_value;
  off_tuning_param.pdist_x_max = new_off_tuning_param.pdist_x_max;
  off_tuning_param.pdist_x_min = new_off_tuning_param.pdist_x_min;
  off_tuning_param.pdist_y_max = new_off_tuning_param.pdist_y_max;
  off_tuning_param.pdist_y_min = new_off_tuning_param.pdist_y_min;
  off_tuning_param.pdist_z_max = new_off_tuning_param.pdist_z_max;
  off_tuning_param.pdist_z_min = new_off_tuning_param.pdist_z_min;
  off_tuning_param.sdist_x_max = new_off_tuning_param.sdist_x_max;
  off_tuning_param.sdist_x_min = new_off_tuning_param.sdist_x_min;
  off_tuning_param.sdist_y_max = new_off_tuning_param.sdist_y_max;
  off_tuning_param.sdist_y_min = new_off_tuning_param.sdist_y_min;
  off_tuning_param.sdist_z_max = new_off_tuning_param.sdist_z_max;
  off_tuning_param.sdist_z_min = new_off_tuning_param.sdist_z_min;
  off_tuning_param.sweight_max = new_off_tuning_param.sweight_max;
  off_tuning_param.sweight_scale = new_off_tuning_param.sweight_scale;
  off_tuning_param.egrid_vdim = new_off_tuning_param.egrid_vdim;
}

void Tentabot::setOnTuningParams(Tentabot::OnTuningParams new_on_tuning_param)
{
  on_tuning_param.tbin_obs_cnt_threshold = new_on_tuning_param.tbin_obs_cnt_threshold;
  on_tuning_param.clear_scale = new_on_tuning_param.clear_scale;
  on_tuning_param.clutter_scale = new_on_tuning_param.clutter_scale;
  on_tuning_param.close_scale = new_on_tuning_param.close_scale;
  on_tuning_param.smooth_scale = new_on_tuning_param.smooth_scale;
  on_tuning_param.crash_dist_scale = new_on_tuning_param.crash_dist_scale;
}

void Tentabot::setHeuristicParams(Tentabot::HeuristicParams new_heuristic_param)
{
  heuristic_param.occupancy_set = new_heuristic_param.occupancy_set;
  heuristic_param.navigability_set = new_heuristic_param.navigability_set;
  heuristic_param.clearance_set = new_heuristic_param.clearance_set;
  heuristic_param.clutterness_set = new_heuristic_param.clutterness_set;
  heuristic_param.closeness_set = new_heuristic_param.closeness_set;
  heuristic_param.smoothness_set = new_heuristic_param.smoothness_set;
}

void Tentabot::setStatusParams(Tentabot::StatusParams new_status_param)
{
  status_param.tentacle_length_data = new_status_param.tentacle_length_data;
  status_param.tentacle_bbx_min = new_status_param.tentacle_bbx_min;
  status_param.tentacle_bbx_max = new_status_param.tentacle_bbx_max;
  status_param.egrid_vnum_x_max = new_status_param.egrid_vnum_x_max;
  status_param.egrid_vnum_x_min = new_status_param.egrid_vnum_x_min;
  status_param.egrid_vnum_y_max = new_status_param.egrid_vnum_y_max;
  status_param.egrid_vnum_y_min = new_status_param.egrid_vnum_y_min;
  status_param.egrid_vnum_z_max = new_status_param.egrid_vnum_z_max;
  status_param.egrid_vnum_z_min = new_status_param.egrid_vnum_z_min;
  status_param.egrid_dist_x_max = new_status_param.egrid_dist_x_max;
  status_param.egrid_dist_x_min = new_status_param.egrid_dist_x_min;
  status_param.egrid_dist_y_max = new_status_param.egrid_dist_y_max;
  status_param.egrid_dist_y_min = new_status_param.egrid_dist_y_min;
  status_param.egrid_dist_z_max = new_status_param.egrid_dist_z_max;
  status_param.egrid_dist_z_min = new_status_param.egrid_dist_z_min;
  status_param.ego_grid_data.ovox_pos = new_status_param.ego_grid_data.ovox_pos;
  status_param.ego_grid_data.ovox_value = new_status_param.ego_grid_data.ovox_value;
  status_param.support_vox_data = new_status_param.support_vox_data;
  status_param.sample_weight_data = new_status_param.sample_weight_data;
  status_param.tentacle_weight_data = new_status_param.tentacle_weight_data;
  status_param.measured_map_msg = new_status_param.measured_map_msg;
  status_param.measured_robot_pose = new_status_param.measured_robot_pose;
  status_param.map_frame_name = new_status_param.map_frame_name;
  status_param.tmap = new_status_param.tmap;
  status_param.transform_robot_wrt_world = new_status_param.transform_robot_wrt_world;
  status_param.robot_pose = new_status_param.robot_pose;
  status_param.prev_robot_pose = new_status_param.prev_robot_pose;
  status_param.prev_time = new_status_param.prev_time;
  status_param.dt = new_status_param.dt;
  status_param.lat_speed = new_status_param.lat_speed;
  status_param.yaw_velo = new_status_param.yaw_velo;
  status_param.command_pose = new_status_param.command_pose;
  status_param.command_velo = new_status_param.command_velo;
  status_param.tcrash_bin = new_status_param.tcrash_bin;
  status_param.navigability_flag = new_status_param.navigability_flag;
  status_param.best_tentacle = new_status_param.best_tentacle;
  status_param.ex_best_tentacle = new_status_param.ex_best_tentacle;
  status_param.ex_best_sample = new_status_param.ex_best_sample;
  status_param.nav_result = new_status_param.nav_result;
  status_param.nav_length = new_status_param.nav_length;
  status_param.nav_duration = new_status_param.nav_duration;
  status_param.command_pose_pub = new_status_param.command_pose_pub;
  status_param.command_velo_pub = new_status_param.command_velo_pub;
  status_param.counter = new_status_param.counter;
  status_param.navexit_flag = new_status_param.navexit_flag;
}

void Tentabot::setVisuParams(Tentabot::VisuParams new_visu_param)
{
  visu_param.robot_visu_pub = new_visu_param.robot_visu_pub;
  visu_param.tentacle_visu_pub = new_visu_param.tentacle_visu_pub;
  visu_param.best_tentacle_visu_pub = new_visu_param.best_tentacle_visu_pub;
  visu_param.tsamp_visu_pub = new_visu_param.tsamp_visu_pub;
  visu_param.support_vox_visu_pub = new_visu_param.support_vox_visu_pub;
  visu_param.path_visu_pub = new_visu_param.path_visu_pub;
  visu_param.command_visu_pub = new_visu_param.command_visu_pub;
  visu_param.debug_visu_pub = new_visu_param.debug_visu_pub;
  visu_param.robot_visu = new_visu_param.robot_visu;
  visu_param.tentacle_visu = new_visu_param.tentacle_visu;
  visu_param.best_tentacle_visu = new_visu_param.best_tentacle_visu;
  visu_param.tsamp_visu = new_visu_param.tsamp_visu;
  visu_param.support_vox_visu = new_visu_param.support_vox_visu;
  visu_param.occupancy_pc.header = new_visu_param.occupancy_pc.header;
  visu_param.occupancy_pc.points = new_visu_param.occupancy_pc.points;
  visu_param.occupancy_pc.channels = new_visu_param.occupancy_pc.channels;
  visu_param.path_visu = new_visu_param.path_visu;
  visu_param.command_visu = new_visu_param.command_visu;
  visu_param.debug_visu = new_visu_param.debug_visu;
}

void Tentabot::setDataPath(string data_path)
{
  data_path = data_path;
}

void Tentabot::clearTentacleData()
{
  int tentacle_cnt = off_tuning_param.tentacle_data.size();

  for(int i = 0; i < tentacle_cnt; i++)
  {
    off_tuning_param.tentacle_data[i].clear();
  }

  off_tuning_param.tentacle_data.clear();
}

void Tentabot::clearSupportVoxData()
{
  int svdnum = status_param.support_vox_data.size();

  for(int i = 0; i < svdnum; i++)
  {
    status_param.support_vox_data[i].clear();
  }

  status_param.support_vox_data.clear();
}

void Tentabot::publishTentabot()
{
  if(process_param.visu_flag == true)
  {
    goal_util.publishGoal();

    publishRobot();

    publishTentacleTsampSupportvox();

    publishOccupancy();

    publishPath();

    publishCommand();
  }
}

void Tentabot::publishRobot()
{
  if(process_param.visu_flag == true)
  {
    //visu_param.robot_visu.header.seq++;
    visu_param.robot_visu.header.stamp = ros::Time::now();
    visu_param.robot_visu_pub.publish(visu_param.robot_visu);
  }
}

void Tentabot::publishTentacleTsampSupportvox()
{
  if(process_param.visu_flag == true)
  {
    int tentacle_cnt = off_tuning_param.tentacle_data.size();

    for(int k = 0; k < tentacle_cnt; k++)
    {
      // UPDATE SEQUENCE AND STAMP FOR TENTACLES
      //visu_param.tentacle_visu.markers[k].header.seq++;
      visu_param.tentacle_visu.markers[k].header.stamp = ros::Time::now();

      // UPDATE SEQUENCE AND STAMP FOR SUPPORT CELLS
      //visu_param.support_vox_visu.markers[k].header.seq++;
      visu_param.support_vox_visu.markers[k].header.stamp = ros::Time::now();

      // UPDATE SEQUENCE AND STAMP FOR SAMPLING POINTS ON TENTACLES 
      //visu_param.tsamp_visu.markers[k].header.seq++;
      visu_param.tsamp_visu.markers[k].header.stamp = ros::Time::now();
    }

    for (int i = 0; i < visu_param.best_tentacle_visu.markers.size(); ++i)
    {
      // UPDATE SEQUENCE AND STAMP FOR TENTACLES
      //visu_param.best_tentacle_visu.markers[i].header.seq++;
      visu_param.best_tentacle_visu.markers[i].header.stamp = ros::Time::now();
    }

    visu_param.tentacle_visu_pub.publish(visu_param.tentacle_visu);
    visu_param.best_tentacle_visu_pub.publish(visu_param.best_tentacle_visu);
    visu_param.tsamp_visu_pub.publish(visu_param.tsamp_visu);
    visu_param.support_vox_visu_pub.publish(visu_param.support_vox_visu);
  }
}

void Tentabot::publishOccupancy()
{
  if(process_param.visu_flag == true)
  {
    //visu_param.occupancy_pc.header.seq++;
    visu_param.occupancy_pc.header.stamp = ros::Time::now();
    visu_param.occupancy_pc_pub.publish(visu_param.occupancy_pc);
  }
}

void Tentabot::publishPath()
{
  if(process_param.visu_flag == true)
  {
    visu_param.path_visu.header.frame_id = status_param.map_frame_name;
    //visu_param.path_visu.header.seq++;
    visu_param.path_visu.header.stamp = ros::Time::now();

    visu_param.path_visu_pub.publish(visu_param.path_visu);
  }
}

void Tentabot::publishCommand()
{
  if(process_param.visu_flag == true)
  {
    visu_param.command_visu.header.frame_id = status_param.map_frame_name;
    //visu_param.command_visu.header.seq++;
    visu_param.command_visu.header.stamp = ros::Time::now();

    visu_param.command_visu_pub.publish(visu_param.command_visu);
  }
}

void Tentabot::publishDebugVisu()
{
  if(process_param.visu_flag == true)
  {
    //visu_param.debug_visu.header.seq++;
    visu_param.debug_visu.header.stamp = ros::Time::now();
    visu_param.debug_visu_pub.publish(visu_param.debug_visu);
  }
}

void Tentabot::publishDebugArrayVisu()
{
  if(process_param.visu_flag == true)
  {
    for (int i = 0; i < visu_param.debug_array_visu.markers.size(); ++i)
    {
      //visu_param.debug_array_visu.markers[i].header.seq++;
      //visu_param.debug_array_visu.markers[i].header.stamp = ros::Time(0);
      visu_param.debug_array_visu.markers[i].header.stamp = ros::Time::now();
    }
    visu_param.debug_array_visu_pub.publish(visu_param.debug_array_visu);
  }
}

void Tentabot::transformPoint(string frame_from,
                              string frame_to,
                              geometry_msgs::Point& p_to_msg)
{
  tf::Point p_from_tf;
  geometry_msgs::Point p_from_msg;
  p_from_msg.x = 0;
  p_from_msg.y = 0;
  p_from_msg.z = 0;
  tf::pointMsgToTF(p_from_msg, p_from_tf);
  tf::Stamped<tf::Point> p_from_stamped_tf(p_from_tf, ros::Time(0), frame_from);
  tf::Stamped<tf::Point> p_to_stamped_tf;
  geometry_msgs::PointStamped p_to_stamped_msg;

  try
  {
    tflistener -> transformPoint(frame_to, p_from_stamped_tf, p_to_stamped_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_INFO("Tentabot::transformPoint -> Couldn't get transform!");
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }

  tf::pointStampedTFToMsg(p_to_stamped_tf, p_to_stamped_msg);
  p_to_msg = p_to_stamped_msg.point;
}

void Tentabot::transformPoint(string frame_from, 
                              geometry_msgs::Point& p_from_msg, 
                              string frame_to, 
                              geometry_msgs::Point& p_to_msg)
{
  tf::Point p_from_tf;
  tf::pointMsgToTF(p_from_msg, p_from_tf);
  tf::Stamped<tf::Point> p_from_stamped_tf(p_from_tf, ros::Time(0), frame_from);
  tf::Stamped<tf::Point> p_to_stamped_tf;
  geometry_msgs::PointStamped p_to_stamped_msg;

  try
  {
    tflistener -> transformPoint(frame_to, p_from_stamped_tf, p_to_stamped_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_INFO("Tentabot::transformPoint -> Couldn't get transform!");
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }

  tf::pointStampedTFToMsg(p_to_stamped_tf, p_to_stamped_msg);
  p_to_msg = p_to_stamped_msg.point;
}

void Tentabot::transformPoint(string frame_from, 
                              geometry_msgs::Point32& p_from_msg, 
                              string frame_to, 
                              geometry_msgs::Point32& p_to_msg)
{
  geometry_msgs::Point tmp_p_from;
  tmp_p_from.x = p_from_msg.x;
  tmp_p_from.y = p_from_msg.y;
  tmp_p_from.z = p_from_msg.z;
  geometry_msgs::Point tmp_p_to;

  transformPoint(frame_from, tmp_p_from, frame_to, tmp_p_to);

  p_to_msg.x = tmp_p_to.x;
  p_to_msg.y = tmp_p_to.y;
  p_to_msg.z = tmp_p_to.z;
}

void Tentabot::transformOrientation(string frame_from, 
                                    geometry_msgs::Quaternion& q_from_msg, 
                                    string frame_to, 
                                    geometry_msgs::Quaternion& q_to_msg)
{
  tf::Quaternion q_from_tf;
  tf::quaternionMsgToTF(q_from_msg, q_from_tf);
  tf::Stamped<tf::Quaternion> q_from_stamped_tf(q_from_tf, ros::Time(0), frame_from);
  tf::Stamped<tf::Quaternion> q_to_stamped_tf;
  geometry_msgs::QuaternionStamped q_to_stamped_msg;

  try
  {
    tflistener -> transformQuaternion(frame_to, q_from_stamped_tf, q_to_stamped_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_INFO("Tentabot::transformOrientation -> Couldn't get transform!");
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }

  tf::quaternionStampedTFToMsg(q_to_stamped_tf, q_to_stamped_msg);
  q_to_msg = q_to_stamped_msg.quaternion;
}

void Tentabot::transformOrientation(string frame_from, 
                                    double roll_from, 
                                    double pitch_from, 
                                    double yaw_from, 
                                    string frame_to, 
                                    geometry_msgs::Quaternion& q_to_msg)
{
  tf::Quaternion q_from_tf;
  q_from_tf.setRPY(roll_from, pitch_from, yaw_from);
  tf::Stamped<tf::Quaternion> q_from_stamped_tf(q_from_tf, ros::Time(0), frame_from);
  tf::Stamped<tf::Quaternion> q_to_stamped_tf;
  geometry_msgs::QuaternionStamped q_to_stamped_msg;

  try
  {
    tflistener -> transformQuaternion(frame_to, q_from_stamped_tf, q_to_stamped_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_INFO("Tentabot::transformOrientation -> Couldn't get transform!");
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }

  tf::quaternionStampedTFToMsg(q_to_stamped_tf, q_to_stamped_msg);
  q_to_msg = q_to_stamped_msg.quaternion;
}

void Tentabot::transformPC2(const sensor_msgs::PointCloud2& pc2_from, sensor_msgs::PointCloud2& pc2_to)
{
  string frame_from = pc2_from.header.frame_id;
  string frame_to = pc2_to.header.frame_id;

  tf::StampedTransform transform;
  try
  {
    tflistener -> lookupTransform(frame_to, frame_from, ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) 
  {
    ROS_INFO("Tentabot::transformPC2 -> Couldn't get transform!");
    ROS_ERROR("%s",ex.what());
  }

  pcl_ros::transformPointCloud(frame_to, transform, pc2_from, pc2_to);
}

void Tentabot::mapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  status_param.map_frame_name = msg -> header.frame_id;
  status_param.measured_map_msg = *msg;
}

void Tentabot::robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  //cout << "Tentabot::robotPoseCallback -> Incoming data..." << endl;
  status_param.measured_robot_pose = *msg;
  visu_param.path_visu.points.push_back(status_param.measured_robot_pose.position);

  // PUBLISH THE ROBOT, TENTACLES, SAMPLING POINTS AND SUPPORT VOXELS
  publishTentabot();

  // NUA TODO: CHECK CONSTRAINTS SOMEWHERE ELSE BASED ON THE GIVEN TASK
  /*
  if(status_param.measured_robot_pose.position.z < 0.1)
  {
    cout << "Did I fall?" << endl;
    status_param.nav_result = -1;
  }
  */
}

void Tentabot::odometryCallback(const nav_msgs::Odometry& msg)
{
  //cout << "Tentabot::odometryCallback -> Incoming data..." << endl;
  status_param.measured_robot_pose = msg.pose.pose;
  visu_param.path_visu.points.push_back(status_param.measured_robot_pose.position);

  // PUBLISH THE ROBOT, TENTACLES, SAMPLING POINTS AND SUPPORT VOXELS
  publishTentabot();
}

void Tentabot::mainCallback(const ros::TimerEvent& e) 
{
  if(status_param.navexit_flag == false)
  {
    // UPDATE PLANNING STATES
    auto t0 = std::chrono::high_resolution_clock::now();
    update_planning_states();

    // UPDATE EGO GRID DATA
    auto t1 = std::chrono::high_resolution_clock::now();
    update_ego_grid_data();

    // UPDATE HEURISTIC FUNCTIONS
    auto t2 = std::chrono::high_resolution_clock::now();
    update_heuristic_values();

    // SELECT THE BEST TENTACLE
    auto t3 = std::chrono::high_resolution_clock::now();
    select_best_tentacle();

    // MOVE THE TENTABOT
    auto t4 = std::chrono::high_resolution_clock::now();
    //debug_rotate();

    if (robot_param.robot_pose_control_msg != "")
    {
      send_motion_command_by_pose_control();
    }
    else if(robot_param.robot_velo_control_msg != "")
    {
      send_motion_command_by_velocity_control();
    }
    auto t5 = std::chrono::high_resolution_clock::now();

    if (data_path != "")
    {
      mid_data_stream <<  std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count() << "," <<
                          std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << "," <<
                          std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << "," << 
                          std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << "," << 
                          std::chrono::duration_cast<std::chrono::nanoseconds>(t5-t4).count() << endl;
    }
  }
  else
  {
    if (data_path != "")
    {
      // WRITE RESULTS BENCHMARKS
      string post_data_filename = data_path + data_tag + "_result_" + robot_param.world_name + ".csv";
      post_data_stream.open(post_data_filename);
      post_data_stream << "nav_result[1:success/0:time_limit/-1:crash],nav_duration[s],nav_length[m]\n";
      post_data_stream << status_param.nav_result << ",";
      post_data_stream << status_param.nav_duration << ",";
      post_data_stream << status_param.nav_length << "\n";
    }

    cout << "nav_result: " << status_param.nav_result << endl;
    cout << "nav_duration: " << status_param.nav_duration << endl;
    cout << "nav_length: " << status_param.nav_length << endl;

    ros::shutdown();
  }
}

bool Tentabot::rl_step(tentabot::rl_step::Request &req, tentabot::rl_step::Response &res) 
{
  // UPDATE PLANNING STATES
  update_planning_states();

  // UPDATE EGO GRID DATA
  update_ego_grid_data();

  // UPDATE HEURISTIC FUNCTIONS
  update_heuristic_values();

  // UPDATE RL OBSERVATION
  //res.navigability_set = heuristic_param.navigability_set;
  res.occupancy_set = heuristic_param.occupancy_set;
  res.clearance_set = heuristic_param.clearance_set;
  res.clutterness_set = heuristic_param.clutterness_set;
  res.closeness_set = heuristic_param.closeness_set;
  //res.smoothness_set = heuristic_param.smoothness_set;

  return true;
}

bool Tentabot::update_goal(tentabot::update_goal::Request &req, tentabot::update_goal::Response &res)
{
  goal_util.setGoalPoint(req.goal);

  //cout << "Tentabot::update_goal -> Goal updated to x: " << goal_util.getActiveGoal().position.x << ", y: " << goal_util.getActiveGoal().position.y << endl;

  res.success = true;
  return true;
}

int Tentabot::toIndex(double pos, int grid_vnum)
{
  return grid_vnum + floor(pos / off_tuning_param.egrid_vdim);
}

int Tentabot::toLinIndex(tf::Vector3 po)
{
  int grid_vnumx = status_param.egrid_vnum_x_max + status_param.egrid_vnum_x_min;
  int grid_vnumy = status_param.egrid_vnum_y_max + status_param.egrid_vnum_y_min;

  int ind_x = toIndex(po.x(), status_param.egrid_vnum_x_min);
  int ind_y = toIndex(po.y(), status_param.egrid_vnum_y_min);
  int ind_z = toIndex(po.z(), status_param.egrid_vnum_z_min);

  return (ind_x + ind_y * grid_vnumx + ind_z * grid_vnumx * grid_vnumy);
}

int Tentabot::toLinIndex(geometry_msgs::Point po)
{
  int grid_vnumx = status_param.egrid_vnum_x_max + status_param.egrid_vnum_x_min;
  int grid_vnumy = status_param.egrid_vnum_y_max + status_param.egrid_vnum_y_min;

  int ind_x = toIndex(po.x, status_param.egrid_vnum_x_min);
  int ind_y = toIndex(po.y, status_param.egrid_vnum_y_min);
  int ind_z = toIndex(po.z, status_param.egrid_vnum_z_min);

  return (ind_x + ind_y * grid_vnumx + ind_z * grid_vnumx * grid_vnumy);
}

int Tentabot::toLinIndex(geometry_msgs::Point32 po)
{
  int grid_vnumx = status_param.egrid_vnum_x_max + status_param.egrid_vnum_x_min;
  int grid_vnumy = status_param.egrid_vnum_y_max + status_param.egrid_vnum_y_min;

  int ind_x = toIndex(po.x, status_param.egrid_vnum_x_min);
  int ind_y = toIndex(po.y, status_param.egrid_vnum_y_min);
  int ind_z = toIndex(po.z, status_param.egrid_vnum_z_min);

  return (ind_x + ind_y * grid_vnumx + ind_z * grid_vnumx * grid_vnumy);
}

void Tentabot::toPoint(int ind, geometry_msgs::Point& po)
{
  double grid_vdim = off_tuning_param.egrid_vdim;
  int grid_vnumx = status_param.egrid_vnum_x_max + status_param.egrid_vnum_x_min;
  int grid_vnumy = status_param.egrid_vnum_y_max + status_param.egrid_vnum_y_min;

  po.x = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) % grid_vnumx  - status_param.egrid_vnum_x_min) + 0.5 * grid_vdim;
  po.y = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) / grid_vnumx  - status_param.egrid_vnum_y_min) + 0.5 * grid_vdim;
  po.z = grid_vdim * (ind / (grid_vnumx * grid_vnumy) - status_param.egrid_vnum_z_min) + 0.5 * grid_vdim;
}

void Tentabot::toPoint(int ind, geometry_msgs::Point32& po)
{
  double grid_vdim = off_tuning_param.egrid_vdim;
  int grid_vnumx = status_param.egrid_vnum_x_max + status_param.egrid_vnum_x_min;
  int grid_vnumy = status_param.egrid_vnum_y_max + status_param.egrid_vnum_y_min;

  po.x = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) % grid_vnumx  - status_param.egrid_vnum_x_min) + 0.5 * grid_vdim;
  po.y = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) / grid_vnumx  - status_param.egrid_vnum_y_min) + 0.5 * grid_vdim;
  po.z = grid_vdim * (ind / (grid_vnumx * grid_vnumy) - status_param.egrid_vnum_z_min) + 0.5 * grid_vdim;
}

void Tentabot::initialize(NodeHandle& nh)
{
  data_tag = createFileName();

  sort_tentacles();

  calculate_tentacle_length_and_bbx_data();

  status_param.egrid_vnum_x_max = abs(status_param.tentacle_bbx_max.x + off_tuning_param.sdist_x_max) / off_tuning_param.egrid_vdim + 1;
  status_param.egrid_vnum_x_min = abs(status_param.tentacle_bbx_min.x + off_tuning_param.sdist_x_min) / off_tuning_param.egrid_vdim + 1;
  status_param.egrid_vnum_y_max = abs(status_param.tentacle_bbx_max.y + off_tuning_param.sdist_y_max) / off_tuning_param.egrid_vdim + 1;
  status_param.egrid_vnum_y_min = abs(status_param.tentacle_bbx_min.y + off_tuning_param.sdist_y_min) / off_tuning_param.egrid_vdim + 1;
  status_param.egrid_vnum_z_max = abs(status_param.tentacle_bbx_max.z + off_tuning_param.sdist_z_max) / off_tuning_param.egrid_vdim + 1;
  status_param.egrid_vnum_z_min = abs(status_param.tentacle_bbx_min.z + off_tuning_param.sdist_z_min) / off_tuning_param.egrid_vdim + 1;

  status_param.egrid_dist_x_max = off_tuning_param.egrid_vdim * status_param.egrid_vnum_x_max;
  status_param.egrid_dist_x_min = -1 * off_tuning_param.egrid_vdim * status_param.egrid_vnum_x_min;
  status_param.egrid_dist_y_max = off_tuning_param.egrid_vdim * status_param.egrid_vnum_y_max;
  status_param.egrid_dist_y_min = -1 * off_tuning_param.egrid_vdim * status_param.egrid_vnum_y_min;
  status_param.egrid_dist_z_max = off_tuning_param.egrid_vdim * status_param.egrid_vnum_z_max;
  status_param.egrid_dist_z_min = -1 * off_tuning_param.egrid_vdim * status_param.egrid_vnum_z_min;

  ros::Time t1 = ros::Time::now();
  construct_ego_grid_data();
  ros::Time t2 = ros::Time::now();
  construct_priority_support_voxel_data();
  ros::Time t3 = ros::Time::now();
  construct_sample_weight_data();
  construct_tentacle_weight_data();

  status_param.map_frame_name = "";
  status_param.measured_robot_pose = robot_param.init_robot_pose;
  status_param.robot_pose = status_param.measured_robot_pose;
  status_param.prev_robot_pose = status_param.robot_pose;
  status_param.tcrash_bin.resize(off_tuning_param.tentacle_data.size());
  status_param.navigability_flag = false;
  status_param.best_tentacle = -1;
  status_param.ex_best_tentacle = -1;
  status_param.ex_best_sample = -1;
  status_param.nav_result = 8;
  status_param.nav_length = 0;
  status_param.nav_duration = 0;
  status_param.desired_lat_speed = 1.0;
  status_param.lat_speed_weight = 1.0;
  status_param.counter = 0;
  status_param.navexit_flag = false;

  if(robot_param.robot_pose_control_msg != "")
  {
    status_param.command_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(robot_param.robot_pose_control_msg, 100);
  }
  else if(robot_param.robot_velo_control_msg != "")
  {
    status_param.command_velo_pub = nh.advertise<geometry_msgs::Twist>(robot_param.robot_velo_control_msg, 100);
  }

  if(process_param.visu_flag == true)
  {
    fillRobotVisu();
    fillTentacleTsampSupportvoxVisu();
    visu_param.occupancy_pc.header.frame_id = robot_param.robot_frame_name;
    fillPathVisu();
    fillCommandVisu();

    visu_param.robot_visu_pub = nh.advertise<visualization_msgs::Marker>("tentabot", 100);
    visu_param.tentacle_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("tentacles", 100);
    visu_param.best_tentacle_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("best_tentacle", 100);
    visu_param.tsamp_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("sampling_points", 100);
    visu_param.support_vox_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("support_voxels", 100);
    visu_param.occupancy_pc_pub = nh.advertise<sensor_msgs::PointCloud>("ego_occupancy_pc", 100);
    visu_param.path_visu_pub = nh.advertise<visualization_msgs::Marker>("path2glory", 100);
    visu_param.command_visu_pub = nh.advertise<visualization_msgs::Marker>("commandante", 100);
    visu_param.debug_visu_pub = nh.advertise<visualization_msgs::Marker>("debug", 100);
    visu_param.debug_array_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("debug_array", 100);
  }

  // WRITE PARAMETERS
  if (data_path != "")
  {
    string input_data_filename = data_path + data_tag + "_param_" + robot_param.world_name + ".csv";
    input_data_stream.open(input_data_filename);
    input_data_stream << "nav_dt," << process_param.nav_dt << "\n";
    input_data_stream << "sensor_frame_name," << robot_param.sensor_frame_name << "\n";
    input_data_stream << "dummy_max_lat_velo," << robot_param.dummy_max_lat_velo << "\n";
    input_data_stream << "dummy_max_lat_acc," << robot_param.dummy_max_lat_acc << "\n";
    input_data_stream << "dummy_max_yaw_velo," << robot_param.dummy_max_yaw_velo << "\n";
    input_data_stream << "dummy_max_yaw_acc," << robot_param.dummy_max_yaw_acc << "\n";
    input_data_stream << "robot_pose_control_msg," << robot_param.robot_pose_control_msg << "\n";
    input_data_stream << "robot_velo_control_msg," << robot_param.robot_velo_control_msg << "\n";
    input_data_stream << "tentacle_data_path," << off_tuning_param.tentacle_data_path << "\n";
    input_data_stream << "max_occupancy_belief_value," << off_tuning_param.max_occupancy_belief_value << "\n";
    input_data_stream << "pdist_x_max," << off_tuning_param.pdist_x_max << "\n";
    input_data_stream << "pdist_x_min," << off_tuning_param.pdist_x_min << "\n";
    input_data_stream << "pdist_y_max," << off_tuning_param.pdist_y_max << "\n";
    input_data_stream << "pdist_y_min," << off_tuning_param.pdist_y_min << "\n";
    input_data_stream << "pdist_z_max," << off_tuning_param.pdist_z_max << "\n";
    input_data_stream << "pdist_z_min," << off_tuning_param.pdist_z_min << "\n";
    input_data_stream << "sdist_x_max," << off_tuning_param.sdist_x_max << "\n";
    input_data_stream << "sdist_x_min," << off_tuning_param.sdist_x_min << "\n";
    input_data_stream << "sdist_y_max," << off_tuning_param.sdist_y_max << "\n";
    input_data_stream << "sdist_y_min," << off_tuning_param.sdist_y_min << "\n";
    input_data_stream << "sdist_z_max," << off_tuning_param.sdist_z_max << "\n";
    input_data_stream << "sdist_z_min," << off_tuning_param.sdist_z_min << "\n";
    input_data_stream << "sweight_max," << off_tuning_param.sweight_max << "\n";
    input_data_stream << "sweight_scale," << off_tuning_param.sweight_scale << "\n";
    input_data_stream << "egrid_vdim," << off_tuning_param.egrid_vdim << "\n";
    input_data_stream << "egrid_vnum_x_max," << status_param.egrid_vnum_x_max << "\n";
    input_data_stream << "egrid_vnum_x_min," << status_param.egrid_vnum_x_min << "\n";
    input_data_stream << "egrid_vnum_y_max," << status_param.egrid_vnum_y_max << "\n";
    input_data_stream << "egrid_vnum_y_min," << status_param.egrid_vnum_y_min << "\n";
    input_data_stream << "egrid_vnum_z_max," << status_param.egrid_vnum_z_max << "\n";
    input_data_stream << "egrid_vnum_z_min," << status_param.egrid_vnum_z_min << "\n";
    input_data_stream << "tbin_obs_cnt_threshold," << on_tuning_param.tbin_obs_cnt_threshold << "\n";
    input_data_stream << "crash_dist_scale," << on_tuning_param.crash_dist_scale << "\n";
    input_data_stream << "clear_scale," << on_tuning_param.clear_scale << "\n";
    input_data_stream << "clutter_scale," << on_tuning_param.clutter_scale << "\n";
    input_data_stream << "close_scale," << on_tuning_param.close_scale << "\n";
    input_data_stream << "smooth_scale," << on_tuning_param.smooth_scale << "\n";
    input_data_stream.close();

    // WRITE PRE BENCHMARKS
    string pre_data_filename = data_path + data_tag + "_pre_" + robot_param.world_name + ".csv";
    pre_data_stream.open(pre_data_filename);
    pre_data_stream << "grid[s],voxel[s]\n";
    pre_data_stream << (t2-t1).toSec() << ",";
    pre_data_stream << (t3-t2).toSec() << "\n";
    pre_data_stream.close();

    // WRITE PROCESS BENCHMARKS
    string mid_data_filename = data_path + data_tag + "_process_" + robot_param.world_name + ".csv";
    mid_data_stream.open(mid_data_filename);
    mid_data_stream << "update_planning_states[ns],update_ego_grid_data[ns],update_heuristic_values[ns],select_best_tentacle[ns],send_motion_command[ns]\n";
  }
}

vector<Tentabot::OccupancyVoxel> Tentabot::extract_priority_support_voxels_by_bbx(vector<geometry_msgs::Point>& traj)
{
  vector<OccupancyVoxel> svg;

  // SET THE CORNER POINTS IN BOUNDING BOX IN WORLD COORDINATE
  vector<tf::Vector3> bbx_pdist;
  bbx_pdist.push_back(tf::Vector3(off_tuning_param.pdist_x_max, off_tuning_param.pdist_y_max, off_tuning_param.pdist_z_max));
  bbx_pdist.push_back(tf::Vector3(off_tuning_param.pdist_x_max, off_tuning_param.pdist_y_min, off_tuning_param.pdist_z_max));
  bbx_pdist.push_back(tf::Vector3(off_tuning_param.pdist_x_min, off_tuning_param.pdist_y_max, off_tuning_param.pdist_z_max));
  bbx_pdist.push_back(tf::Vector3(off_tuning_param.pdist_x_min, off_tuning_param.pdist_y_min, off_tuning_param.pdist_z_max));
  bbx_pdist.push_back(tf::Vector3(off_tuning_param.pdist_x_max, off_tuning_param.pdist_y_max, off_tuning_param.pdist_z_min));
  bbx_pdist.push_back(tf::Vector3(off_tuning_param.pdist_x_max, off_tuning_param.pdist_y_min, off_tuning_param.pdist_z_min));
  bbx_pdist.push_back(tf::Vector3(off_tuning_param.pdist_x_min, off_tuning_param.pdist_y_max, off_tuning_param.pdist_z_min));
  bbx_pdist.push_back(tf::Vector3(off_tuning_param.pdist_x_min, off_tuning_param.pdist_y_min, off_tuning_param.pdist_z_min));

  vector<tf::Vector3> bbx_sdist;
  bbx_sdist.push_back(tf::Vector3(off_tuning_param.sdist_x_max, off_tuning_param.sdist_y_max, off_tuning_param.sdist_z_max));
  bbx_sdist.push_back(tf::Vector3(off_tuning_param.sdist_x_max, off_tuning_param.sdist_y_min, off_tuning_param.sdist_z_max));
  bbx_sdist.push_back(tf::Vector3(off_tuning_param.sdist_x_min, off_tuning_param.sdist_y_max, off_tuning_param.sdist_z_max));
  bbx_sdist.push_back(tf::Vector3(off_tuning_param.sdist_x_min, off_tuning_param.sdist_y_min, off_tuning_param.sdist_z_max));
  bbx_sdist.push_back(tf::Vector3(off_tuning_param.sdist_x_max, off_tuning_param.sdist_y_max, off_tuning_param.sdist_z_min));
  bbx_sdist.push_back(tf::Vector3(off_tuning_param.sdist_x_max, off_tuning_param.sdist_y_min, off_tuning_param.sdist_z_min));
  bbx_sdist.push_back(tf::Vector3(off_tuning_param.sdist_x_min, off_tuning_param.sdist_y_max, off_tuning_param.sdist_z_min));
  bbx_sdist.push_back(tf::Vector3(off_tuning_param.sdist_x_min, off_tuning_param.sdist_y_min, off_tuning_param.sdist_z_min));

  vector<tf::Transform> Tinv_vec;

  vector<double> bbx_pdist_tf_x;
  vector<double> bbx_pdist_tf_y;
  vector<double> bbx_pdist_tf_z;

  vector<double> bbx_sdist_tf_x;
  vector<double> bbx_sdist_tf_y;
  vector<double> bbx_sdist_tf_z;

  tf::Vector3 tp_pdist, tp_sdist;
  double yaw = 0;
  double pitch = 0;

  for (int i = 0; i < traj.size(); ++i)
  {
    //NUA TODO: FIX THE ATAN2 PROBLEM BIGGER THAN PI
    /*
    if (i < traj.size()-1)
    {
      yaw = atan2(traj[i+1].y - traj[i].y, traj[i+1].x - traj[i].x);
      pitch = atan2(traj[i+1].z - traj[i].z, traj[i+1].x - traj[i].x);
    }
    */

    tf::Transform T;
    tf::Matrix3x3 rot;
    rot.setRPY(0, pitch, yaw);
    tf::Vector3 trans(traj[i].x, traj[i].y, traj[i].z);
    T.setOrigin(trans);
    T.setBasis(rot);
    Tinv_vec.push_back(T.inverse());

    for (int j = 0; j < bbx_pdist.size(); ++j)
    {
      tp_pdist = T * bbx_pdist[j];
      bbx_pdist_tf_x.push_back(tp_pdist.x());
      bbx_pdist_tf_y.push_back(tp_pdist.y());
      bbx_pdist_tf_z.push_back(tp_pdist.z());
    }

    for (int k = 0; k < bbx_sdist.size(); ++k)
    {
      tp_sdist = T * bbx_sdist[k];
      bbx_sdist_tf_x.push_back(tp_sdist.x());
      bbx_sdist_tf_y.push_back(tp_sdist.y());
      bbx_sdist_tf_z.push_back(tp_sdist.z());
    }
  }

  // FIND BOUNDARIES IN X
  double egrid_vnumx = status_param.egrid_vnum_x_max + status_param.egrid_vnum_x_min;
  double min_pdist_x = *min_element(bbx_pdist_tf_x.begin(), bbx_pdist_tf_x.end());
  double max_pdist_x = *max_element(bbx_pdist_tf_x.begin(), bbx_pdist_tf_x.end());
  double min_sdist_x = *min_element(bbx_sdist_tf_x.begin(), bbx_sdist_tf_x.end());
  double max_sdist_x = *max_element(bbx_sdist_tf_x.begin(), bbx_sdist_tf_x.end());
  int min_vox_ind_x = toIndex(min_sdist_x, status_param.egrid_vnum_x_min);
  int max_vox_ind_x = toIndex(max_sdist_x, status_param.egrid_vnum_x_min);

  if(min_vox_ind_x < 0)
  {
    min_vox_ind_x = 0;
  }

  if(max_vox_ind_x >= status_param.egrid_vnum_x_max + status_param.egrid_vnum_x_min)
  {
    max_vox_ind_x = status_param.egrid_vnum_x_max + status_param.egrid_vnum_x_min - 1;
  }

  // FIND BOUNDARIES IN Y
  double egrid_vnumy = status_param.egrid_vnum_y_max + status_param.egrid_vnum_y_min;
  double min_pdist_y = *min_element(bbx_pdist_tf_y.begin(), bbx_pdist_tf_y.end());
  double max_pdist_y = *max_element(bbx_pdist_tf_y.begin(), bbx_pdist_tf_y.end());
  double min_sdist_y = *min_element(bbx_sdist_tf_y.begin(), bbx_sdist_tf_y.end());
  double max_sdist_y = *max_element(bbx_sdist_tf_y.begin(), bbx_sdist_tf_y.end());
  int min_vox_ind_y = toIndex(min_sdist_y, status_param.egrid_vnum_y_min);
  int max_vox_ind_y = toIndex(max_sdist_y, status_param.egrid_vnum_y_min);

  if(min_vox_ind_y < 0)
  {
    min_vox_ind_y = 0;
  }

  if(max_vox_ind_y >= egrid_vnumy)
  {
    max_vox_ind_y = egrid_vnumy - 1;
  }

  // FIND BOUNDARIES IN Z
  double egrid_vnumz = status_param.egrid_vnum_z_max + status_param.egrid_vnum_z_min;
  double min_pdist_z = *min_element(bbx_pdist_tf_z.begin(), bbx_pdist_tf_z.end());
  double max_pdist_z = *max_element(bbx_pdist_tf_z.begin(), bbx_pdist_tf_z.end());
  double min_sdist_z = *min_element(bbx_sdist_tf_z.begin(), bbx_sdist_tf_z.end());
  double max_sdist_z = *max_element(bbx_sdist_tf_z.begin(), bbx_sdist_tf_z.end());
  int min_vox_ind_z = toIndex(min_sdist_z, status_param.egrid_vnum_z_min);
  int max_vox_ind_z = toIndex(max_sdist_z, status_param.egrid_vnum_z_min);

  if(min_vox_ind_z < 0)
  {
    min_vox_ind_z = 0;
  }

  if(max_vox_ind_z >= egrid_vnumz)
  {
    max_vox_ind_z = egrid_vnumz - 1;
  }

  // EXTRACT PRIORITY/SUPPORT VOXELS
  int gind;
  //vector <double> cd;   // [0]: closest distance, [1]: closest trajectory sample index
  double s2p, p2cell, p2cell_min;
  tf::Vector3 cc_tf;
  double max_pdist = max({abs(min_pdist_x), abs(max_pdist_x), abs(min_pdist_y), abs(max_pdist_y), abs(min_pdist_z), abs(max_pdist_z)});
  geometry_msgs::Point cell_center, pmin;
  vector<double> px_vec = {min_pdist_x, max_pdist_x};
  vector<double> py_vec = {min_pdist_y, max_pdist_y};
  vector<double> pz_vec = {min_pdist_z, max_pdist_z};
  vector<geometry_msgs::Point> p_vec;

  for (int k = 0; k < pz_vec.size(); ++k)
  {
    for (int j = 0; j < py_vec.size(); ++j)
    {
      for (int i = 0; i < px_vec.size(); ++i)
      {
        geometry_msgs::Point p;
        p.x = px_vec[i];
        p.y = py_vec[j];
        p.z = pz_vec[k];
        p_vec.push_back(p);
      }
    }
  }

  for(int i = min_vox_ind_x; i <= max_vox_ind_x; i++)
  {
    for(int j = min_vox_ind_y; j <= max_vox_ind_y; j++)
    {
      for(int r = min_vox_ind_z; r <= max_vox_ind_z; r++)
      {
        gind = i + j * egrid_vnumx + r * egrid_vnumx * egrid_vnumy;
        cell_center = status_param.ego_grid_data.ovox_pos[gind];

        for (int s = 0; s < traj.size(); ++s)
        {
          p2cell_min = find_Euclidean_distance(p_vec[0], cell_center);
          pmin = p_vec[0];
          for (int k = 1; k < p_vec.size(); ++k)
          {
            p2cell = find_Euclidean_distance(p_vec[k], cell_center);
            if(p2cell < p2cell_min)
            {
              p2cell_min = p2cell;
              pmin = p_vec[k];
            }
          }
          s2p = find_norm(pmin);

          // CHECK IF IT IS SUPPORT VOXEL
          cc_tf = Tinv_vec[s] * tf::Vector3(cell_center.x, cell_center.y, cell_center.z);

          if( isInBBx(cc_tf,  off_tuning_param.sdist_x_min, off_tuning_param.sdist_x_max, 
                              off_tuning_param.sdist_y_min, off_tuning_param.sdist_y_max, 
                              off_tuning_param.sdist_z_min, off_tuning_param.sdist_z_max) )
          {
            OccupancyVoxel sv;
            sv.index = gind;
            sv.histbin = s;

            // CHECK IF IT IS PRIORITY VOXEL
            if( isInBBx(cc_tf,  off_tuning_param.pdist_x_min, off_tuning_param.pdist_x_max, 
                                off_tuning_param.pdist_y_min, off_tuning_param.pdist_y_max, 
                                off_tuning_param.pdist_z_min, off_tuning_param.pdist_z_max) )
            {
              sv.weight = off_tuning_param.sweight_max;
              sv.flag = true;
            }
            else
            {
              sv.weight = off_tuning_param.sweight_max * s2p / (s2p + p2cell_min);
              //sv.weight = off_tuning_param.sweight_max / (off_tuning_param.sweight_scale * (dist - max_pdist));
              sv.flag = false;
            }
            svg.push_back(sv);
          }
        }
      }
    }
  }
  return svg;
}

vector<Tentabot::OccupancyVoxel> Tentabot::extract_priority_support_voxels_by_radius(vector<geometry_msgs::Point>& traj)
{
  vector<OccupancyVoxel> svg;

  double pdist = max( { off_tuning_param.pdist_x_max, off_tuning_param.pdist_x_min, 
                        off_tuning_param.pdist_y_max, off_tuning_param.pdist_y_min, 
                        off_tuning_param.pdist_z_max, off_tuning_param.pdist_z_min} );
  double sdist = max( { off_tuning_param.sdist_x_max, off_tuning_param.sdist_x_min, 
                        off_tuning_param.sdist_y_max, off_tuning_param.sdist_y_min, 
                        off_tuning_param.sdist_z_max, off_tuning_param.sdist_z_min} );

  double egrid_vnumx = status_param.egrid_vnum_x_max + status_param.egrid_vnum_x_min;
  double egrid_vnumy = status_param.egrid_vnum_y_max + status_param.egrid_vnum_y_min;
  double egrid_vnumz = status_param.egrid_vnum_z_max + status_param.egrid_vnum_z_min;
  vector<double> traj_limits = find_limits(traj);
  int min_vox_ind_x = toIndex(traj_limits[0] - sdist, status_param.egrid_vnum_x_min);
  int max_vox_ind_x = toIndex(traj_limits[1] + sdist, status_param.egrid_vnum_x_min);

  if(min_vox_ind_x < 0)
  {
    min_vox_ind_x = 0;
  }

  if(max_vox_ind_x >= egrid_vnumx)
  {
    max_vox_ind_x = egrid_vnumx - 1;
  }

  int min_vox_ind_y = toIndex(traj_limits[2] - sdist, status_param.egrid_vnum_y_min);
  int max_vox_ind_y = toIndex(traj_limits[3] + sdist, status_param.egrid_vnum_y_min);

  if(min_vox_ind_y < 0)
  {
    min_vox_ind_y = 0;
  }

  if(max_vox_ind_y >= egrid_vnumy)
  {
    max_vox_ind_y = egrid_vnumy - 1;
  }

  int min_vox_ind_z = toIndex(traj_limits[4] - sdist, status_param.egrid_vnum_z_min);
  int max_vox_ind_z = toIndex(traj_limits[5] + sdist, status_param.egrid_vnum_z_min);

  if(min_vox_ind_z < 0)
  {
    min_vox_ind_z = 0;
  }

  if(max_vox_ind_z >= egrid_vnumz)
  {
    max_vox_ind_z = egrid_vnumz - 1;
  }

  geometry_msgs::Point cell_center;        
  int gind;
  vector <double> cd;           // [0]: closest distance, [1]: closest tentacle point index

  for(int i = min_vox_ind_x; i <= max_vox_ind_x; i++)
  {
    for(int j = min_vox_ind_y; j <= max_vox_ind_y; j++)
    {
      for(int r = min_vox_ind_z; r <= max_vox_ind_z; r++)
      {
        OccupancyVoxel sv;
        gind = i + j * egrid_vnumx + r * egrid_vnumx * egrid_vnumy;
        cell_center = status_param.ego_grid_data.ovox_pos[gind];
        cd = find_closest_dist(traj, cell_center);

        if(cd[0] <= sdist)
        {
          sv.index = gind;     
          sv.histbin = (int) cd[1];
          if(cd[0] <= pdist)
          {
            sv.weight = off_tuning_param.sweight_max;
            sv.flag = true;
          }
          else
          {
            sv.weight = off_tuning_param.sweight_max * pdist / cd[0];
            //sv.weight = off_tuning_param.sweight_max / (off_tuning_param.sweight_scale * (cd[0] - pdist));
            sv.flag = false;
          }
          svg.push_back(sv);
        }
      }
    }
  }
  return svg;
}

void Tentabot::calculate_tentacle_length_and_bbx_data()
{
  status_param.tentacle_length_data.clear();
  double tlen;
  status_param.tentacle_bbx_min = off_tuning_param.tentacle_data[0][0];
  status_param.tentacle_bbx_max = off_tuning_param.tentacle_data[0][0];

  for (int i = 0; i < off_tuning_param.tentacle_data.size(); ++i)
  {
    tlen = 0;
    for (int j = 0; j < off_tuning_param.tentacle_data[i].size()-1; ++j)
    {
      tlen += find_Euclidean_distance(off_tuning_param.tentacle_data[i][j], off_tuning_param.tentacle_data[i][j+1]);

      if (off_tuning_param.tentacle_data[i][j].x < status_param.tentacle_bbx_min.x)
      {
        status_param.tentacle_bbx_min.x = off_tuning_param.tentacle_data[i][j].x;
      }

      if (off_tuning_param.tentacle_data[i][j].y < status_param.tentacle_bbx_min.y)
      {
        status_param.tentacle_bbx_min.y = off_tuning_param.tentacle_data[i][j].y;
      }

      if (off_tuning_param.tentacle_data[i][j].z < status_param.tentacle_bbx_min.z)
      {
        status_param.tentacle_bbx_min.z = off_tuning_param.tentacle_data[i][j].z;
      }

      if (off_tuning_param.tentacle_data[i][j].x > status_param.tentacle_bbx_max.x)
      {
        status_param.tentacle_bbx_max.x = off_tuning_param.tentacle_data[i][j].x;
      }

      if (off_tuning_param.tentacle_data[i][j].y > status_param.tentacle_bbx_max.y)
      {
        status_param.tentacle_bbx_max.y = off_tuning_param.tentacle_data[i][j].y;
      }

      if (off_tuning_param.tentacle_data[i][j].z > status_param.tentacle_bbx_max.z)
      {
        status_param.tentacle_bbx_max.z = off_tuning_param.tentacle_data[i][j].z;
      }
    }
    status_param.tentacle_length_data.push_back(tlen);

    if (off_tuning_param.tentacle_data[i].back().x < status_param.tentacle_bbx_min.x)
    {
      status_param.tentacle_bbx_min.x = off_tuning_param.tentacle_data[i].back().x;
    }

    if (off_tuning_param.tentacle_data[i].back().y < status_param.tentacle_bbx_min.y)
    {
      status_param.tentacle_bbx_min.y = off_tuning_param.tentacle_data[i].back().y;
    }

    if (off_tuning_param.tentacle_data[i].back().z < status_param.tentacle_bbx_min.z)
    {
      status_param.tentacle_bbx_min.z = off_tuning_param.tentacle_data[i].back().z;
    }

    if (off_tuning_param.tentacle_data[i].back().x > status_param.tentacle_bbx_max.x)
    {
      status_param.tentacle_bbx_max.x = off_tuning_param.tentacle_data[i].back().x;
    }

    if (off_tuning_param.tentacle_data[i].back().y > status_param.tentacle_bbx_max.y)
    {
      status_param.tentacle_bbx_max.y = off_tuning_param.tentacle_data[i].back().y;
    }

    if (off_tuning_param.tentacle_data[i].back().z > status_param.tentacle_bbx_max.z)
    {
      status_param.tentacle_bbx_max.z = off_tuning_param.tentacle_data[i].back().z;
    }
  }
}

void Tentabot::construct_ego_grid_data()
{
  double egrid_vnumx = status_param.egrid_vnum_x_max + status_param.egrid_vnum_x_min;
  double egrid_vnumy = status_param.egrid_vnum_y_max + status_param.egrid_vnum_y_min;
  double egrid_vnumz = status_param.egrid_vnum_z_max + status_param.egrid_vnum_z_min;
  int total_voxel_cnt = egrid_vnumx * egrid_vnumy * egrid_vnumz;

  status_param.ego_grid_data.ovox_pos.resize(total_voxel_cnt);

  for(int v = 0; v < total_voxel_cnt; v++)
  {
    geometry_msgs::Point po;
    toPoint(v, po);
    status_param.ego_grid_data.ovox_pos[v] = po;
  }

  status_param.ego_grid_data.ovox_value.resize(total_voxel_cnt);
}

void Tentabot::construct_priority_support_voxel_data()
{
  clearSupportVoxData();
  for (int i = 0; i < off_tuning_param.tentacle_data.size(); ++i)
  {
    //status_param.support_vox_data.push_back(extract_priority_support_voxels_by_radius(off_tuning_param.tentacle_data[i]));
    status_param.support_vox_data.push_back(extract_priority_support_voxels_by_bbx(off_tuning_param.tentacle_data[i]));
  }
}

void Tentabot::construct_sample_weight_data()
{
  clear_vector(status_param.sample_weight_data);

  int tentacle_cnt = off_tuning_param.tentacle_data.size();
  int sample_cnt;
  int n = 0;

  for (int k = 0; k < off_tuning_param.tentacle_data.size(); ++k)
  {
    sample_cnt = off_tuning_param.tentacle_data[k].size();
    vector<double> tvw(sample_cnt, 0);
    n = 0;
   
    for (int s = 0; s < status_param.support_vox_data[k].size(); ++s)
    {
      if(status_param.support_vox_data[k][s].histbin == 0)
      {
        n++;
      }
      tvw[status_param.support_vox_data[k][s].histbin] += status_param.support_vox_data[k][s].weight;
    }
    status_param.sample_weight_data.push_back(tvw);
  }

  /*
  cout << "tentabot::construct_sample_weight_data -> n: " << n << endl;
  for (int i = 0; i < status_param.sample_weight_data.size(); ++i)
  {
    cout << i << " -> ";
    print(status_param.sample_weight_data[i]);
  }
  */
}

void Tentabot::construct_tentacle_weight_data()
{
  status_param.tentacle_weight_data.clear();
  
  double total_weight;

  for (int i = 0; i < status_param.sample_weight_data.size(); ++i)
  {
    total_weight = 0;
    for (int j = 0; j < status_param.sample_weight_data[i].size(); ++j)
    {
      total_weight += status_param.sample_weight_data[i][j];
    }

    status_param.tentacle_weight_data.push_back(total_weight);
  }

  //cout << "tentabot::construct_tentacle_weight_data -> " << endl;
  //print(status_param.tentacle_weight_data);
}

double Tentabot::calculate_between_tentacle_closeness(vector<geometry_msgs::Point>& tentacle1, vector<geometry_msgs::Point>& tentacle2)
{
  int t1_sample_cnt = tentacle1.size();
  int t2_sample_cnt = tentacle2.size();
  int t_sample_cnt;
  int t1_delta = 1;
  int t2_delta = 1;
  double total_dist = 0;

  if (t1_sample_cnt < t2_sample_cnt)
  {
    t_sample_cnt = t1_sample_cnt;
    t2_delta = t2_sample_cnt / t1_sample_cnt;
  }
  else
  {
    t_sample_cnt = t2_sample_cnt;
    t1_delta = t1_sample_cnt / t2_sample_cnt;
  }

  for (int i = 0; i < t_sample_cnt; ++i)
  {
    total_dist += find_Euclidean_distance(tentacle1[t1_delta*i], tentacle2[t2_delta*i]);
  }

  return total_dist;
}

void Tentabot::sort_tentacles(string sort_type)
{
  vector<vector<geometry_msgs::Point>> tentacle_data_original = off_tuning_param.tentacle_data;
  vector<vector<double>> velocity_control_data_original = off_tuning_param.velocity_control_data;
  
  int tentacle_cnt = off_tuning_param.tentacle_data.size();
  int sample_cnt;

  double t_avg_x = 0;
  double t_avg_y = 0;
  double t_avg_z = 0;

  vector<double> tentacle_avg;
  vector<double> tentacle_avg_x;
  vector<double> tentacle_avg_y;
  vector<double> tentacle_avg_z;
  vector<int> ordered_indices;
  int min_index;

  for (int i = 0; i < tentacle_cnt; ++i)
  {
    sample_cnt = off_tuning_param.tentacle_data[i].size();

    t_avg_x = 0;
    t_avg_y = 0;
    t_avg_z = 0;

    for (int j = 0; j < sample_cnt; ++j)
    {
      t_avg_x += off_tuning_param.tentacle_data[i][j].x;
      t_avg_y += off_tuning_param.tentacle_data[i][j].y;
      t_avg_z += off_tuning_param.tentacle_data[i][j].z;
    }

    t_avg_x /= sample_cnt;
    t_avg_y /= sample_cnt;
    t_avg_z /= sample_cnt;

    tentacle_avg_x.push_back(t_avg_x);
    tentacle_avg_y.push_back(t_avg_y);
    tentacle_avg_z.push_back(t_avg_z);
  }

  if (sort_type == "x")
  {
    cout << "Sorting tentacles by x-axis..." << endl;
    tentacle_avg = tentacle_avg_x;
  }
  else if (sort_type == "y")
  {
    cout << "Sorting tentacles by y-axis..." << endl;
    tentacle_avg = tentacle_avg_y;
  }
  else if (sort_type == "z")
  {
    cout << "Sorting tentacles by z-axis..." << endl;
    tentacle_avg = tentacle_avg_z;
  }
  else
  {
    cout << "Tentabot::sort_tentacles -> sort_type is not specified! Sorting tentacles by y-axis..." << endl;
    tentacle_avg = tentacle_avg_y;
  }

  vector<double> sorted_indices = sort_indices(tentacle_avg);

  /*
  cout << "Tentabot::sort_tentacles -> Unsorted:" << endl;
  for (int i = 0; i < tentacle_avg.size(); ++i)
  {
    cout << i << " -> " << tentacle_avg[i] << endl;
  }
  */

  //cout << "Tentabot::sort_tentacles -> Sorted:" << endl;
  for (int i = 0; i < sorted_indices.size(); ++i)
  {
    //cout << i << " -> " << tentacle_avg[sorted_indices[i]] << endl;

    off_tuning_param.tentacle_data[i] = tentacle_data_original[sorted_indices[i]];

    if (off_tuning_param.velocity_control_data.size() > 0)
    {
      off_tuning_param.velocity_control_data[i] = velocity_control_data_original[sorted_indices[i]];
    }
  }
}

void Tentabot::fillRobotVisu()
{
  visu_param.robot_visu.ns = robot_param.robot_name;
  visu_param.robot_visu.action = visualization_msgs::Marker::ADD;
  visu_param.robot_visu.pose.position.x = 0;
  visu_param.robot_visu.pose.position.y = 0;
  visu_param.robot_visu.pose.position.z = 0;
  visu_param.robot_visu.pose.orientation.x = 0;
  visu_param.robot_visu.pose.orientation.y = 0;
  visu_param.robot_visu.pose.orientation.z = 0;
  visu_param.robot_visu.pose.orientation.w = 1;
  visu_param.robot_visu.id = 888;
  visu_param.robot_visu.type = visualization_msgs::Marker::SPHERE;
  visu_param.robot_visu.scale.x = robot_param.robot_bbx_x_max - robot_param.robot_bbx_x_min;
  visu_param.robot_visu.scale.y = robot_param.robot_bbx_y_max - robot_param.robot_bbx_y_min;
  visu_param.robot_visu.scale.z = robot_param.robot_bbx_z_max - robot_param.robot_bbx_z_min;
  visu_param.robot_visu.color.r = 0.128;
  visu_param.robot_visu.color.g = 0.0;
  visu_param.robot_visu.color.b = 0.128;
  visu_param.robot_visu.color.a = 1.0;
  visu_param.robot_visu.header.frame_id = robot_param.robot_frame_name;
}

void Tentabot::fillTentacleTsampSupportvoxVisu()
{
  int tentacle_cnt = off_tuning_param.tentacle_data.size();
  int tsamp_cnt;

  // VISUALIZE TENTACLES, SUPPORT/PRIORITY VOXELS
  for(int k = 0; k < tentacle_cnt; k++)
  {
    // SET TENTACLE VISUALIZATION SETTINGS
    visualization_msgs::Marker tentacle_line_strip;
    tentacle_line_strip.ns = "tentacle" + to_string(k);
    tentacle_line_strip.id = k;
    tentacle_line_strip.header.frame_id = robot_param.robot_frame_name;
    tentacle_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    tentacle_line_strip.action = visualization_msgs::Marker::ADD;
    tentacle_line_strip.pose.orientation.w = 1.0;
    tentacle_line_strip.scale.x = 0.02;
    tentacle_line_strip.color.r = 1.0;
    tentacle_line_strip.color.g = 1.0;
    tentacle_line_strip.color.b = 1.0;
    tentacle_line_strip.color.a = 1.0;

    // SET SAMPLING POINTS VISUALIZATION SETTINGS
    visualization_msgs::Marker tentacle_tsamp;
    tentacle_tsamp.ns = "sampling_points" + to_string(k);
    tentacle_tsamp.id = k;
    tentacle_tsamp.header.frame_id = robot_param.robot_frame_name;
    tentacle_tsamp.type = visualization_msgs::Marker::SPHERE_LIST;
    tentacle_tsamp.action = visualization_msgs::Marker::ADD;
    tentacle_tsamp.pose.orientation.w = 1.0;
    tentacle_tsamp.scale.x = 0.05;
    tentacle_tsamp.scale.y = 0.05;
    tentacle_tsamp.scale.z = 0.05;
    tentacle_tsamp.color.r = 0.0;
    tentacle_tsamp.color.g = 0.0;
    tentacle_tsamp.color.b = 0.0;
    tentacle_tsamp.color.a = 1;

    tsamp_cnt = off_tuning_param.tentacle_data[k].size();

    for(int p = 0; p < tsamp_cnt; p++)
    {
      if (tsamp_cnt > 1)
      {
        tentacle_line_strip.points.push_back(off_tuning_param.tentacle_data[k][p]);
      }
      else
      {
        tentacle_line_strip.points.push_back(off_tuning_param.tentacle_data[k][p]);
        tentacle_line_strip.points.push_back(off_tuning_param.tentacle_data[k][p]);
      }

      tentacle_tsamp.points.push_back(off_tuning_param.tentacle_data[k][p]);
    }
    visu_param.tentacle_visu.markers.push_back(tentacle_line_strip);
    visu_param.tsamp_visu.markers.push_back(tentacle_tsamp);

    // SET SUPPORT/PRIORITY VOXEL VISUALIZATION SETTINGS
    visualization_msgs::Marker support_vox_points;
    support_vox_points.ns = "support_voxel" + to_string(k);
    support_vox_points.id = k;
    support_vox_points.header.frame_id = robot_param.robot_frame_name;
    support_vox_points.type = visualization_msgs::Marker::CUBE_LIST;
    support_vox_points.action = visualization_msgs::Marker::ADD;
    support_vox_points.pose.orientation.w = 1.0;
    support_vox_points.scale.x = off_tuning_param.egrid_vdim;
    support_vox_points.scale.y = off_tuning_param.egrid_vdim;
    support_vox_points.scale.z = off_tuning_param.egrid_vdim;

    for(int s = 0; s < status_param.support_vox_data[k].size(); s++)
    {
      geometry_msgs::Point po;
      toPoint(status_param.support_vox_data[k][s].index, po);

      std_msgs::ColorRGBA sv_point_color;
      support_vox_points.points.push_back(po);

      sv_point_color.g = 0;
      sv_point_color.a = 0.05;

      if(status_param.support_vox_data[k][s].flag)
      { 
        sv_point_color.r = 1.0;   
        sv_point_color.b = 0;
      }
      else
      {
        sv_point_color.r = 0.5;
        sv_point_color.b = 0.5;
      }
      support_vox_points.colors.push_back(sv_point_color);
    }
    visu_param.support_vox_visu.markers.push_back(support_vox_points);
  }
}

void Tentabot::fillBestTentacleVisu()
{
  visu_param.best_tentacle_visu.markers.clear();

  // SET TENTACLE VISUALIZATION SETTINGS
  visualization_msgs::Marker tentacle_line_strip;
  tentacle_line_strip.ns = "best_tentacle";
  tentacle_line_strip.id = 1;
  tentacle_line_strip.header.frame_id = robot_param.robot_frame_name;
  tentacle_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  tentacle_line_strip.action = visualization_msgs::Marker::ADD;
  tentacle_line_strip.pose.orientation.w = 1.0;
  tentacle_line_strip.scale.x = 0.02;
  tentacle_line_strip.color.r = 1.0;
  tentacle_line_strip.color.g = 0.647;
  tentacle_line_strip.color.b = 0.039;
  tentacle_line_strip.color.a = 1.0;

  // SET SAMPLING POINTS VISUALIZATION SETTINGS
  visualization_msgs::Marker tentacle_tsamp;
  tentacle_tsamp.ns = "best_sampling_points";
  tentacle_tsamp.id = 1;
  tentacle_tsamp.header.frame_id = robot_param.robot_frame_name;
  tentacle_tsamp.type = visualization_msgs::Marker::SPHERE_LIST;
  tentacle_tsamp.action = visualization_msgs::Marker::ADD;
  tentacle_tsamp.pose.orientation.w = 1.0;
  tentacle_tsamp.scale.x = 0.05;
  tentacle_tsamp.scale.y = 0.05;
  tentacle_tsamp.scale.z = 0.05;
  tentacle_tsamp.color.r = 0.0;
  tentacle_tsamp.color.g = 0.0;
  tentacle_tsamp.color.b = 0.0;
  tentacle_tsamp.color.a = 1;

  double tsamp_cnt = off_tuning_param.tentacle_data[status_param.best_tentacle].size();

  for(int p = 0; p < tsamp_cnt; p++)
  {
    if (tsamp_cnt > 1)
    {
      tentacle_line_strip.points.push_back(off_tuning_param.tentacle_data[status_param.best_tentacle][p]);
    }
    else
    {
      tentacle_line_strip.points.push_back(off_tuning_param.tentacle_data[status_param.best_tentacle][p]);
      tentacle_line_strip.points.push_back(off_tuning_param.tentacle_data[status_param.best_tentacle][p]);
    }

    tentacle_tsamp.points.push_back(off_tuning_param.tentacle_data[status_param.best_tentacle][p]);
  }
  visu_param.best_tentacle_visu.markers.push_back(tentacle_line_strip);
  visu_param.best_tentacle_visu.markers.push_back(tentacle_tsamp);
}

void Tentabot::fillPathVisu()
{
  if(process_param.visu_flag == true)
  {
    visu_param.path_visu.ns = "path2glory";
    visu_param.path_visu.id = 753;
    visu_param.path_visu.type = visualization_msgs::Marker::SPHERE_LIST;
    visu_param.path_visu.action = visualization_msgs::Marker::ADD;
    visu_param.path_visu.pose.orientation.w = 1.0;
    visu_param.path_visu.scale.x = 0.2;
    visu_param.path_visu.scale.y = 0.2;
    visu_param.path_visu.scale.z = 0.2;
    visu_param.path_visu.color.r = 1.0;
    visu_param.path_visu.color.g = 0.1;
    visu_param.path_visu.color.b = 1.0;
    visu_param.path_visu.color.a = 1.0;
  }
}

void Tentabot::fillCommandVisu()
{
  if(process_param.visu_flag == true)
  {
    visu_param.command_visu.ns = "commandante";
    visu_param.command_visu.id = 1928;
    visu_param.command_visu.type = visualization_msgs::Marker::SPHERE_LIST;
    visu_param.command_visu.action = visualization_msgs::Marker::ADD;
    visu_param.command_visu.pose.orientation.w = 1.0;
    visu_param.command_visu.scale.x = 0.2;
    visu_param.command_visu.scale.y = 0.2;
    visu_param.command_visu.scale.z = 0.2;
    visu_param.command_visu.color.r = 0.05;
    visu_param.command_visu.color.g = 1.0;
    visu_param.command_visu.color.b = 1.0;
    visu_param.command_visu.color.a = 1.0;
  }
}

void Tentabot::fillDebugVisu(geometry_msgs::Point p, string frame_name)
{
  visu_param.debug_visu.ns = robot_param.robot_name;
  visu_param.debug_visu.action = visualization_msgs::Marker::ADD;
  visu_param.debug_visu.pose.position.x = p.x;
  visu_param.debug_visu.pose.position.y = p.y;
  visu_param.debug_visu.pose.position.z = p.z;
  visu_param.debug_visu.pose.orientation.x = 0;
  visu_param.debug_visu.pose.orientation.y = 0;
  visu_param.debug_visu.pose.orientation.z = 0;
  visu_param.debug_visu.pose.orientation.w = 1;
  visu_param.debug_visu.id = 7;
  visu_param.debug_visu.type = visualization_msgs::Marker::SPHERE;
  visu_param.debug_visu.scale.x = 0.1;
  visu_param.debug_visu.scale.y = 0.1;
  visu_param.debug_visu.scale.z = 0.1;
  visu_param.debug_visu.color.r = 1.0;
  visu_param.debug_visu.color.g = 0.7;
  visu_param.debug_visu.color.b = 0.0;
  visu_param.debug_visu.color.a = 1.0;
  visu_param.debug_visu.header.frame_id = frame_name;
}

void Tentabot::fillDebugArrayVisu(vector<geometry_msgs::Point> v, string frame_name)
{
  visu_param.debug_array_visu.markers.clear();

  for(int i = 0; i < v.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.ns = "point" + to_string(i);
    marker.id = i;
    marker.header.frame_id = frame_name;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.position = v[i];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    visu_param.debug_array_visu.markers.push_back(marker); 
  }
}

void Tentabot::interpol(geometry_msgs::Point p, double dist, tf::Vector3& pint)
{
  double p_norm = find_norm(p);
  double m;

  //ROS_INFO_STREAM("pnorm: " << p_norm);

  if (p_norm == 0)
  {
    m = 0.0;
  }
  else if (p_norm > dist)
  {
    m = dist / p_norm;
  }
  else
  {
    m = 1.0;
  }
  
  pint.setValue(m * p.x, m * p.y, m * p.z);
}

geometry_msgs::Point Tentabot::interpol(geometry_msgs::Point p, double dist)
{
  double p_norm = find_norm(p);
  double m;

  //ROS_INFO_STREAM("pnorm: " << p_norm);

  if (p_norm == 0)
  {
    m = 0.0;
  }
  else if (p_norm > dist)
  {
    m = dist / p_norm;
  }
  else
  {
    m = 1.0;
  }
  
  geometry_msgs::Point pm;
  pm.x = m * p.x;
  pm.y = m * p.y;
  pm.z = m * p.z;

  return pm;
}

geometry_msgs::Point Tentabot::interpol(geometry_msgs::Point p1, 
                                        geometry_msgs::Point p2, 
                                        double dist_from_p1)
{
  geometry_msgs::Point v;
  v.x = p2.x - p1.x;
  v.y = p2.y - p1.y;
  v.z = p2.z - p1.z;

  double v_norm = find_norm(v);
  double m;

  if (v_norm == 0)
  {
    m = 0.0;
  }
  else if (v_norm > dist_from_p1)
  {
    m = dist_from_p1 / v_norm;
  }
  else
  {
    m = 1.0;
  }

  geometry_msgs::Point pm;
  pm.x = p1.x + m * v.x;
  pm.y = p1.y + m * v.y;
  pm.z = p1.z + m * v.z;

  return pm;
}

vector<geometry_msgs::Point> Tentabot::equadistant(geometry_msgs::Point p1, 
                                                   geometry_msgs::Point p2, 
                                                   int num_btw)
{
  vector<geometry_msgs::Point> p_vector;
  double t;
  double delta_t = find_Euclidean_distance(p1, p2) / (10 * (num_btw + 1));

  for (int i = 0; i < num_btw; ++i)
  {
    t = delta_t * (i+1);

    geometry_msgs::Point p;
    p.x = p1.x + (p2.x - p1.x) * t;
    p.y = p1.y + (p2.y - p1.y) * t;
    p.z = p1.z + (p2.z - p1.z) * t;
    p_vector.push_back(p);
  }
  return p_vector;
}

bool Tentabot::isOccupied(double x, double y, double z)
{
  OcTreeNode* node = status_param.tmap -> search(x, y, z);
  if(node)
  {
    return status_param.tmap -> isNodeOccupied(node);
  }
  else
  {
    return true;
  }
}

void Tentabot::update_planning_states()
{
  // UPDATE COUNTER BEFORE PLANNING
  status_param.counter++;

  // UPDATE CURRENT ROBOT POSE BEFORE PLANNING
  status_param.robot_pose = status_param.measured_robot_pose;

  // UPDATE CURRENT MAP BEFORE PLANNING
  status_param.tmap = std::shared_ptr<octomap::ColorOcTree> (dynamic_cast<octomap::ColorOcTree*> (octomap_msgs::msgToMap(status_param.measured_map_msg)));

  // UPDATE CURRENT TRANSFORMATION MATRIX OF ROBOT WRT WORLD BEFORE PLANNING
  try
  {
    //tflistener -> waitForTransform(status_param.map_frame_name, robot_param.robot_frame_name, ros::Time::now(), ros::Duration(1.0));
    tflistener -> lookupTransform(status_param.map_frame_name, robot_param.robot_frame_name, ros::Time(0), status_param.transform_robot_wrt_world);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("Tentabot::update_planning_states -> Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  // UPDATE DELTA TIME BEFORE PLANNING
  if (status_param.counter == 1)
  {
    status_param.dt = process_param.nav_dt;
  }
  else
  {
    status_param.dt = (ros::Time::now() - status_param.prev_time).toSec();
  }

  // UPDATE PREVIOUS TIME BEFORE PLANNING 
  status_param.prev_time = ros::Time::now();

  // UPDATE CURRENT LATERAL SPEED BEFORE PLANNING
  double delta_dist = find_Euclidean_distance(status_param.prev_robot_pose.position, status_param.robot_pose.position);
  status_param.lat_speed = delta_dist / status_param.dt;
  
  // UPDATE CURRENT ANGULAR VELOCITY BEFORE PLANNING
  tf::Quaternion previous_q(status_param.prev_robot_pose.orientation.x,
                            status_param.prev_robot_pose.orientation.y, 
                            status_param.prev_robot_pose.orientation.z, 
                            status_param.prev_robot_pose.orientation.w);
  tf::Quaternion current_q( status_param.robot_pose.orientation.x, 
                            status_param.robot_pose.orientation.y, 
                            status_param.robot_pose.orientation.z, 
                            status_param.robot_pose.orientation.w);
  tf::Quaternion delta_q = current_q - previous_q;

  tf::Matrix3x3 previous_m(previous_q);
  double previous_roll, previous_pitch, previous_yaw;
  previous_m.getRPY(previous_roll, previous_pitch, previous_yaw);

  tf::Matrix3x3 current_m(current_q);
  double current_roll, current_pitch, current_yaw;
  current_m.getRPY(current_roll, current_pitch, current_yaw);

  status_param.yaw_velo = (current_yaw - previous_yaw) / status_param.dt;

  // UPDATE TOTAL NAVIGATION DURATION AND PATH LENGTH BEFORE PLANNING
  status_param.nav_duration += status_param.dt;
  status_param.nav_length += delta_dist;

  // UPDATE PREVIOUS ROBOT POSE
  status_param.prev_robot_pose = status_param.robot_pose;

  /*
  std::cout << "counter: " << status_param.counter << endl;
  std::cout << "dt: " << status_param.dt << endl;
  std::cout << "lat_speed: " << status_param.lat_speed << std::endl;
  std::cout << "yaw_velo: " << status_param.yaw_velo << std::endl;
  */
}

// DESCRIPTION: UPDATE OCCUPANCY MAP OF THE ROBOT
void Tentabot::update_ego_grid_data()
{
  // UPDATE THE LINEAR OCCUPANCY GRID AROUND THE ROBOT
  int vox_index;
  int total_voxel_cnt = status_param.ego_grid_data.ovox_pos.size();
  status_param.ego_grid_data.ovox_value.clear(); 
  status_param.ego_grid_data.ovox_value.resize(total_voxel_cnt);
  
  if (process_param.visu_flag == true)
  {
    visu_param.occupancy_pc.points.clear();
  }

  tf::Vector3 bbx_min(status_param.egrid_dist_x_min, status_param.egrid_dist_y_min, status_param.egrid_dist_z_min);
  tf::Vector3 bbx_max(status_param.egrid_dist_x_max, status_param.egrid_dist_y_max, status_param.egrid_dist_z_max);

  for(octomap::ColorOcTree::iterator it = status_param.tmap -> begin(); it != status_param.tmap -> end(); ++it)
  {
    // status_param.tmap -> inBBX(it.getKey())
    tf::Vector3 op_wrt_world(it.getX(), it.getY(), it.getZ());
    tf::Vector3 op_wrt_robot = status_param.transform_robot_wrt_world.inverse() * op_wrt_world;

    if ( isInBBx(op_wrt_robot, bbx_min, bbx_max) && isOccupied(op_wrt_world.x(), op_wrt_world.y(), op_wrt_world.z()) )
    {
      vox_index = toLinIndex(op_wrt_robot);

      if ( vox_index >= 0 && vox_index < total_voxel_cnt )
      {
        if (status_param.ego_grid_data.ovox_value[vox_index] < off_tuning_param.max_occupancy_belief_value)
        {
          status_param.ego_grid_data.ovox_value[vox_index] = off_tuning_param.max_occupancy_belief_value;

          if (process_param.visu_flag == true)
          {
            geometry_msgs::Point32 po;
            po.x = op_wrt_robot.x();
            po.y = op_wrt_robot.y();
            po.z = op_wrt_robot.z();

            visu_param.occupancy_pc.points.push_back(po);
          }
        }
      }
    }
  }
}

void Tentabot::update_heuristic_values()
{
  int tentacle_cnt = off_tuning_param.tentacle_data.size();
  int tsamp_cnt;
  double crash_dist;
  double tlen_k;
  double delta_len;

  // NUA TODO: FIND A BETTER WAY TO ENSURE GOAL IS AVAILABLE TO AVOID CRASH!
  while(goal_util.getGoal().size() <= 0)
  {
    ;
  }
  geometry_msgs::Pose active_goal = goal_util.getActiveGoal();

  double dist2goal = find_Euclidean_distance(active_goal.position, status_param.robot_pose.position);

  vector<int> tentacle_bin;
  double total_weight;
  double total_weighted_occupancy;
  bool occupancy_flag;
  
  double temp_len;
  double clutterness_value_avg;

  int close_index;
  double close_value;
  double min_close_value;

  double max_closeness = 0;
  double max_smoothness_dist = 0;

  // CLEAR NAVIGABILITY, CLEARANCE, clutterness AND CLOSENESS SETS
  heuristic_param.occupancy_set.clear();  
  heuristic_param.occupancy_set.resize(tentacle_cnt);

  heuristic_param.navigability_set.clear();  
  heuristic_param.navigability_set.resize(tentacle_cnt);
  
  heuristic_param.clearance_set.clear();
  heuristic_param.clearance_set.resize(tentacle_cnt);

  heuristic_param.clutterness_set.clear();
  heuristic_param.clutterness_set.resize(tentacle_cnt);
    
  heuristic_param.closeness_set.clear();
  heuristic_param.closeness_set.resize(tentacle_cnt);

  heuristic_param.smoothness_set.clear();
  heuristic_param.smoothness_set.resize(tentacle_cnt);

  // FOR EACH TENTACLE
  for(int k = 0; k < tentacle_cnt; k++)
  {
    tsamp_cnt = off_tuning_param.tentacle_data[k].size();

    // CALCULATE TOTAL WEIGHT AND TOTAL WEIGHTED OCCUPANCY FOR EACH TENTACLE
    total_weight = 0;
    total_weighted_occupancy = 0;
    vector<double> total_weighted_occupancy_vector(tsamp_cnt, 0);
    occupancy_flag = false;
    tentacle_bin.clear();
    tentacle_bin.resize(tsamp_cnt);

    for(int s = 0; s < status_param.support_vox_data[k].size(); s++)
    {   
      if(status_param.support_vox_data[k][s].flag && status_param.ego_grid_data.ovox_value[status_param.support_vox_data[k][s].index] > 0)
      {
        tentacle_bin[status_param.support_vox_data[k][s].histbin] += 1;
      }

      total_weight += status_param.support_vox_data[k][s].weight;
      total_weighted_occupancy += status_param.support_vox_data[k][s].weight * status_param.ego_grid_data.ovox_value[status_param.support_vox_data[k][s].index];
 
      total_weighted_occupancy_vector[status_param.support_vox_data[k][s].histbin] += status_param.support_vox_data[k][s].weight * status_param.ego_grid_data.ovox_value[status_param.support_vox_data[k][s].index];
    }

    // DETERMINE OCCUPANCY VALUE OF THE TENTACLE
    bool occupied_flag = false;

    for (int m = 0; m < tsamp_cnt; ++m)
    {
      if (tentacle_bin[m] >= on_tuning_param.tbin_obs_cnt_threshold)
      {
        occupied_flag = true;
      }

      if (occupied_flag)
      {
        heuristic_param.occupancy_set[k] += status_param.sample_weight_data[k][m] * off_tuning_param.max_occupancy_belief_value;
      }
      else
      {
        heuristic_param.occupancy_set[k] += total_weighted_occupancy_vector[m];
      }
    }
    heuristic_param.occupancy_set[k] /= status_param.tentacle_weight_data[k] * off_tuning_param.max_occupancy_belief_value;

    // DETERMINE NAVIGABILITY OF THE TENTACLE, 1: NAVIGABLE, 0: NON-NAVIGABLE, -1: TEMPORARILY NAVIGABLE
    int b = 0;
    bool navigability_end_flag = false;
    tlen_k = status_param.tentacle_length_data[k];
    //crash_dist = on_tuning_param.crash_dist_scale * tlen_k * status_param.lat_speed / robot_param.dummy_max_lat_velo;
    crash_dist = on_tuning_param.crash_dist_scale * tlen_k;
    delta_len = tlen_k / tsamp_cnt;

    if (crash_dist < delta_len)
    {
      crash_dist = delta_len;
    }
    else if (crash_dist > tlen_k)
    {
      crash_dist = tlen_k;
    }

    heuristic_param.navigability_set[k] = 1;

    if(process_param.visu_flag == true)
    {
      visu_param.tentacle_visu.markers[k].color.r = 0.0;
      visu_param.tentacle_visu.markers[k].color.g = 1.0;
      visu_param.tentacle_visu.markers[k].color.b = 0.0;
    }

    status_param.tcrash_bin[k] = tsamp_cnt - 1;

    while (!navigability_end_flag && (b < tsamp_cnt))
    {
      if (tentacle_bin[b] >= on_tuning_param.tbin_obs_cnt_threshold)
      {
        temp_len = delta_len * (b + 1);

        if (temp_len > crash_dist)
        {
          heuristic_param.navigability_set[k] = -1;

          if(process_param.visu_flag == true)
          {
            visu_param.tentacle_visu.markers[k].color.r = 0.0;
            visu_param.tentacle_visu.markers[k].color.g = 0.0;
            visu_param.tentacle_visu.markers[k].color.b = 1.0;
          }
        }
        else
        {
          heuristic_param.navigability_set[k] = 0;

          if(process_param.visu_flag == true)
          {
            visu_param.tentacle_visu.markers[k].color.r = 1.0;
            visu_param.tentacle_visu.markers[k].color.g = 0.0;
            visu_param.tentacle_visu.markers[k].color.b = 0.0;
          }
        }

        if (b == 0)
        {
          status_param.tcrash_bin[k] = b;
        }
        else
        {
          status_param.tcrash_bin[k] = b-1;
        }
        navigability_end_flag = true;
      }
      b++;
    }

    // DETERMINE CLEARANCE VALUE
    if (temp_len < delta_len)
    {
      temp_len = 0;
    }
    else
    {
      temp_len -= delta_len;
    }
    
    if(heuristic_param.navigability_set[k] == 1)
    {
      heuristic_param.clearance_set[k] = 0;
    }
    else
    {
      heuristic_param.clearance_set[k] = 1 - temp_len / (delta_len * tsamp_cnt);
    }

    // DETERMINE NEARBY CLUTTERNESS VALUE
    clutterness_value_avg = total_weighted_occupancy / total_weight;
    heuristic_param.clutterness_set[k] = (2 / (1 + exp(-1 * clutterness_value_avg))) - 1;

    // DETERMINE TARGET CLOSENESS VALUE
    tf::Vector3 crash_po_wrt_robot( off_tuning_param.tentacle_data[k][status_param.tcrash_bin[k]].x,
                                    off_tuning_param.tentacle_data[k][status_param.tcrash_bin[k]].y,
                                    off_tuning_param.tentacle_data[k][status_param.tcrash_bin[k]].z);
    tf::Vector3 crash_po_wrt_world = status_param.transform_robot_wrt_world * crash_po_wrt_robot;
    min_close_value = find_Euclidean_distance(active_goal.position, crash_po_wrt_world);
    close_index = status_param.tcrash_bin[k];
    
    for (int c = 0; c < status_param.tcrash_bin[k]; ++c)
    {
      crash_po_wrt_robot.setValue(off_tuning_param.tentacle_data[k][c].x,
                                  off_tuning_param.tentacle_data[k][c].y,
                                  off_tuning_param.tentacle_data[k][c].z);
      crash_po_wrt_world = status_param.transform_robot_wrt_world * crash_po_wrt_robot;
      close_value = find_Euclidean_distance(active_goal.position, crash_po_wrt_world);

      if (min_close_value > close_value)
      {
        min_close_value = close_value;
        close_index = c;
      }
    }
    heuristic_param.closeness_set[k] = min_close_value;

    if(max_closeness < heuristic_param.closeness_set[k])
    {
      max_closeness = heuristic_param.closeness_set[k];
    }

    //DETERMINE SMOOTHNESS VALUE  
    if(status_param.ex_best_tentacle > 0)
    {
      heuristic_param.smoothness_set[k] = find_Euclidean_distance(off_tuning_param.tentacle_data[status_param.ex_best_tentacle][0], 
                                                                  off_tuning_param.tentacle_data[k][0]);

      if(max_smoothness_dist < heuristic_param.smoothness_set[k])
      {
        max_smoothness_dist = heuristic_param.smoothness_set[k];
      }
    }
    else
    {
      max_smoothness_dist = 1;
      heuristic_param.smoothness_set[k] = 0;
    }
  }

  //fillDebugArrayVisu(v, robot_param.robot_frame_name);
  //publishDebugArrayVisu();

  for(int k = 0; k < tentacle_cnt; k++)
  {
    heuristic_param.closeness_set[k] = heuristic_param.closeness_set[k] / max_closeness;
    heuristic_param.smoothness_set[k] = heuristic_param.smoothness_set[k] / max_smoothness_dist;
  }
}

// DESCRIPTION: SELECT THE BEST TENTACLE
void Tentabot::select_best_tentacle()
{
  int tentacle_cnt = off_tuning_param.tentacle_data.size();
  double tentacle_value;
  double weighted_clearance_value;
  double weighted_clutterness_value;
  double weighted_closeness_value;
  double weighted_smoothness_value;
  double best_tentacle_value = INF;
  bool no_drivable_tentacle = true;

  for(int k = 0; k < tentacle_cnt; k++)
  {
    if( heuristic_param.navigability_set[k] != 0 )
    {
      weighted_clearance_value = on_tuning_param.clear_scale * heuristic_param.clearance_set[k];
      weighted_clutterness_value = on_tuning_param.clutter_scale * heuristic_param.clutterness_set[k];
      weighted_closeness_value = on_tuning_param.close_scale * heuristic_param.closeness_set[k];
      weighted_smoothness_value = on_tuning_param.smooth_scale * heuristic_param.smoothness_set[k];

      tentacle_value = weighted_clearance_value + weighted_clutterness_value + weighted_closeness_value + weighted_smoothness_value;

      /*
      ROS_INFO_STREAM("Tentacle " << k);
      ROS_INFO_STREAM("tentacle_value: " << tentacle_value);
      cout << "" << endl;
      ROS_INFO_STREAM("clear_scale: " << on_tuning_param.clear_scale);
      ROS_INFO_STREAM("clearance: " << heuristic_param.clearance_set[k]);
      ROS_INFO_STREAM("weighted_clearance_value: " << weighted_clearance_value);
      cout << "" << endl;
      ROS_INFO_STREAM("clutter_scale: " << on_tuning_param.clutter_scale);
      ROS_INFO_STREAM("clutterness: " << heuristic_param.clutterness_set[k]);
      ROS_INFO_STREAM("weighted_clutterness_value: " << weighted_clutterness_value);
      cout << "" << endl;
      ROS_INFO_STREAM("close_scale: " << on_tuning_param.close_scale);
      ROS_INFO_STREAM("closeness: " << heuristic_param.closeness_set[k]);
      ROS_INFO_STREAM("weighted_closeness_value: " << weighted_closeness_value);
      cout << "" << endl;
      ROS_INFO_STREAM("smooth_scale: " << on_tuning_param.smooth_scale);
      ROS_INFO_STREAM("smoothness: " << heuristic_param.smoothness_set[k]);
      ROS_INFO_STREAM("weighted_smoothness_value: " << weighted_smoothness_value);
      cout << "-----------------------------------------" << endl;
      cout << "" << endl;
      */

      if(no_drivable_tentacle)
      {
          best_tentacle_value = tentacle_value;
          status_param.best_tentacle = k;
          no_drivable_tentacle = false;
      }
      else
      {
        if(best_tentacle_value > tentacle_value)
        {
          best_tentacle_value = tentacle_value;
          status_param.best_tentacle = k;   
        }
      }
    }
  }

  if(no_drivable_tentacle == true)
  {
    cout << "Tentabot::select_best_tentacle -> No drivable tentacle!" << endl;

    status_param.navigability_flag = false;
    
    for(int k = 0; k < tentacle_cnt; k++)
    {
      tentacle_value = on_tuning_param.clear_scale * heuristic_param.clearance_set[k] + 
                       on_tuning_param.clutter_scale * heuristic_param.clutterness_set[k] + 
                       on_tuning_param.close_scale * heuristic_param.closeness_set[k] + 
                       on_tuning_param.smooth_scale * heuristic_param.smoothness_set[k];

      if(k == 0)
      {
          best_tentacle_value = tentacle_value;
          status_param.best_tentacle = 0;
      }
      else
      {
        if(best_tentacle_value > tentacle_value)
        {
          best_tentacle_value = tentacle_value;
          status_param.best_tentacle = k;   
        }
      }
    }
  }
  else
  {
    status_param.navigability_flag = true;
  }

  if(process_param.visu_flag == true)
  {
    fillBestTentacleVisu();
    /*
    visu_param.tentacle_visu.markers[status_param.best_tentacle].color.r = 0.0;
    visu_param.tentacle_visu.markers[status_param.best_tentacle].color.g = 1.0;
    visu_param.tentacle_visu.markers[status_param.best_tentacle].color.b = 1.0;
    */
  }

  /*
  ROS_INFO_STREAM("best tentacle " << status_param.best_tentacle);
  ROS_INFO_STREAM("tentacle_value: " << best_tentacle_value);
  cout << "" << endl;
  ROS_INFO_STREAM("clear_scale: " << on_tuning_param.clear_scale);
  ROS_INFO_STREAM("clearance: " << heuristic_param.clearance_set[status_param.best_tentacle]);
  ROS_INFO_STREAM("weighted_clearance_value: " << on_tuning_param.clear_scale * heuristic_param.clearance_set[status_param.best_tentacle]);
  cout << "" << endl;
  ROS_INFO_STREAM("clutter_scale: " << on_tuning_param.clutter_scale);
  ROS_INFO_STREAM("clutterness: " << heuristic_param.clutterness_set[status_param.best_tentacle]);
  ROS_INFO_STREAM("weighted_clutterness_value: " << on_tuning_param.clutter_scale * heuristic_param.clutterness_set[status_param.best_tentacle]);
  cout << "" << endl;
  ROS_INFO_STREAM("close_scale: " << on_tuning_param.close_scale);
  ROS_INFO_STREAM("closeness: " << heuristic_param.closeness_set[status_param.best_tentacle]);
  ROS_INFO_STREAM("weighted_closeness_value: " << on_tuning_param.close_scale * heuristic_param.closeness_set[status_param.best_tentacle]);
  cout << "" << endl;
  ROS_INFO_STREAM("smooth_scale: " << on_tuning_param.smooth_scale);
  ROS_INFO_STREAM("smoothness: " << heuristic_param.smoothness_set[status_param.best_tentacle]);
  ROS_INFO_STREAM("weighted_smoothness_value: " << on_tuning_param.smooth_scale * heuristic_param.smoothness_set[status_param.best_tentacle]);
  cout << "-----------------------------------------" << endl;
  cout << "" << endl;
  */
    
  status_param.ex_best_tentacle = status_param.best_tentacle;
}

void Tentabot::debug_rotate()
{
  double next_yaw = 0.2;
  tf::Vector3 next_point_wrt_robot(0, 0, 0);

  tf::Quaternion next_quat_wrt_robot;
  next_quat_wrt_robot.setRPY(0, 0, next_yaw);
  tf::Quaternion next_quat_wrt_world = status_param.transform_robot_wrt_world * next_quat_wrt_robot;
  tf::quaternionTFToMsg(next_quat_wrt_world, status_param.command_pose.pose.orientation);

  tf::Vector3 next_point_wrt_world = status_param.transform_robot_wrt_world * next_point_wrt_robot;
  status_param.command_pose.pose.position.x = next_point_wrt_world.x();
  status_param.command_pose.pose.position.y = next_point_wrt_world.y();
  status_param.command_pose.pose.position.z = 1;
  
  //status_param.command_pose.header.seq++;
  status_param.command_pose.header.stamp = ros::Time::now();
  status_param.command_pose.header.frame_id = status_param.map_frame_name;
  status_param.command_pose_pub.publish(status_param.command_pose);

}

// DESCRIPTION: MOVE THE ROBOT SENDING CALCULATED POSE TO THE CONTROLLER
void Tentabot::send_motion_command_by_pose_control()
{
  geometry_msgs::Pose active_goal = goal_util.getActiveGoal();

  double dist2goal = find_Euclidean_distance(active_goal.position, status_param.robot_pose.position);

  if(status_param.nav_result == -1)                         // crash
  {
    cout << "OMG! I think I've just hit something!!!" << endl;
    status_param.navexit_flag = true;
  }
  else if(dist2goal < process_param.goal_close_threshold)   // reach goal
  {
    if(!goal_util.switchActiveGoal())
    {
      cout << "Cawabunga! The goal has reached!" << endl;
      status_param.navexit_flag = true;
      status_param.nav_result = 1;
    }
    else
    {
      cout << "Yey! Waypoint #" << goal_util.getActiveGoalIndex()-1 << " has reached!" << endl;
      active_goal = goal_util.getActiveGoal();
      dist2goal = find_Euclidean_distance(active_goal.position, status_param.robot_pose.position);
    }
  }
  else if(status_param.nav_duration > process_param.time_limit)     // time-out
  {
    cout << "Ugh! I am too late..." << endl;
    status_param.navexit_flag = true;
    status_param.nav_result = 0;
  }

  if(!status_param.navexit_flag)
  {
    double next_yaw;
    double min_speed = 0.1;
    double speed_inc_dec = 0.2;
    double weight_inc_dec = 0.2;
    double angular_velocity_weight = 3.0;
    
    tf::Vector3 next_point_wrt_robot( off_tuning_param.tentacle_data[status_param.best_tentacle][0].x, 
                                      off_tuning_param.tentacle_data[status_param.best_tentacle][0].y, 
                                      off_tuning_param.tentacle_data[status_param.best_tentacle][0].z);

    double max_yaw_angle_dt = robot_param.dummy_max_yaw_velo * status_param.dt;

    if (status_param.counter == 1)
    {
      status_param.command_pose.pose = robot_param.init_robot_pose;
    }
    else
    {
      // CALCULATE NEXT YAW ANGLE
      next_yaw = atan2(next_point_wrt_robot.y(), next_point_wrt_robot.x());
      if (abs(next_yaw) > max_yaw_angle_dt)
      {
        next_yaw *= max_yaw_angle_dt / abs(next_yaw);
      }
      next_yaw *= angular_velocity_weight;

      // CONDITIONS TO SLOW DOWN THE LATERAL SPEED
      if(dist2goal < 3.0)
      {
        status_param.desired_lat_speed -= speed_inc_dec;
      }
      else
      {
        status_param.desired_lat_speed += speed_inc_dec;
      }

      // CHECK THE LATERAL SPEED BOUNDARIES
      if(status_param.desired_lat_speed > robot_param.dummy_max_lat_velo)
      {
        status_param.desired_lat_speed = robot_param.dummy_max_lat_velo;
      }
      else if(status_param.desired_lat_speed < min_speed)
      {
        status_param.desired_lat_speed = min_speed;
      }
      
      // CALCULATE NEXT POSITION
      if (status_param.desired_lat_speed >= status_param.lat_speed && abs(next_yaw) < 0.25*PI)
      {
        //lateral_velocity_weight = status_param.lat_speed / status_param.desired_lat_speed;
        status_param.lat_speed_weight += weight_inc_dec;
      }
      else
      {
        status_param.lat_speed_weight -= weight_inc_dec;
        //lateral_velocity_weight = status_param.desired_lat_speed / status_param.lat_speed;
      }

      if (status_param.lat_speed_weight < 0.5)
      {
        status_param.lat_speed_weight = 0.5;
      }

      if (status_param.lat_speed_weight > 4.0)
      {
        status_param.lat_speed_weight = 4.0;
      }

      next_point_wrt_robot *= status_param.lat_speed_weight;
    }

    /*
    // UPDATE CURRENT TRANSFORMATION MATRIX OF ROBOT WRT WORLD BEFORE EXECUTING CALCULATED POSE
    tf::Transform transform_robot_wrt_world;
    tf::StampedTransform transform_current_robot_wrt_world;
    try
    {
      //tflistener -> waitForTransform(map_util.getMapFrameName(), robot_param.robot_frame_name, ros::Time::now(), ros::Duration(1.0));
      tflistener -> lookupTransform(status_param.map_frame_name, robot_param.robot_frame_name, ros::Time(0), transform_current_robot_wrt_world);
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("Tentabot::send_motion_command_by_pose_control -> Couldn't get transform!");
      ROS_ERROR("%s", ex.what());
    }
    transform_robot_wrt_world = transform_current_robot_wrt_world * transform_current_robot_wrt_world.inverse() * status_param.transform_robot_wrt_world;
    */

    // UPDATE THE COMMAND POSE
    tf::Quaternion next_quat_wrt_robot;
    next_quat_wrt_robot.setRPY(0, 0, next_yaw);
    tf::Quaternion next_quat_wrt_world = status_param.transform_robot_wrt_world * next_quat_wrt_robot;
    tf::quaternionTFToMsg(next_quat_wrt_world, status_param.command_pose.pose.orientation);

    tf::Vector3 next_point_wrt_world = status_param.transform_robot_wrt_world * next_point_wrt_robot;
    status_param.command_pose.pose.position.x = next_point_wrt_world.x();
    status_param.command_pose.pose.position.y = next_point_wrt_world.y();
    status_param.command_pose.pose.position.z = next_point_wrt_world.z();
    
    //status_param.command_pose.header.seq++;
    status_param.command_pose.header.stamp = ros::Time::now();
    status_param.command_pose.header.frame_id = status_param.map_frame_name;
    status_param.command_pose_pub.publish(status_param.command_pose);

    /*
    cout << "-----------------" << endl;
    cout << "Tentabot::send_motion_command_by_pose_control -> counter: " << status_param.counter << endl;
    cout << "command_pose.position.x:  " << status_param.command_pose.pose.position.x << endl;
    cout << "command_pose.position.y:  " << status_param.command_pose.pose.position.y << endl;
    cout << "command_pose.position.z:  " << status_param.command_pose.pose.position.z << endl;
    cout << "-----------------" << endl;
    */

    visu_param.command_visu.points.push_back(status_param.command_pose.pose.position);
  }
}

void Tentabot::send_motion_command_by_velocity_control()
{
  geometry_msgs::Pose active_goal = goal_util.getActiveGoal();

  double dist2goal = find_Euclidean_distance(active_goal.position, status_param.robot_pose.position);

  if(status_param.nav_result == -1)                         // crash
  {
    cout << "OMG! I think I've just hit something!!!" << endl;
    status_param.navexit_flag = true;
  }
  else if(dist2goal < process_param.goal_close_threshold)   // reach goal
  {
    if(!goal_util.switchActiveGoal())
    {
      cout << "Cawabunga! The goal has reached!" << endl;
      status_param.navexit_flag = true;
      status_param.nav_result = 1;
    }
    else
    {
      cout << "Yey! Waypoint #" << goal_util.getActiveGoalIndex()-1 << " has reached!" << endl;
      active_goal = goal_util.getActiveGoal();
      dist2goal = find_Euclidean_distance(active_goal.position, status_param.robot_pose.position);
    }
  }
  else if(status_param.nav_duration > process_param.time_limit)     // time-out
  {
    cout << "Ugh! I am too late..." << endl;
    status_param.navexit_flag = true;
    status_param.nav_result = 0;
  }

  if(!status_param.navexit_flag)
  {
    status_param.command_velo.linear.x = off_tuning_param.velocity_control_data[status_param.best_tentacle][0];
    status_param.command_velo.angular.z = off_tuning_param.velocity_control_data[status_param.best_tentacle][1];
    status_param.command_velo_pub.publish(status_param.command_velo);

    /*
    cout << "-----------------" << endl;
    cout << "command_velo.linear.x:  " << status_param.command_velo.linear.x << endl;
    cout << "command_velo.angular.z:  " << status_param.command_velo.angular.z << endl;
    cout << "-----------------" << endl;
    */
  }
}