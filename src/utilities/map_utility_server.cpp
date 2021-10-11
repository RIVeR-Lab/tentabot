// LAST UPDATE: 2021.10.05
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:
// 1) Map Service is not working with main callback. Asynchronous spin is working but then map gets messy!

// --CUSTOM LIBRARIES--
#include "map_utility.h"

int main(int argc, char** argv)
{
  cout << "map_utility_server::main -> Map Utility Server is processing..." << endl;

  // INITIALIZE ROS
  ros::init(argc, argv, "map_utility_server");
  
  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
  ros::NodeHandle pnh("~");

  //NUA TODO 1
  //ros::MultiThreadedSpinner spinner(2);
  //ros::AsyncSpinner spinner(0);
  //spinner.start();

  // INITIALIZE TRANSFORM LISTENER
  tf::TransformListener* listener = new tf::TransformListener;

  // INITIALIZE AND SET WORLD PARAMETERS
  string world_frame_name, map_name, map_frame_name, sensor_pc2_msg_name, sensor_pc2_direction, sensor_laser_msg_name;
  double map_resolution, map_server_dt, sensor_pc2_min_range, sensor_pc2_max_range, sensor_pc2_max_yaw, sensor_pc2_max_pitch;
  int skip_cnt_reset_sensor_range;
  float sensor_laser_max_range;
  double bbx_x_max, bbx_x_min, bbx_y_max, bbx_y_min, bbx_z_max, bbx_z_min;
  double crop_x_max, crop_x_min, crop_y_max, crop_y_min, crop_z_max, crop_z_min;
  bool filter_ground, local_map_flag, dynamic_flag;
  double filter_ground_threshold;

  pnh.param<string>("world_frame_name", world_frame_name, "");
  pnh.param<string>("map_name", map_name, "");
  pnh.param<string>("map_frame_name", map_frame_name, "");
  pnh.param("map_resolution", map_resolution, 0.0);
  pnh.param("map_server_dt", map_server_dt, 0.0);
  pnh.param("local_map_flag", local_map_flag, false);
  pnh.param("dynamic_flag", dynamic_flag, false);
  pnh.param("skip_cnt_reset_sensor_range", skip_cnt_reset_sensor_range, 0);
  pnh.param<string>("sensor_pc2_msg_name", sensor_pc2_msg_name, "");
  pnh.param<string>("sensor_pc2_direction", sensor_pc2_direction, "");
  pnh.param("sensor_pc2_min_range", sensor_pc2_min_range, 0.0);
  pnh.param("sensor_pc2_max_range", sensor_pc2_max_range, 0.0);
  pnh.param("sensor_pc2_max_yaw", sensor_pc2_max_yaw, 0.0);
  pnh.param("sensor_pc2_max_pitch", sensor_pc2_max_pitch, 0.0);
  pnh.param<string>("sensor_laser_msg_name", sensor_laser_msg_name, "");
  pnh.param<float>("sensor_laser_max_range", sensor_laser_max_range, 0.0);
  pnh.param("crop_x_max", crop_x_max, 0.0);
  pnh.param("crop_x_min", crop_x_min, 0.0);
  pnh.param("crop_y_max", crop_y_max, 0.0);
  pnh.param("crop_y_min", crop_y_min, 0.0);
  pnh.param("crop_z_max", crop_z_max, 0.0);
  pnh.param("crop_z_min", crop_z_min, 0.0);
  pnh.param("bbx_x_max", bbx_x_max, 0.0);
  pnh.param("bbx_x_min", bbx_x_min, 0.0);
  pnh.param("bbx_y_max", bbx_y_max, 0.0);
  pnh.param("bbx_y_min", bbx_y_min, 0.0);
  pnh.param("bbx_z_max", bbx_z_max, 0.0);
  pnh.param("bbx_z_min", bbx_z_min, 0.0);
  pnh.param("filter_ground", filter_ground, false);
  pnh.param("filter_ground_threshold", filter_ground_threshold, 0.0);
  
  // INITIALIZE AND SET MAP PARAMETERS
  MapUtility mu(nh);
  mu.setWorldFrameName(world_frame_name);
  mu.setMapName(map_name);
  mu.setMapFrameName(map_frame_name);
  mu.setMapResolution(map_resolution);
  mu.setMapServerDt(map_server_dt);
  mu.setLocalMapFlag(local_map_flag);
  mu.setDynamicFlag(dynamic_flag);
  mu.setSkipCntResetSensorRange(skip_cnt_reset_sensor_range);
  mu.setSensorPC2MsgName(sensor_pc2_msg_name);
  mu.setSensorPC2Direction(sensor_pc2_direction);
  mu.setSensorPC2MinRange(sensor_pc2_min_range);
  mu.setSensorPC2MaxRange(sensor_pc2_max_range);
  mu.setSensorPC2MaxYaw(sensor_pc2_max_yaw);
  mu.setSensorPC2MaxPitch(sensor_pc2_max_pitch);
  mu.setSensorLaserMsgName(sensor_laser_msg_name);
  mu.setSensorLaserMaxRange(sensor_laser_max_range);
  mu.setCropXMax(crop_x_max);
  mu.setCropXMin(crop_x_min);
  mu.setCropYMax(crop_y_max);
  mu.setCropYMin(crop_y_min);
  mu.setCropZMax(crop_z_max);
  mu.setCropZMin(crop_z_min);
  mu.setBBxXMax(bbx_x_max);
  mu.setBBxXMin(bbx_x_min);
  mu.setBBxYMax(bbx_y_max);
  mu.setBBxYMin(bbx_y_min);
  mu.setBBxZMax(bbx_z_max);
  mu.setBBxZMin(bbx_z_min);
  mu.setFilterGround(filter_ground);
  mu.setFilterGroundThreshold(filter_ground_threshold);

  mu.resetMap();
  mu.setSensorRangeQuery();
  mu.constructCameraSensorRange();
  mu.constructLaserSensorRange();

  // SUBSCRIBE TO THE ODOMETRY DATA
  //ros::Subscriber sub_odom = nh.subscribe(map_origin_msg_name, 1000, &MapUtility::mapPoseCallback, &mu);
  //ros::Subscriber sub_odom = nh.subscribe(map_frame_name, 10, &MapUtility::mapOdometryCallback, &mu);

  // SUBSCRIBE TO THE OCCUPANCY SENSOR DATA (PointCloud2)
  ros::Subscriber sub_pc2 = nh.subscribe(sensor_pc2_msg_name, 1000, &MapUtility::pc2Callback, &mu);

  // SUBSCRIBE TO THE OCCUPANCY SENSOR DATA (LaserScan)
  ros::Subscriber sub_laser = nh.subscribe(sensor_laser_msg_name, 1000, &MapUtility::laserCallback, &mu);

  // NUA TODO 1: MAP SERVICE
  //ros::ServiceServer service_reset_map_utility = nh.advertiseService("reset_map_utility", &MapUtility::reset_map_utility, &mu);

  ros::Duration(1.0).sleep();

  // MAP SERVER LOOP
  ros::Timer timer = nh.createTimer(ros::Duration(map_server_dt), &MapUtility::mainCallback, &mu);

  //spinner.spin();
  //ros::waitForShutdown();
  ros::spin();

  return 0;
}