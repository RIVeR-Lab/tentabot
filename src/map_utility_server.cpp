// LAST UPDATE: 2022.03.23
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

  pnh.param<string>("/world_frame_name", world_frame_name, "");
  pnh.param<string>("/map_name", map_name, "");
  pnh.param<string>("/map_frame_name", map_frame_name, "");
  pnh.param("/map_resolution", map_resolution, 0.0);
  pnh.param("/map_server_dt", map_server_dt, 0.0);
  pnh.param("/local_map_flag", local_map_flag, false);
  pnh.param("/dynamic_flag", dynamic_flag, false);
  pnh.param("/skip_cnt_reset_sensor_range", skip_cnt_reset_sensor_range, 0);
  pnh.param<string>("/sensor_pc2_msg_name", sensor_pc2_msg_name, "");
  pnh.param<string>("/sensor_pc2_direction", sensor_pc2_direction, "");
  pnh.param("/sensor_pc2_min_range", sensor_pc2_min_range, 0.0);
  pnh.param("/sensor_pc2_max_range", sensor_pc2_max_range, 0.0);
  pnh.param("/sensor_pc2_max_yaw", sensor_pc2_max_yaw, 0.0);
  pnh.param("/sensor_pc2_max_pitch", sensor_pc2_max_pitch, 0.0);
  pnh.param<string>("/sensor_laser_msg_name", sensor_laser_msg_name, "");
  pnh.param<float>("/sensor_laser_max_range", sensor_laser_max_range, 0.0);
  pnh.param("/crop_x_max", crop_x_max, 0.0);
  pnh.param("/crop_x_min", crop_x_min, 0.0);
  pnh.param("/crop_y_max", crop_y_max, 0.0);
  pnh.param("/crop_y_min", crop_y_min, 0.0);
  pnh.param("/crop_z_max", crop_z_max, 0.0);
  pnh.param("/crop_z_min", crop_z_min, 0.0);
  pnh.param("/bbx_x_max", bbx_x_max, 0.0);
  pnh.param("/bbx_x_min", bbx_x_min, 0.0);
  pnh.param("/bbx_y_max", bbx_y_max, 0.0);
  pnh.param("/bbx_y_min", bbx_y_min, 0.0);
  pnh.param("/bbx_z_max", bbx_z_max, 0.0);
  pnh.param("/bbx_z_min", bbx_z_min, 0.0);
  pnh.param("/filter_ground", filter_ground, false);
  pnh.param("/filter_ground_threshold", filter_ground_threshold, 0.0);
  
  /*
  cout << "map_utility_server::main -> world_frame_name: " << world_frame_name << endl;
  cout << "map_utility_server::main -> map_name: " << map_name << endl;
  cout << "map_utility_server::main -> map_frame_name: " << map_frame_name << endl;
  cout << "map_utility_server::main -> map_resolution: " << map_resolution << endl;
  cout << "map_utility_server::main -> map_server_dt: " << map_server_dt << endl;
  cout << "map_utility_server::main -> local_map_flag: " << local_map_flag << endl;
  cout << "map_utility_server::main -> dynamic_flag: " << dynamic_flag << endl;
  cout << "map_utility_server::main -> skip_cnt_reset_sensor_range: " << skip_cnt_reset_sensor_range << endl;
  cout << "map_utility_server::main -> sensor_pc2_msg_name: " << sensor_pc2_msg_name << endl;
  cout << "map_utility_server::main -> sensor_pc2_direction: " << sensor_pc2_direction << endl;
  cout << "map_utility_server::main -> sensor_pc2_min_range: " << sensor_pc2_min_range << endl;
  cout << "map_utility_server::main -> sensor_pc2_max_range: " << sensor_pc2_max_range << endl;
  cout << "map_utility_server::main -> sensor_pc2_max_yaw: " << sensor_pc2_max_yaw << endl;
  cout << "map_utility_server::main -> sensor_pc2_max_pitch: " << sensor_pc2_max_pitch << endl;
  cout << "map_utility_server::main -> sensor_laser_msg_name: " << sensor_laser_msg_name << endl;
  cout << "map_utility_server::main -> sensor_laser_max_range: " << sensor_laser_max_range << endl;
  cout << "map_utility_server::main -> crop_x_max: " << crop_x_max << endl;
  cout << "map_utility_server::main -> crop_x_min: " << crop_x_min << endl;
  cout << "map_utility_server::main -> crop_y_max: " << crop_y_max << endl;
  cout << "map_utility_server::main -> crop_y_min: " << crop_y_min << endl;
  cout << "map_utility_server::main -> crop_z_max: " << crop_z_max << endl;
  cout << "map_utility_server::main -> crop_z_min: " << crop_z_min << endl;
  cout << "map_utility_server::main -> bbx_x_max: " << bbx_x_max << endl;
  cout << "map_utility_server::main -> bbx_x_min: " << bbx_x_min << endl;
  cout << "map_utility_server::main -> bbx_y_max: " << bbx_y_max << endl;
  cout << "map_utility_server::main -> bbx_y_min: " << bbx_y_min << endl;
  cout << "map_utility_server::main -> bbx_z_max: " << bbx_z_max << endl;
  cout << "map_utility_server::main -> bbx_z_min: " << bbx_z_min << endl;
  cout << "map_utility_server::main -> filter_ground: " << filter_ground << endl;
  cout << "map_utility_server::main -> filter_ground_threshold: " << filter_ground_threshold << endl;
  */

  // INITIALIZE AND SET MAP PARAMETERS
  MapUtility mu(nh, map_name, sensor_pc2_msg_name, sensor_laser_msg_name);
  mu.setWorldFrameName(world_frame_name);
  mu.setMapFrameName(map_frame_name);
  mu.setMapResolution(map_resolution);
  mu.setMapServerDt(map_server_dt);
  mu.setLocalMapFlag(local_map_flag);
  mu.setDynamicFlag(dynamic_flag);
  mu.setSkipCntResetSensorRange(skip_cnt_reset_sensor_range);
  mu.setSensorPC2Direction(sensor_pc2_direction);
  mu.setSensorPC2MinRange(sensor_pc2_min_range);
  mu.setSensorPC2MaxRange(sensor_pc2_max_range);
  mu.setSensorPC2MaxYaw(sensor_pc2_max_yaw);
  mu.setSensorPC2MaxPitch(sensor_pc2_max_pitch);
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

  ros::Duration(1.0).sleep();

  // MAP SERVER LOOP
  ros::Timer timer = nh.createTimer(ros::Duration(map_server_dt), &MapUtility::mainCallback, &mu);

  //spinner.spin();
  //ros::waitForShutdown();
  ros::spin();

  return 0;
}