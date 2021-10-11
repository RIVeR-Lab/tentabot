// LAST UPDATE: 2021.10.05
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
// 
// TODO:
// - Laser is cleaned each timestep but assumed cylinder hull, could be more detailed as cumulative convex hulls.

// --CUSTOM LIBRARIES--
#include "map_utility.h"

MapUtility::MapUtility()
{
  tflistener = new tf::TransformListener;
  world_frame_name = "";
  map_name = "";
  map_frame_name = "";
  sensor_pc2_msg_name = "";
  sensor_pc2_direction = "";
  sensor_pc2_frame_name = "";
  sensor_pc2_pose.position.x = 0;
  sensor_pc2_pose.position.y = 0;
  sensor_pc2_pose.position.z = 0;
  sensor_pc2_pose.orientation.x = 0;
  sensor_pc2_pose.orientation.y = 0;
  sensor_pc2_pose.orientation.z = 0;
  sensor_pc2_pose.orientation.w = 0;
  map_pose = sensor_pc2_pose;
  x_range.clear();
  y_range.clear();
  z_range.clear();
  map_resolution = 0;
  pc_resolution_scale = 0;
  max_occupancy_belief_value = 0;
  map_server_dt = 0;
  skip_cnt = 0;
  oct_msg.header.frame_id = world_frame_name;
  pc_msg.header.frame_id = world_frame_name;
  pc2_msg.header.frame_id = world_frame_name;
}

MapUtility::MapUtility(NodeHandle& nh)
{
  tflistener = new tf::TransformListener;

  world_frame_name = "";
  map_name = "";
  map_frame_name = "";
  sensor_pc2_msg_name = "";
  sensor_pc2_direction = "";
  sensor_pc2_frame_name = "";
  
  sensor_pc2_pose.position.x = 0;
  sensor_pc2_pose.position.y = 0;
  sensor_pc2_pose.position.z = 0;
  sensor_pc2_pose.orientation.x = 0;
  sensor_pc2_pose.orientation.y = 0;
  sensor_pc2_pose.orientation.z = 0;
  sensor_pc2_pose.orientation.w = 0;
  map_pose = sensor_pc2_pose;
  
  x_range.clear();
  y_range.clear();
  z_range.clear();
  
  map_resolution = 0;
  pc_resolution_scale = 0;
  max_occupancy_belief_value = 100;
  map_server_dt = 0;
  skip_cnt = 0;
  
  oct_msg.header.frame_id = world_frame_name;
  pc_msg.header.frame_id = world_frame_name;
  pc2_msg.header.frame_id = world_frame_name;

  oct_msg_pub = nh.advertise<octomap_msgs::Octomap>("server_octomap", 1000);
  pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("server_PC", 1);
  pc2_msg_pub = nh.advertise<sensor_msgs::PointCloud2>("server_PC2", 1);
  debug_array_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("server_debug_array", 1);
  debug_visu_pub = nh.advertise<visualization_msgs::Marker>("server_debug", 1);
}

MapUtility::MapUtility(const MapUtility& mu)
{
  tflistener = mu.tflistener;
  world_frame_name = mu.world_frame_name;
  map_name = mu.map_name;
  map_frame_name = mu.map_frame_name;
  sensor_pc2_msg_name = mu.sensor_pc2_msg_name;
  sensor_pc2_direction = mu.sensor_pc2_direction;
  sensor_pc2_frame_name = mu.sensor_pc2_frame_name;
  measured_sensor_pc2_pose = mu.measured_sensor_pc2_pose;
  measured_map_pose = mu.measured_map_pose;
  sensor_pc2_pose = mu.sensor_pc2_pose;
  map_pose = mu.map_pose;
  x_range = mu.x_range;
  y_range = mu.y_range;
  z_range = mu.z_range;
  bbx_x_max = mu.bbx_x_max;
  bbx_x_min = mu.bbx_x_min;
  bbx_y_max = mu.bbx_y_max;
  bbx_y_min = mu.bbx_y_min;
  bbx_z_max = mu.bbx_z_max;
  bbx_z_min = mu.bbx_z_min;
  crop_x_max = mu.crop_x_max;
  crop_x_min = mu.crop_x_min;
  crop_y_max = mu.crop_y_max;
  crop_y_min = mu.crop_y_min;
  crop_z_max = mu.crop_z_max;
  crop_z_min = mu.crop_z_min;
  map_resolution = mu.map_resolution;
  pc_resolution_scale = mu.pc_resolution_scale;
  max_occupancy_belief_value = mu.max_occupancy_belief_value;
  map_server_dt = mu.map_server_dt;
  skip_cnt = mu.skip_cnt;
  oct = mu.oct;
  oct_msg = mu.oct_msg;
  oct_pc = mu.oct_pc;
  measured_pc_msg = mu.measured_pc_msg;
  measured_pc2_msg = mu.measured_pc2_msg;
  pc_msg = mu.pc_msg;
  pc2_msg = mu.pc2_msg;
  debug_array_visu = mu.debug_array_visu;
  debug_visu = mu.debug_visu;
  oct_msg_pub = mu.oct_msg_pub;
  pc_msg_pub = mu.pc_msg_pub;
  pc2_msg_pub = mu.pc2_msg_pub;
  debug_array_visu_pub = mu.debug_array_visu_pub;
  debug_visu_pub = mu.debug_visu_pub;
}

MapUtility::~MapUtility()
{
  //ROS_INFO( "Calling Destructor for MapUtility..." );
  delete[] tflistener;
  //delete oct;
}

MapUtility& MapUtility::operator = (const MapUtility& mu) 
{
  tflistener = mu.tflistener;
  world_frame_name = mu.world_frame_name;
  map_name = mu.map_name;
  map_frame_name = mu.map_frame_name;
  sensor_pc2_msg_name = mu.sensor_pc2_msg_name;
  sensor_pc2_direction = mu.sensor_pc2_direction;
  sensor_pc2_frame_name = mu.sensor_pc2_frame_name;
  measured_sensor_pc2_pose = mu.measured_sensor_pc2_pose;
  measured_map_pose = mu.measured_map_pose;
  sensor_pc2_pose = mu.sensor_pc2_pose;
  map_pose = mu.map_pose;
  x_range = mu.x_range;
  y_range = mu.y_range;
  z_range = mu.z_range;
  bbx_x_max = mu.bbx_x_max;
  bbx_x_min = mu.bbx_x_min;
  bbx_y_max = mu.bbx_y_max;
  bbx_y_min = mu.bbx_y_min;
  bbx_z_max = mu.bbx_z_max;
  bbx_z_min = mu.bbx_z_min;
  crop_x_max = mu.crop_x_max;
  crop_x_min = mu.crop_x_min;
  crop_y_max = mu.crop_y_max;
  crop_y_min = mu.crop_y_min;
  crop_z_max = mu.crop_z_max;
  crop_z_min = mu.crop_z_min;
  map_resolution = mu.map_resolution;
  pc_resolution_scale = mu.pc_resolution_scale;
  max_occupancy_belief_value = mu.max_occupancy_belief_value;
  map_server_dt = mu.map_server_dt;
  skip_cnt = mu.skip_cnt;
  oct = mu.oct;
  oct_msg = mu.oct_msg;
  oct_pc = mu.oct_pc;
  measured_pc_msg = mu.measured_pc_msg;
  measured_pc2_msg = mu.measured_pc2_msg;
  pc_msg = mu.pc_msg;
  pc2_msg = mu.pc2_msg;
  debug_array_visu = mu.debug_array_visu;
  debug_visu = mu.debug_visu;
  oct_msg_pub = mu.oct_msg_pub;
  pc_msg_pub = mu.pc_msg_pub;
  pc2_msg_pub = mu.pc2_msg_pub;
  debug_array_visu_pub = mu.debug_array_visu_pub;
  debug_visu_pub = mu.debug_visu_pub;
  return *this;
}

string MapUtility::getWorldFrameName()
{
  return world_frame_name;
}

string MapUtility::getMapName()
{
  return map_name;
}

string MapUtility::getMapFrameName()
{
  return map_frame_name;
}

string MapUtility::getSensorPC2MsgName()
{
  return sensor_pc2_msg_name;
}

string MapUtility::getSensorPC2Direction()
{
  return sensor_pc2_direction;
}

string MapUtility::getSensorPC2FrameName()
{
  return sensor_pc2_frame_name;
}

double MapUtility::getSensorPC2MinRange()
{
  return sensor_pc2_min_range;
}

double MapUtility::getSensorPC2MaxRange()
{
  return sensor_pc2_max_range;
}

double MapUtility::getSensorPC2MaxYaw()
{
  return sensor_pc2_max_yaw;
}

double MapUtility::getSensorPC2MaxPitch()
{
  return sensor_pc2_max_pitch;
}

string MapUtility::getSensorLaserMsgName()
{
  return sensor_laser_msg_name;
}

float MapUtility::getSensorLaserMaxRange()
{
  return sensor_laser_max_range;
}

geometry_msgs::Pose MapUtility::getMeasuredSensorPC2Pose()
{
  return measured_sensor_pc2_pose;
}

geometry_msgs::Pose MapUtility::getSensorPC2Pose()
{
  return sensor_pc2_pose;
}

geometry_msgs::Pose MapUtility::getMeasuredMapPose()
{
  return measured_map_pose;
}

geometry_msgs::Pose MapUtility::getMapPose()
{
  return map_pose;
}

vector<double> MapUtility::getXRange()
{
  return x_range;
}

vector<double> MapUtility::getYRange()
{
  return y_range;
}

vector<double> MapUtility::getZRange()
{
  return z_range;
}

double MapUtility::getBBxXMax()
{
  return bbx_x_max;
}

double MapUtility::getBBxXMin()
{
  return bbx_x_min;
}

double MapUtility::getBBxYMax()
{
  return bbx_y_max;
}

double MapUtility::getBBxYMin()
{
  return bbx_y_min;
}

double MapUtility::getBBxZMax()
{
  return bbx_z_max;
}

double MapUtility::getBBxZMin()
{
  return bbx_z_min;
}

double MapUtility::getCropXMax()
{
  return crop_x_max;
}

double MapUtility::getCropXMin()
{
  return crop_x_min;
}

double MapUtility::getCropYMax()
{
  return crop_y_max;
}

double MapUtility::getCropYMin()
{
  return crop_y_min;
}

double MapUtility::getCropZMax()
{
  return crop_z_max;
}

double MapUtility::getCropZMin()
{
  return crop_z_min;
}

bool MapUtility::getFilterGround()
{
  return filter_ground;
}

double MapUtility::getFilterGroundThreshold()
{
  return filter_ground_threshold;
}

double MapUtility::getMapResolution()
{
  return map_resolution;
}

double MapUtility::getPCResolutionScale()
{
  return pc_resolution_scale;
}

double MapUtility::getMaxOccupancyBeliefValue()
{
  return max_occupancy_belief_value;
}

double MapUtility::getMapServerDt()
{
  return map_server_dt;
}

bool MapUtility::getLocalMapFlag()
{
  return local_map_flag;
}

bool MapUtility::getDynamicFlag()
{
  return dynamic_flag;
}

int MapUtility::getSkipCntResetSensorRange()
{
  return skip_cnt_reset_sensor_range;
}

shared_ptr<octomap::ColorOcTree> MapUtility::getOct()
{
  return oct;
}

octomap_msgs::Octomap& MapUtility::getOctMsg()
{
  return oct_msg;
}

octomap::Pointcloud& MapUtility::getOctPC()
{
  return oct_pc;
}

sensor_msgs::PointCloud& MapUtility::getMeasuredPCMsg()
{
  return measured_pc_msg;
}

sensor_msgs::PointCloud2& MapUtility::getMeasuredPC2Msg()
{
  return measured_pc2_msg;
}

sensor_msgs::PointCloud& MapUtility::getPCMsg()
{
  return pc_msg;
}

sensor_msgs::PointCloud2& MapUtility::getPC2Msg()
{
  return pc2_msg;
}

ros::Publisher MapUtility::getOctMsgPub()
{
  return oct_msg_pub;
}

ros::Publisher MapUtility::getPCMsgPub()
{
  return pc_msg_pub;
}

ros::Publisher MapUtility::getPC2MsgPub()
{
  return pc2_msg_pub;
}

void MapUtility::setWorldFrameName(string new_world_frame_name)
{
  world_frame_name = new_world_frame_name;
}

void MapUtility::setMapName(string new_map_name)
{
  map_name = new_map_name;
}

void MapUtility::setMapFrameName(string new_map_frame_name)
{
  map_frame_name = new_map_frame_name;
}

void MapUtility::setSensorPC2MsgName(string new_sensor_pc2_msg_name)
{
  sensor_pc2_msg_name = new_sensor_pc2_msg_name;
}

void MapUtility::setSensorPC2Direction(string new_sensor_pc2_direction)
{
  sensor_pc2_direction = new_sensor_pc2_direction;
}

void MapUtility::setSensorPC2FrameName(string new_sensor_pc2_frame_name)
{
  sensor_pc2_frame_name = new_sensor_pc2_frame_name;
}

void MapUtility::setSensorPC2MinRange(double new_sensor_pc2_min_range)
{
  sensor_pc2_min_range = new_sensor_pc2_min_range;
}

void MapUtility::setSensorPC2MaxRange(double new_sensor_pc2_max_range)
{
  sensor_pc2_max_range = new_sensor_pc2_max_range;
}

void MapUtility::setSensorPC2MaxYaw(double new_sensor_pc2_max_yaw)
{
  sensor_pc2_max_yaw = new_sensor_pc2_max_yaw;
}

void MapUtility::setSensorPC2MaxPitch(double new_sensor_pc2_max_pitch)
{
  sensor_pc2_max_pitch = new_sensor_pc2_max_pitch;
}

void MapUtility::setSensorLaserMsgName(string new_sensor_laser_msg_name)
{
  sensor_laser_msg_name = new_sensor_laser_msg_name;
}

void MapUtility::setSensorLaserMaxRange(float new_sensor_laser_max_range)
{
  sensor_laser_max_range = new_sensor_laser_max_range;
}

void MapUtility::setMeasuredSensorPC2Pose(geometry_msgs::Pose new_measured_sensor_pc2_pose)
{
  measured_sensor_pc2_pose = new_measured_sensor_pc2_pose;
}

void MapUtility::setSensorPC2Pose(geometry_msgs::Pose new_sensor_pc2_pose)
{
  sensor_pc2_pose = new_sensor_pc2_pose;
}

void MapUtility::setMeasuredMapPose(geometry_msgs::Pose new_measured_map_pose)
{
  measured_map_pose = new_measured_map_pose;
}

void MapUtility::setMapPose(geometry_msgs::Pose new_map_pose)
{
  map_pose = new_map_pose;
}

void MapUtility::setXRange(double x0, double x1)
{
  x_range[0] = x0;
  x_range[1] = x1;
}

void MapUtility::setYRange(double y0, double y1)
{
  y_range[0] = y0;
  y_range[1] = y1;
}

void MapUtility::setZRange(double z0, double z1)
{
  z_range[0] = z0;
  z_range[1] = z1;
}

void MapUtility::setBBxXMax(double new_bbx_x_max)
{
  bbx_x_max = new_bbx_x_max;
}

void MapUtility::setBBxXMin(double new_bbx_x_min)
{
  bbx_x_min = new_bbx_x_min;
}

void MapUtility::setBBxYMax(double new_bbx_y_max)
{
  bbx_y_max = new_bbx_y_max;
}

void MapUtility::setBBxYMin(double new_bbx_y_min)
{
  bbx_y_min = new_bbx_y_min;
}

void MapUtility::setBBxZMax(double new_bbx_z_max)
{
  bbx_z_max = new_bbx_z_max;
}

void MapUtility::setBBxZMin(double new_bbx_z_min)
{
  bbx_z_min = new_bbx_z_min;
}

void MapUtility::setCropXMax(double new_crop_x_max)
{
  crop_x_max = new_crop_x_max;
}

void MapUtility::setCropXMin(double new_crop_x_min)
{
  crop_x_min = new_crop_x_min;
}

void MapUtility::setCropYMax(double new_crop_y_max)
{
  crop_y_max = new_crop_y_max;
}

void MapUtility::setCropYMin(double new_crop_y_min)
{
  crop_y_min = new_crop_y_min;
}

void MapUtility::setCropZMax(double new_crop_z_max)
{
  crop_z_max = new_crop_z_max;
}

void MapUtility::setCropZMin(double new_crop_z_min)
{
  crop_z_min = new_crop_z_min;
}

void MapUtility::setFilterGround(bool new_filter_ground)
{
  filter_ground =  new_filter_ground;
}

void MapUtility::setFilterGroundThreshold(double new_filter_ground_threshold)
{
  filter_ground_threshold =  new_filter_ground_threshold;
}

void MapUtility::setMapResolution(double new_map_resolution)
{
  map_resolution = new_map_resolution;

  //oct = new ColorOcTree(map_resolution);
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution);
}

void MapUtility::setPCResolutionScale(double new_pc_resolution_scale)
{
  pc_resolution_scale = new_pc_resolution_scale;
}

void MapUtility::setMaxOccupancyBeliefValue(double new_max_occupancy_belief_value)
{
  max_occupancy_belief_value = new_max_occupancy_belief_value;
}

void MapUtility::setMapServerDt(double new_map_server_dt)
{
  map_server_dt = new_map_server_dt;
}

void MapUtility::setLocalMapFlag(bool new_local_map_flag)
{
  local_map_flag = new_local_map_flag;
}

void MapUtility::setDynamicFlag(bool new_dynamic_flag)
{
  dynamic_flag = new_dynamic_flag;
}

void MapUtility::setSkipCntResetSensorRange(double new_skip_cnt_reset_sensor_range)
{
  skip_cnt_reset_sensor_range = new_skip_cnt_reset_sensor_range;
}

void MapUtility::setOctPC(octomap::Pointcloud& new_oct_pc)
{
  oct_pc = new_oct_pc;
}

void MapUtility::setMeasuredPCMsg(sensor_msgs::PointCloud& new_measured_pc_msg)
{
  measured_pc_msg = new_measured_pc_msg;
}

void MapUtility::setMeasuredPC2Msg(sensor_msgs::PointCloud2& new_measured_pc2_msg)
{
  measured_pc2_msg = new_measured_pc2_msg;
}

void MapUtility::setPCMsg(sensor_msgs::PointCloud& new_pc_msg)
{
  pc_msg = new_pc_msg;
}

void MapUtility::setPC2Msg(sensor_msgs::PointCloud2& new_pc2_msg)
{
  pc2_msg = new_pc2_msg;
}

void MapUtility::resetMap()
{
  //oct = new ColorOcTree(map_resolution);
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution);

  fillOctMsgFromOct();
}

void MapUtility::transformPoint(string frame_from,
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

void MapUtility::createColorOcTree(double new_map_resolution, sensor_msgs::PointCloud& new_pc, vector<int> color_RGB)
{
  //oct = new ColorOcTree(new_map_resolution);
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution);

  int pcd_size = new_pc.points.size();

  for(int i = 0; i < pcd_size; i++)
  {
    oct -> updateNode(new_pc.points[i].x, new_pc.points[i].y, new_pc.points[i].z, true);
    oct -> setNodeColor(oct -> coordToKey(new_pc.points[i].x, new_pc.points[i].y, new_pc.points[i].z), color_RGB[0], color_RGB[1], color_RGB[2]);
  }
  fillOctMsgFromOct();
}

vector<geometry_msgs::Point32> MapUtility::extract_pc_from_node_center(geometry_msgs::Point center)
{
  double half_vdim = 0.5 * map_resolution;
  if (pc_resolution_scale <= 0)
  {
    ROS_WARN("MapUtility::extract_pc_from_node_center -> pc_resolution_scale has not set. It is set to 1.");
    pc_resolution_scale = 1;
  }
  double pc_resolution = pc_resolution_scale * map_resolution;
  vector<geometry_msgs::Point32> opc;
  geometry_msgs::Point minp;
  geometry_msgs::Point maxp;
  minp.x = center.x - half_vdim;                    // (-x, -y, -z)
  minp.y = center.y - half_vdim;
  minp.z = center.z - half_vdim;
  maxp.x = center.x + half_vdim;                    // (+x, +y, +z)
  maxp.y = center.y + half_vdim;
  maxp.z = center.z + half_vdim;

  for(double i = minp.x; i < maxp.x; i += pc_resolution)
  {
    for(double j = minp.y; j < maxp.y; j += pc_resolution)
    {
      for(double k = minp.z; k < maxp.z; k += pc_resolution)
      {
        geometry_msgs::Point32 newp;
        newp.x = i;
        newp.y = j;
        newp.z = k;
        opc.push_back(newp);

        if(i == minp.x)
        {
          geometry_msgs::Point32 newpx;
          newpx.x = maxp.x;
          newpx.y = j;
          newpx.z = k;
          opc.push_back(newpx);
        }
        
        if(j == minp.y)
        {
          geometry_msgs::Point32 newpy;
          newpy.x = i;
          newpy.y = maxp.y;
          newpy.z = k;
          opc.push_back(newpy);
        }

        if(k == minp.z)
        {
          geometry_msgs::Point32 newpz;
          newpz.x = i;
          newpz.y = j;
          newpz.z = maxp.z;
          opc.push_back(newpz);
        }
        
        if(i == minp.x && j == minp.y)
        {
          geometry_msgs::Point32 newpxy;
          newpxy.x = maxp.x;
          newpxy.y = maxp.y;
          newpxy.z = k;
          opc.push_back(newpxy);
        }      

        if(i == minp.x && k == minp.z)
        {
          geometry_msgs::Point32 newpxz;
          newpxz.x = maxp.x;
          newpxz.y = j;
          newpxz.z = maxp.z;
          opc.push_back(newpxz);
        }
        
        if(j == minp.y && k == minp.z)
        {
          geometry_msgs::Point32 newpyz;
          newpyz.x = i;
          newpyz.y = maxp.y;
          newpyz.z = maxp.z;
          opc.push_back(newpyz);
        }
        
        if(i == minp.x && j == minp.y && k == minp.z)
        {
          geometry_msgs::Point32 newpxyz;
          newpxyz.x = maxp.x;
          newpxyz.y = maxp.y;
          newpxyz.z = maxp.z;
          opc.push_back(newpxyz);
        }
      }
    }
  }
  return opc;
}

void MapUtility::fillOct(sensor_msgs::PointCloud& pc_msg)
{
  oct -> clear();

  for(int i = 0; i < pc_msg.points.size(); i++)
  {
    oct -> updateNode(pc_msg.points[i].x, pc_msg.points[i].y, pc_msg.points[i].z, true);
  }
}

void MapUtility::fillOctFromMeasuredPCMsg()
{
  oct -> clear();

  for(int i = 0; i < measured_pc_msg.points.size(); i++)
  {
    oct -> updateNode(measured_pc_msg.points[i].x, measured_pc_msg.points[i].y, measured_pc_msg.points[i].z, true);
  }
}

void MapUtility::fillOctMsgFromOct()
{
  oct_msg.data.clear();
  oct_msg.header.frame_id = world_frame_name;
  oct_msg.binary = false;
  oct_msg.id = map_name;
  oct_msg.resolution = map_resolution;
  octomap_msgs::fullMapToMsg(*oct, oct_msg);
}

void MapUtility::fillPCMsgFromOct()
{
  pc_msg.points.clear();
  
  for(octomap::ColorOcTree::iterator it = oct -> begin(); it != oct -> end(); ++it)
  {
    geometry_msgs::Point32 op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    pc_msg.points.push_back(op);
  }
  pc_msg.header.frame_id = world_frame_name;
}

void MapUtility::fillPCMsgFromOctByResolutionScale()
{
  pc_msg.points.clear();
  
  for(octomap::ColorOcTree::iterator it = oct -> begin(); it != oct -> end(); ++it)
  {
    geometry_msgs::Point op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    vector<geometry_msgs::Point32> opc = extract_pc_from_node_center(op);
    for(int i = 0; i < opc.size(); i++)
    {
      pc_msg.points.push_back(opc[i]);
    }
  }
  pc_msg.header.frame_id = world_frame_name;
}

void MapUtility::fillDebugArrayVisu(vector<tf::Vector3> v)
{
  debug_array_visu.markers.clear();

  for(int i = 0; i < v.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.ns = "point" + to_string(i);
    marker.id = i;
    marker.header.frame_id = world_frame_name;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.position.x = v[i].x();
    marker.pose.position.y = v[i].y();
    marker.pose.position.z = v[i].z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    debug_array_visu.markers.push_back(marker); 
  }
}

void MapUtility::fillDebugVisu(vector<tf::Vector3> v)
{
  debug_visu.points.clear();

  debug_visu.ns = "points";
  debug_visu.id = 1;
  debug_visu.header.frame_id = world_frame_name;
  debug_visu.type = visualization_msgs::Marker::POINTS;
  debug_visu.action = visualization_msgs::Marker::ADD;
  debug_visu.pose.orientation.w = 1.0;
  debug_visu.scale.x = 0.04;
  debug_visu.scale.y = 0.04;
  debug_visu.scale.z = 0.04;
  debug_visu.color.r = 1.0;
  debug_visu.color.g = 1.0;
  debug_visu.color.b = 0.0;
  debug_visu.color.a = 1.0;

  debug_visu.points.clear();
  for(int i = 0; i < v.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = v[i].x();
    p.y = v[i].y();
    p.z = v[i].z();

    debug_visu.points.push_back(p); 
  }
}

void MapUtility::insertToOctFromOctPC(octomap::point3d sensor_pc2_origin)
{
  oct -> insertPointCloud(oct_pc, sensor_pc2_origin, sensor_pc2_max_range, false, true);
}

void MapUtility::addToOct(sensor_msgs::PointCloud& pc_msg)
{
  for(int i = 0; i < pc_msg.points.size(); i++)
  {
    oct -> updateNode(pc_msg.points[i].x, pc_msg.points[i].y, pc_msg.points[i].z, true);
  }
}

void MapUtility::addToOctFromMeasuredPCMsg()
{
  for(int i = 0; i < measured_pc_msg.points.size(); i++)
  {
    oct -> updateNode(measured_pc_msg.points[i].x, measured_pc_msg.points[i].y, measured_pc_msg.points[i].z, true);
  }
  fillOctMsgFromOct();
}

void MapUtility::addToPCMsg(sensor_msgs::PointCloud& new_pc_msg)
{
  pc_msg.header = new_pc_msg.header;
  pc_msg.channels = new_pc_msg.channels;
  pc_msg.points.insert(end(pc_msg.points), begin(new_pc_msg.points), end(new_pc_msg.points));
}

void MapUtility::addToPCMsg(geometry_msgs::Point32 new_point)
{
  pc_msg.points.push_back(new_point);
}

void MapUtility::clearMeasuredPCMsg()
{
  measured_pc_msg.points.clear();
}

void MapUtility::clearPCMsg()
{
  pc_msg.points.clear();
}

bool MapUtility::isOccupied(double x, double y, double z)
{
  OcTreeNode* node = oct -> search(x, y, z);
  if(node)
  {
    return oct -> isNodeOccupied(node);
  }
  else
  {
    return false;
  }
}

bool MapUtility::isOccupied(geometry_msgs::Point po)
{
  OcTreeNode* node = oct -> search(po.x, po.y, po.z);
  if(node)
  {
    return oct -> isNodeOccupied(node);
  }
  else
  {
    return false;
  }
}

void MapUtility::setSensorRangeQuery(float query_point_resolution)
{
  // SET THE QUERY POINT (BOX)
  fcl::Box<float> query(query_point_resolution, query_point_resolution, query_point_resolution);
  query_sharedPtr = std::make_shared< fcl::Box<float> > (query);
}

void MapUtility::constructCameraSensorRange()
{
  tf::Transform aligner;
  aligner.setIdentity();
  tf::Quaternion aligner_q(0, 0, 0, 1);
  if (sensor_pc2_direction == "y")
  {
    //cout << "MapUtility::constructCameraSensorRange -> Sensor is in y direction!" << endl;
    aligner_q.setRPY(0, 0, 0.5*PI);
  }
  else if (sensor_pc2_direction == "z")
  {
    //cout << "MapUtility::constructCameraSensorRange -> Sensor is in z direction!" << endl;
    aligner_q.setRPY(0, -0.5*PI, 0);
  }

  aligner.setRotation(aligner_q);
  
  // SET THE CAMERA SENSOR RANGE AS POLYTOPE
  tf::Vector3 vert;

  vert.setValue(  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v0( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v1( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v2( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v3( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v4( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v5( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v6( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v7( vert.x(), vert.y(), vert.z() );

  std::vector< fcl::Vector3<float> > vertices;
  auto vertices_sharedPtr = std::make_shared< std::vector< fcl::Vector3<float> > > (vertices);

  vertices_sharedPtr.get()->push_back(v0);
  vertices_sharedPtr.get()->push_back(v1);
  vertices_sharedPtr.get()->push_back(v2);
  vertices_sharedPtr.get()->push_back(v3);
  vertices_sharedPtr.get()->push_back(v4);
  vertices_sharedPtr.get()->push_back(v5);
  vertices_sharedPtr.get()->push_back(v6);
  vertices_sharedPtr.get()->push_back(v7);

  vector<int> f0 = {4,0,1,2,3};
  vector<int> f1 = {4,4,5,1,0};
  vector<int> f2 = {4,7,6,5,4};
  vector<int> f3 = {4,3,2,6,7};
  vector<int> f4 = {4,4,0,3,7};
  vector<int> f5 = {4,1,5,6,2};

  vector<int> faces;
  auto faces_sharedPtr = std::make_shared< std::vector<int> > (faces);

  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f0.begin(), f0.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f1.begin(), f1.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f2.begin(), f2.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f3.begin(), f3.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f4.begin(), f4.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f5.begin(), f5.end());

  int num_faces = 6;
  bool throw_if_invalid = true;

  fcl::Convex<float> sensor_pc2_range_hull(vertices_sharedPtr, num_faces, faces_sharedPtr, throw_if_invalid);

  sensor_pc2_range_sharedPtr = std::make_shared< fcl::Convex<float> > (sensor_pc2_range_hull);
}

void MapUtility::constructLaserSensorRange(float sensor_laser_z_range)
{
  // SET THE LASER SENSOR COVERAGE AS CYLINDER
  fcl::Cylinder<float> sensor_laser_hull(sensor_laser_max_range, sensor_laser_z_range);
  sensor_laser_range_sharedPtr = std::make_shared< fcl::Cylinder<float> > (sensor_laser_hull);
}

bool MapUtility::isInSensorCameraRange(tf::Vector3 query, bool on_flag)
{
  if (on_flag)
  {
    // Set the query collision obj
    fcl::Vector3f query_T(query.x(), query.y(), query.z());
    fcl::Transform3f query_tr = fcl::Transform3f::Identity();
    query_tr.translation() = query_T;
    fcl::CollisionObjectf* query_collision_obj = new fcl::CollisionObjectf(query_sharedPtr, query_tr);

    // Set the camera sensor range collision obj
    fcl::CollisionObjectf* sensor_pc2_range_collision_obj = new fcl::CollisionObjectf(sensor_pc2_range_sharedPtr, fcl::Transform3f::Identity());

    // Perform collision tests
    sensor_pc2_range_result.clear();

    fcl::collide(sensor_pc2_range_collision_obj, query_collision_obj, sensor_pc2_range_request, sensor_pc2_range_result);

    delete query_collision_obj;
    delete sensor_pc2_range_collision_obj;
    
    return sensor_pc2_range_result.isCollision();
  }
  else
  {
    return false;
  }
}

bool MapUtility::isInSensorLaserRange(tf::Vector3 query, bool on_flag)
{
  if (on_flag)
  {
    // Set the query collision obj
    fcl::Vector3f query_T(query.x(), query.y(), query.z());
    fcl::Transform3f query_tr = fcl::Transform3f::Identity();
    query_tr.translation() = query_T;
    fcl::CollisionObjectf* query_collision_obj = new fcl::CollisionObjectf(query_sharedPtr, query_tr);

    // Set the laser sensor range collision obj
    fcl::CollisionObjectf* laser_sensor_collision_obj = new fcl::CollisionObjectf(sensor_laser_range_sharedPtr, fcl::Transform3f::Identity());

    // Perform collision tests
    sensor_laser_range_result.clear();

    fcl::collide(laser_sensor_collision_obj, query_collision_obj, sensor_laser_range_request, sensor_laser_range_result);

    delete query_collision_obj;
    delete laser_sensor_collision_obj;
    
    return sensor_laser_range_result.isCollision();
  }
  else
  {
    return false;
  }
}

bool MapUtility::isInCube(geometry_msgs::Point po, geometry_msgs::Point center, double rad)
{
  return (po.x >= center.x - rad) && (po.x <= center.x + rad) && (po.y >= center.y - rad) && (po.y <= center.y + rad) && (po.z >= center.z - rad) && (po.z <= center.z + rad);
}

bool MapUtility::isOccupiedByGoal(double x, double y, double z, vector<geometry_msgs::Pose> goal)
{
  geometry_msgs::Point po;
  po.x = x;
  po.y = y;
  po.z = z;

  double free_rad = 2 * map_resolution;

  for(int i = 0; i < goal.size(); i++)
  {
    if( isInCube(po, goal[i].position, free_rad) )
    {
      return true;
    }
  }
  return false;
}

bool MapUtility::isOccupiedByGoal(geometry_msgs::Point po, vector<geometry_msgs::Pose> goal)
{
  double free_rad = 2 * map_resolution;

  for(int i = 0; i < goal.size(); i++)
  {
    if( isInCube(po, goal[i].position, free_rad) )
    {
      return true;
    }
  }
  return false;
}

void MapUtility::addStaticObstacleByResolutionScale2PCMsg(geometry_msgs::Point po)
{
  vector<geometry_msgs::Point32> opc = extract_pc_from_node_center(po);
  for(int i = 0; i < opc.size(); i++)
  {
    pc_msg.points.push_back(opc[i]);
  }
  pc_msg.header.frame_id = world_frame_name;
}

bool MapUtility::addStaticObstacle(double x, 
                                   double y, 
                                   double z, 
                                   bool constraint_flag, 
                                   vector<geometry_msgs::Pose> goal, 
                                   geometry_msgs::Point robot_center, 
                                   double robot_free_rad, 
                                   vector<int> color_RGB)
{
  geometry_msgs::Point po;
  po.x = x;
  po.y = y;
  po.z = z;

  if( constraint_flag && (isOccupied(x, y, z) || isOccupiedByGoal(x, y, z, goal) || isInCube(po, robot_center, robot_free_rad + map_resolution)) )
  {
    return false;
  }
  else
  {
    oct -> updateNode(x, y, z, true);
    oct -> setNodeColor(oct -> coordToKey(x, y, z), color_RGB[0], color_RGB[1], color_RGB[2]);
    fillOctMsgFromOct();
    addStaticObstacleByResolutionScale2PCMsg(po);
    return true;
  }
}

vector<bool> MapUtility::addStaticObstacle(sensor_msgs::PointCloud& pcd, 
                                           bool constraint_flag, 
                                           vector<geometry_msgs::Pose> goal, 
                                           geometry_msgs::Point robot_center, 
                                           double robot_free_rad, 
                                           vector<int> color_RGB)
{
  vector<bool> pc_add_result;
      
  int pcd_size = pcd.points.size();

  for(int i = 0; i < pcd_size; i++)
  {
    pc_add_result.push_back( addStaticObstacle(pcd.points[i].x, pcd.points[i].y, pcd.points[i].z, constraint_flag, goal, robot_center, robot_free_rad, color_RGB) );
  }
  return pc_add_result;
}

void MapUtility::createRandomStaticObstacleMap(int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  oct -> clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(x_range[0], x_range[1]);
    rp.position.y = randdouble(y_range[0], y_range[1]);
    rp.position.z = randdouble(z_range[0], z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + map_resolution)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(x_range[0], x_range[1]);
      rp.position.y = randdouble(y_range[0], y_range[1]);
      rp.position.z = randdouble(z_range[0], z_range[1]);
    }

    oct -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    oct -> setNodeColor(oct -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);  
  }
  fillOctMsgFromOct();
  fillPCMsgFromOct();
}

void MapUtility::createRandomStaticObstacleMap(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  oct -> clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
    rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
    rp.position.z = randdouble(new_z_range[0], new_z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + map_resolution)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
      rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
      rp.position.z = randdouble(new_z_range[0], new_z_range[1]);
    }

    oct -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    oct -> setNodeColor(oct -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);
  }
  fillOctMsgFromOct();
  fillPCMsgFromOct();
}

void MapUtility::addRandomStaticObstacle(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
    rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
    rp.position.z = randdouble(new_z_range[0], new_z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + map_resolution)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
      rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
      rp.position.z = randdouble(new_z_range[0], new_z_range[1]);
    }

    oct -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    oct -> setNodeColor(oct -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);
    addStaticObstacleByResolutionScale2PCMsg(rp.position);
  }
  fillOctMsgFromOct();
}

void MapUtility::createRandomMapSet(string mapset_name, int map_cnt, int map_occupancy_count)
{
  vector<double> goal_x_range;
  goal_x_range.push_back(x_range[0] + 4);
  goal_x_range.push_back(x_range[1] - 4);

  vector<double> goal_y_range;
  goal_y_range.push_back(y_range[0] + 4);
  goal_y_range.push_back(y_range[1] - 4);

  vector<double> goal_z_range;
  goal_z_range.push_back(z_range[0] + 4);
  goal_z_range.push_back(z_range[1] - 4);
  
  for (int i = 0; i < map_cnt; i++)
  {
    createRandomStaticObstacleMap(map_occupancy_count);
    saveMap("mapset/" + mapset_name + "/map" + to_string(i));
  }
}

void MapUtility::crop(sensor_msgs::PointCloud2& cloud_in, octomap::point3d lowerBound, octomap::point3d upperBound, octomap::Pointcloud& cloud_out, bool keep_in)
{
  cloud_out.reserve(cloud_in.data.size() / cloud_in.point_step);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in, "z");

  float min_x, min_y, min_z;
  float max_x, max_y, max_z;
  float x,y,z;

  min_x = lowerBound(0); min_y = lowerBound(1); min_z = lowerBound(2);
  max_x = upperBound(0); max_y = upperBound(1); max_z = upperBound(2);

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
    {
      if( (*iter_x >= min_x) && (*iter_x <= max_x) &&
          (*iter_y >= min_y) && (*iter_y <= max_y) &&
          (*iter_z >= min_z) && (*iter_z <= max_z) )
      {
        if(keep_in)
        {
          cloud_out.push_back(*iter_x, *iter_y, *iter_z);
        }
      }
      else
      {
        if(!keep_in)
        {
          cloud_out.push_back(*iter_x, *iter_y, *iter_z);
        }
      }
    }
  }
}

void MapUtility::cropOctPCFromPC2Msg(octomap::point3d lowerBound, octomap::point3d upperBound, bool keep_in)
{
  oct_pc.clear();

  /*
  if (pc2_msg.data.size() > 0 && laser_pc2_msg.data.size() > 0)
  {
    oct_pc.reserve( (pc2_msg.data.size() + laser_pc2_msg.data.size()) / (pc2_msg.point_step + laser_pc2_msg.point_step) );
  }
  else if(pc2_msg.data.size() > 0)
  {
    oct_pc.reserve(pc2_msg.data.size() / pc2_msg.point_step);
  }
  else if(laser_pc2_msg.data.size() > 0)
  {
    oct_pc.reserve(laser_pc2_msg.data.size() / laser_pc2_msg.point_step);
  }
  */

  if(pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(pc2_msg.data.size() / pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        op_wrt_world.setValue(*iter_x, *iter_y, *iter_z);
        op_wrt_map = transform_map_wrt_world.inverse() * op_wrt_world;

        if( isInBBx(op_wrt_map, lowerBound, upperBound) )
        {
          if(keep_in)
          {
            oct_pc.push_back(*iter_x, *iter_y, *iter_z);

            if(filter_ground)
            {
              if (*iter_z > filter_ground_threshold)
              {
                oct_pc.push_back(*iter_x, *iter_y, *iter_z);
              }
            }
            else
            {
              oct_pc.push_back(*iter_x, *iter_y, *iter_z);
            }
          }
        }
        else
        {
          if(!keep_in)
          { 
            if(filter_ground)
            {
              if (*iter_z > filter_ground_threshold)
              {
                oct_pc.push_back(*iter_x, *iter_y, *iter_z);
              }
            }
            else
            {
              oct_pc.push_back(*iter_x, *iter_y, *iter_z);
            }
          }
        }
      }
    }
  }

  if(laser_pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(laser_pc2_msg.data.size() / laser_pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(laser_pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(laser_pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(laser_pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        op_wrt_world.setValue(*iter_x, *iter_y, *iter_z);
        op_wrt_map = transform_map_wrt_world.inverse() * op_wrt_world;

        if( isInBBx(op_wrt_map, lowerBound, upperBound) )
        {
          if(keep_in)
          {
            oct_pc.push_back(*iter_x, *iter_y, *iter_z);
          }
        }
        else
        {
          if(!keep_in)
          {
            oct_pc.push_back(*iter_x, *iter_y, *iter_z);
          }
        }
      }
    }
  }
}

void MapUtility::updateOctPC()
{
  oct_pc.clear();

  if(pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(pc2_msg.data.size() / pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        oct_pc.push_back(*iter_x, *iter_y, *iter_z);
      }
    }
  }

  if(laser_pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(laser_pc2_msg.data.size() / laser_pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(laser_pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(laser_pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(laser_pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        oct_pc.push_back(*iter_x, *iter_y, *iter_z);
      }
    }
  }
}

void MapUtility::pointCloud2ToOctomap(const sensor_msgs::PointCloud2& cloud_pc2, octomap::Pointcloud& cloud_octomap)
{
  cloud_octomap.reserve(cloud_pc2.data.size() / cloud_pc2.point_step);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_pc2, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
    {
      cloud_octomap.push_back(*iter_x, *iter_y, *iter_z);
    }
  }
}

void MapUtility::publishOctMsg()
{
  oct_msg.header.frame_id = world_frame_name;
  //oct_msg.header.seq++;
  //oct_msg.header.stamp = ros::Time(0);
  oct_msg.header.stamp = ros::Time::now();
  oct_msg_pub.publish(oct_msg);
}

void MapUtility::publishPCMsg()
{
  pc_msg.header.frame_id = world_frame_name;
  pc_msg.header.seq++;
  //pc_msg.header.stamp = ros::Time(0);
  pc_msg.header.stamp = ros::Time::now();
  pc_msg_pub.publish(pc_msg);
}

void MapUtility::publishPC2Msg()
{
  pc2_msg.header.frame_id = world_frame_name;
  pc2_msg.header.seq++;
  //pc2_msg.header.stamp = ros::Time(0);
  pc2_msg.header.stamp = ros::Time::now();
  pc2_msg_pub.publish(pc2_msg);
}

void MapUtility::publishDebugArrayVisu()
{
  for (int i = 0; i < debug_array_visu.markers.size(); ++i)
  {
    debug_array_visu.markers[i].header.seq++;
    //debug_array_visu.markers[i].header.stamp = ros::Time(0);
    debug_array_visu.markers[i].header.stamp = ros::Time::now();
  }
  debug_array_visu_pub.publish(debug_array_visu);
}

void MapUtility::publishDebugVisu()
{
  debug_visu.header.seq++;
  //debug_visu.header.stamp = ros::Time(0);
  debug_visu.header.stamp = ros::Time::now();
  debug_visu_pub.publish(debug_visu);
}

void MapUtility::printDataSize()
{
  cout << "" << endl;
  cout << "map_utility::printDataSize -> oct: " << oct -> size() << endl;
  cout << "map_utility::printDataSize -> oct_msg: " << oct_msg.data.size() << endl;
  cout << "map_utility::printDataSize -> oct_pc: " << oct_pc.size() << endl;
  //cout << "map_utility::printDataSize -> measured_pc_msg: " << measured_pc_msg.points.size() << endl;
  //cout << "map_utility::printDataSize -> measured_pc2_msg: " << measured_pc2_msg.data.size() << endl;
  //cout << "map_utility::printDataSize -> measured_laser_msg: " << measured_laser_msg.ranges.size() << endl;
  //cout << "map_utility::printDataSize -> pc_msg: " << pc_msg.points.size() << endl;
  cout << "map_utility::printDataSize -> pc2_msg: " << pc2_msg.data.size() << endl;
  cout << "map_utility::printDataSize -> laser_pc2_msg: " << laser_pc2_msg.data.size() << endl;
  cout << "" << endl;
}

void MapUtility::saveMap(string filename)
{
  if(filename == "")
  {
    filename = createFileName();
  }

  ofstream map_file;
  map_file.open ("/home/akmandor/catkin_ws/src/tentabot/data/" + filename + ".csv");

  if( map_file.is_open() )
  {
    map_file << "map_name," + map_name + "\n";
    map_file << "world_frame_name," + world_frame_name + "\n";
    map_file << "x_range," + to_string(x_range[0]) + "," + to_string(x_range[1]) + "\n";
    map_file << "y_range," + to_string(y_range[0]) + "," + to_string(y_range[1]) + "\n";
    map_file << "z_range," + to_string(z_range[0]) + "," + to_string(z_range[1]) + "\n";
    map_file << "map_resolution," + to_string(map_resolution) + "\n";
    map_file << "pc_resolution_scale," + to_string(pc_resolution_scale) + "\n";
    map_file << "max_occupancy_belief_value," + to_string(max_occupancy_belief_value) + "\n";

    map_file << "map,\n";

    for(octomap::ColorOcTree::iterator it = oct -> begin(); it != oct -> end(); ++it)
    {
      map_file << to_string(it.getCoordinate().x()) + "," + to_string(it.getCoordinate().y()) + "," + to_string(it.getCoordinate().z()) + "\n";
    }
    map_file.close();
  }
  else
  {
    ROS_WARN("MapUtility::saveMap -> Unable to open map file to save.");
  }
}

// NUA TODO: get the benchmark path
void MapUtility::loadMap(string filename)
{
  ifstream map_file("/home/akmandor/catkin_ws/src/tentabot/data/" + filename + ".csv");

  if( map_file.is_open() )
  {
    ROS_INFO_STREAM("" << filename << " is loading from the file...");

    oct -> clear();

    string line = "";
    bool map_flag = false;
    while( getline(map_file, line) )
    {
      vector<string> vec;
      boost::algorithm::split(vec, line, boost::is_any_of(","));

      if(vec[0] == "map")
      {
        map_flag = true;
        continue;
      }

      if(map_flag)
      {          
        addStaticObstacle( atof(vec[0].c_str()), atof(vec[1].c_str()), atof(vec[2].c_str()) );
      }
    }

    // Close the File
    map_file.close();
    cout << filename + " is loaded!" << endl;
  }
  else
  {
    ROS_WARN_STREAM("MapUtility::loadMap -> Unable to open " << filename << " file to load.");
  }
}

void MapUtility::mapPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  //cout << "MapUtility::mapPoseCallback -> Incoming data..." << endl;
  measured_map_pose = *msg;
}

void MapUtility::mapOdometryCallback(const nav_msgs::Odometry& msg)
{
  //cout << "MapUtility::mapOdometryCallback -> Incoming data..." << endl;
  measured_map_pose = msg.pose.pose;
}

void MapUtility::pc2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //cout << "MapUtility::pc2Callback -> Incoming data..." << endl;
  sensor_pc2_frame_name = msg -> header.frame_id;
  //cout << "MapUtility::pc2Callback -> sensor_pc2_frame_name: " << sensor_pc2_frame_name << endl;

  try
  {
    //tflistener -> waitForTransform(world_frame_name, msg -> header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tflistener -> lookupTransform(world_frame_name, msg -> header.frame_id, ros::Time(0), measured_transform_sensor_pc2_wrt_world);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("MapUtility::pc2Callback -> Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  // SET MEASURED SENSOR POSE
  measured_sensor_pc2_pose.position.x = measured_transform_sensor_pc2_wrt_world.getOrigin().x();
  measured_sensor_pc2_pose.position.y = measured_transform_sensor_pc2_wrt_world.getOrigin().y();
  measured_sensor_pc2_pose.position.z = measured_transform_sensor_pc2_wrt_world.getOrigin().z();
  tf::quaternionTFToMsg(measured_transform_sensor_pc2_wrt_world.getRotation(), measured_sensor_pc2_pose.orientation);

  // SET MEASURED PC2
  pcl_ros::transformPointCloud(world_frame_name, measured_transform_sensor_pc2_wrt_world, *msg, measured_pc2_msg);
}

void MapUtility::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //cout << "MapUtility::laserCallback -> Incoming data..." << endl;
  sensor_laser_frame_name = msg -> header.frame_id;
  //cout << "MapUtility::laserCallback -> sensor_laser_frame_name: " << sensor_laser_frame_name << endl;
  measured_laser_msg = *msg;
}

void MapUtility::update_states()
{
  //map_pose = measured_map_pose;
  sensor_pc2_pose = measured_sensor_pc2_pose;

  try
  {
    // scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment
    tflistener -> waitForTransform(world_frame_name, map_frame_name, ros::Time::now(), ros::Duration(map_server_dt));
    tflistener -> lookupTransform(world_frame_name, map_frame_name, ros::Time(0), transform_map_wrt_world);

    tflistener -> waitForTransform(world_frame_name, sensor_pc2_frame_name, ros::Time::now(), ros::Duration(map_server_dt));
    tflistener -> lookupTransform(world_frame_name, sensor_pc2_frame_name, ros::Time(0), transform_sensor_pc2_wrt_world);

    if (sensor_laser_frame_name != "")
    {
      tflistener -> waitForTransform(world_frame_name, sensor_laser_frame_name, ros::Time::now(), ros::Duration(map_server_dt));
      tflistener -> lookupTransform(world_frame_name, sensor_laser_frame_name, ros::Time(0), transform_sensor_laser_wrt_world);
      sensor_laser_projector.transformLaserScanToPointCloud(world_frame_name, measured_laser_msg, laser_pc2_msg, *tflistener);
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("MapUtility::update_states -> Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  pc2_msg = measured_pc2_msg;
}

void MapUtility::update_map()
{
  // DELETE MAP IN THE SENSOR RANGE
  tf::Vector3 query_wrt_world;
  tf::Vector3 query_wrt_map;
  tf::Vector3 query_wrt_sensor_pc2;
  tf::Vector3 query_wrt_sensor_laser;

  for(octomap::ColorOcTree::iterator it = oct -> begin(); it != oct -> end(); it++)
  {
    query_wrt_world.setValue(it.getX(), it.getY(), it.getZ());
    query_wrt_map = transform_map_wrt_world.inverse() * query_wrt_world;
    query_wrt_sensor_pc2 = transform_sensor_pc2_wrt_world.inverse() * query_wrt_world;
    query_wrt_sensor_laser = transform_sensor_laser_wrt_world.inverse() * query_wrt_world;

    if (  ( !isInBBx(query_wrt_map, bbx_x_min, bbx_x_max, bbx_y_min, bbx_y_max, bbx_z_min, bbx_z_max) ) ||
          ( dynamic_flag && skip_cnt == skip_cnt_reset_sensor_range && isInSensorCameraRange(query_wrt_sensor_pc2, sensor_pc2_msg_name != "") ) ||
          ( dynamic_flag && skip_cnt == skip_cnt_reset_sensor_range && isInSensorLaserRange(query_wrt_sensor_laser, sensor_laser_msg_name != "") ) )
    {
      oct -> deleteNode(it.getKey());
    }
  }

  // CROP RECENT POINTCLOUD2 DATA AND KEEP AS OCTOMAP PC
  octomap::point3d lowerBound(crop_x_min, crop_y_min, crop_z_min);
  octomap::point3d upperBound(crop_x_max, crop_y_max, crop_z_max);
  cropOctPCFromPC2Msg(lowerBound, upperBound, false);
  
  // INSERT RECENT OCTOMAP PC INTO THE MAP
  point3d sensor_pc2_origin(sensor_pc2_pose.position.x, sensor_pc2_pose.position.y, sensor_pc2_pose.position.z);
  insertToOctFromOctPC(sensor_pc2_origin);

  // UPDATE OCTOMAP MSG FROM THE MAP
  fillOctMsgFromOct();

  if (skip_cnt == skip_cnt_reset_sensor_range)
  {
    skip_cnt = 0;
  }
  else
  {
    skip_cnt++;
  }
}

bool MapUtility::reset_map_utility(tentabot::reset_map_utility::Request &req, tentabot::reset_map_utility::Response &res)
{
  cout << "MapUtility::reset_map_utility -> Map is NOT reset! parity: " << req.parity << endl;

  oct -> clear();
  fillOctMsgFromOct();

  cout << "MapUtility::reset_map_utility -> Map is reset!" << endl;

  res.success = true;
  return true;
}

void MapUtility::mainCallback(const ros::TimerEvent& e)
{
  if (!local_map_flag)
  {
    oct -> clear();
    fillOctMsgFromOct();
  }

  //printDataSize();
  
  update_states();

  update_map();

  publishOctMsg();
}