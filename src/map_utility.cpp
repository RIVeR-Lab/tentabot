// LAST UPDATE: 2021.03.20
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

// --CUSTOM LIBRARIES--
#include "map_utility.h"

double MapUtility::randdouble(double from, double to)
{
  double f = (double)rand() / RAND_MAX;
  return from + f * (to - from);  
}

void MapUtility::createColorOcTree(double new_oct_resolution, sensor_msgs::PointCloud& new_pc, vector<int> color_RGB)
{
  //delete this -> octmap;
  octmap = new ColorOcTree(new_oct_resolution);

  int pcd_size = new_pc.points.size();

  for(int i = 0; i < pcd_size; i++)
  {
    octmap -> updateNode(new_pc.points[i].x, new_pc.points[i].y, new_pc.points[i].z, true);
    octmap -> setNodeColor(octmap -> coordToKey(new_pc.points[i].x, new_pc.points[i].y, new_pc.points[i].z), color_RGB[0], color_RGB[1], color_RGB[2]);
  }
  fillOctmapMsgFromOctmap();
}

vector<geometry_msgs::Point32> MapUtility::extract_pc_from_node_center(geometry_msgs::Point center)
{
  double half_vdim = 0.5 * oct_resolution;
  if (pc_resolution_factor == 0)
  {
    ROS_WARN("MapUtility::extract_pc_from_node_center -> pc_resolution_factor has not set. It is set to 1.");
    pc_resolution_factor = 1;
  }
  double pc_resolution = oct_resolution / pc_resolution_factor;
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

string MapUtility::createFileName()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
  std::string str(buffer);

  return str;
}

MapUtility::MapUtility()
{
  map_name = "";
  frame_name = "";
  x_range.clear();
  y_range.clear();
  z_range.clear();
  oct_resolution = 0;
  pc_resolution_factor = 0;
  max_occupancy_belief_value = 0;
  octmap = new ColorOcTree(0.5);
  history_size = 500000;
}

MapUtility::MapUtility(NodeHandle& nh, 
                       vector<double> new_x_range, 
                       vector<double> new_y_range, 
                       vector<double> new_z_range, 
                       double new_oct_resolution, 
                       int new_pc_resolution_factor, 
                       string new_frame_name, 
                       string new_map_name)
{
  map_name = new_map_name;
  frame_name = new_frame_name;
  x_range = new_x_range;
  y_range = new_y_range;
  z_range = new_z_range;
  oct_resolution = new_oct_resolution;
  pc_resolution_factor = new_pc_resolution_factor;
  max_occupancy_belief_value = 100;
  octmap = new ColorOcTree(new_oct_resolution);
  fillOctmapMsgFromOctmap();
  history_size = 500000;
  octmap_msg_pub = nh.advertise<octomap_msgs::Octomap>("Octmap_"+new_map_name, 100);
  recent_pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("RecentPC_"+new_map_name, 100);
  history_pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("HistoryPC_"+new_map_name, 100);
  local_pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("LocalPC_"+new_map_name, 100);
}

MapUtility::MapUtility(NodeHandle& nh, 
                       vector<double> new_x_range, 
                       vector<double> new_y_range, 
                       vector<double> new_z_range, 
                       double new_oct_resolution, 
                       int new_pc_resolution_factor, 
                       string new_frame_name, 
                       string new_map_name, 
                       sensor_msgs::PointCloud& new_pc)
{
  map_name = new_map_name;
  frame_name = new_frame_name;
  x_range = new_x_range;
  y_range = new_y_range;
  z_range = new_z_range;
  oct_resolution = new_oct_resolution;
  pc_resolution_factor = new_pc_resolution_factor;
  max_occupancy_belief_value = 100;
  history_size = 500000;
  createColorOcTree(new_oct_resolution, new_pc);
  fillOctmapMsgFromOctmap();
  fillRecentPCMsgFromOctmapByResolutionFactor();
  octmap_msg_pub = nh.advertise<octomap_msgs::Octomap>("Octmap_"+new_map_name, 100);
  recent_pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("RecentPC_"+new_map_name, 100);
  history_pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("HistoryPC_"+new_map_name, 100);
  local_pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("LocalPC_"+new_map_name, 100);
}

MapUtility::MapUtility(NodeHandle& nh, double new_oct_resolution, string new_frame_name, string new_map_name)
{
  map_name = new_map_name;
  frame_name = new_frame_name;
  oct_resolution = new_oct_resolution;
  max_occupancy_belief_value = 100;
  history_size = 500000;
  octmap = new ColorOcTree(new_oct_resolution);
  fillOctmapMsgFromOctmap();
  octmap_msg_pub = nh.advertise<octomap_msgs::Octomap>("Octmap_"+new_map_name, 100);
  recent_pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("RecentPC_"+new_map_name, 100);
  history_pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("HistoryPC_"+new_map_name, 100);
  local_pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("LocalPC_"+new_map_name, 100);
}

MapUtility::MapUtility(const MapUtility& mu)
{
  map_name = mu.map_name;
  frame_name = mu.frame_name;
  x_range = mu.x_range;
  y_range = mu.y_range;
  z_range = mu.z_range;
  oct_resolution = mu.oct_resolution;
  pc_resolution_factor = mu.pc_resolution_factor;
  max_occupancy_belief_value = mu.max_occupancy_belief_value;
  history_size = mu.history_size;    
  octmap = mu.octmap;
  octmap_msg = mu.octmap_msg;
  recent_pc_msg = mu.recent_pc_msg;
  history_pc_msg = mu.history_pc_msg;
  local_pc_msg = mu.local_pc_msg;
  octmap_msg_pub = mu.octmap_msg_pub;
  recent_pc_msg_pub = mu.recent_pc_msg_pub;
  history_pc_msg_pub = mu.history_pc_msg_pub;
  local_pc_msg_pub = mu.local_pc_msg_pub;
}

MapUtility::~MapUtility()
{
  //ROS_INFO( "Calling Destructor for MapUtility..." );
  delete (this -> octmap);
}

MapUtility& MapUtility::operator = (const MapUtility& mu) 
{ 
  map_name = mu.map_name;
  frame_name = mu.frame_name;
  x_range = mu.x_range;
  y_range = mu.y_range;
  z_range = mu.z_range;
  oct_resolution = mu.oct_resolution;
  pc_resolution_factor = mu.pc_resolution_factor;
  max_occupancy_belief_value = mu.max_occupancy_belief_value;
  history_size = mu.history_size;    
  octmap = mu.octmap;
  octmap_msg = mu.octmap_msg;
  recent_pc_msg = mu.recent_pc_msg;
  history_pc_msg = mu.history_pc_msg;
  local_pc_msg = mu.local_pc_msg;
  octmap_msg_pub = mu.octmap_msg_pub;
  recent_pc_msg_pub = mu.recent_pc_msg_pub;
  history_pc_msg_pub = mu.history_pc_msg_pub;
  local_pc_msg_pub = mu.local_pc_msg_pub;
}

string MapUtility::getMapName()
{
  return map_name;
}

string MapUtility::getFrameName()
{
  return frame_name;
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

double MapUtility::getOctResolution()
{
  return oct_resolution;
}

int MapUtility::getPCResolutionFactor()
{
  return pc_resolution_factor;
}

double MapUtility::getMaxOccupancyBeliefValue()
{
  return max_occupancy_belief_value;
}

int MapUtility::getHistorySize()
{
  return history_size;
}

octomap::ColorOcTree* MapUtility::getOctmap()
{
  return octmap;
}

octomap_msgs::Octomap& MapUtility::getOctmapMsg()
{
  return octmap_msg;
}

sensor_msgs::PointCloud& MapUtility::getRecentPCMsg()
{
  return recent_pc_msg;
}

sensor_msgs::PointCloud& MapUtility::getHistoryPCMsg()
{
  return history_pc_msg;
}

sensor_msgs::PointCloud& MapUtility::getLocalPCMsg()
{
  return local_pc_msg;
}

ros::Publisher MapUtility::getOctmapMsgPub()
{
  return octmap_msg_pub;
}

void MapUtility::setMapName(string new_map_name)
{
  map_name = new_map_name;
}

void MapUtility::setFrameName(string new_frame_name)
{
  frame_name = new_frame_name;
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

void MapUtility::setOctResolution(double new_oct_resolution)
{
  oct_resolution = new_oct_resolution;
}

void MapUtility::setPCResolutionFactor(int new_pc_resolution_factor)
{
  pc_resolution_factor = new_pc_resolution_factor;
}

void MapUtility::setMaxOccupancyBeliefValue(double new_max_occupancy_belief_value)
{
  max_occupancy_belief_value = new_max_occupancy_belief_value;
}

void MapUtility::setHistorySize(int new_history_size)
{
  history_size = new_history_size;
}

void MapUtility::setRecentPCMsg(sensor_msgs::PointCloud& new_recent_pc_msg)
{
  recent_pc_msg = new_recent_pc_msg;
}

void MapUtility::setHistoryPCMsg(sensor_msgs::PointCloud& new_history_pc_msg)
{
  history_pc_msg.header = new_history_pc_msg.header;
  history_pc_msg.channels = new_history_pc_msg.channels;
  history_pc_msg.points = new_history_pc_msg.points;
}

void MapUtility::setLocalPCMsg(sensor_msgs::PointCloud& new_local_pc_msg)
{
  local_pc_msg.header = new_local_pc_msg.header;
  local_pc_msg.channels = new_local_pc_msg.channels;
  local_pc_msg.points = new_local_pc_msg.points;
}

void MapUtility::fillOctmap(sensor_msgs::PointCloud pc_msg)
{
  octmap -> clear();

  for(int i = 0; i < pc_msg.points.size(); i++)
  {
    octmap -> updateNode(pc_msg.points[i].x, pc_msg.points[i].y, pc_msg.points[i].z, true);
  }
}

void MapUtility::fillOctmapFromRecentPCMsg()
{
  octmap -> clear();

  for(int i = 0; i < recent_pc_msg.points.size(); i++)
  {
    octmap -> updateNode(recent_pc_msg.points[i].x, recent_pc_msg.points[i].y, recent_pc_msg.points[i].z, true);
  }
}

void MapUtility::fillOctmapMsgFromOctmap()
{
  octmap_msg.header.frame_id = frame_name;
  octmap_msg.binary = false;
  octmap_msg.id = map_name;
  octmap_msg.resolution = oct_resolution;
  octomap_msgs::fullMapToMsg(*octmap, octmap_msg);
}

void MapUtility::fillOctmapMsgFromOctmap(string new_frame_name)
{
  octmap_msg.header.frame_id = new_frame_name;
  octmap_msg.binary = false;
  octmap_msg.id = map_name;
  octmap_msg.resolution = oct_resolution;
  octomap_msgs::fullMapToMsg(*octmap, octmap_msg);
}

void MapUtility::fillRecentPCMsgFromOctmap()
{
  recent_pc_msg.points.clear();
  
  for(octomap::ColorOcTree::iterator it = octmap -> begin(); it != octmap -> end(); ++it)
  {
    geometry_msgs::Point32 op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    recent_pc_msg.points.push_back(op);
  }
  recent_pc_msg.header.frame_id = frame_name;
}

void MapUtility::fillRecentPCMsgFromOctmapByResolutionFactor()
{
  recent_pc_msg.points.clear();
  
  for(octomap::ColorOcTree::iterator it = octmap -> begin(); it != octmap -> end(); ++it)
  {
    geometry_msgs::Point op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    vector<geometry_msgs::Point32> opc = extract_pc_from_node_center(op);
    for(int i = 0; i < opc.size(); i++)
    {
      recent_pc_msg.points.push_back(opc[i]);
    }
  }
  recent_pc_msg.header.frame_id = frame_name;
}

void MapUtility::fillHistoryPCMsgFromOctmap()
{
  history_pc_msg.points.clear();
  
  for(octomap::ColorOcTree::iterator it = octmap -> begin(); it != octmap -> end(); ++it)
  {
    geometry_msgs::Point32 op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    history_pc_msg.points.push_back(op);
  }
  history_pc_msg.header.frame_id = frame_name;
}

void MapUtility::fillLocalPCMsgFromOctmap()
{
  local_pc_msg.points.clear();
  
  for(octomap::ColorOcTree::iterator it = octmap -> begin(); it != octmap -> end(); ++it)
  {
    geometry_msgs::Point32 op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    local_pc_msg.points.push_back(op);
  }
  local_pc_msg.header.frame_id = frame_name;
}

void MapUtility::addOctmap(sensor_msgs::PointCloud pc_msg)
{
  for(int i = 0; i < pc_msg.points.size(); i++)
  {
    octmap -> updateNode(pc_msg.points[i].x, pc_msg.points[i].y, pc_msg.points[i].z, true);
  }
}

void MapUtility::addOctmapFromRecentPCMsg()
{
  for(int i = 0; i < recent_pc_msg.points.size(); i++)
  {
    octmap -> updateNode(recent_pc_msg.points[i].x, recent_pc_msg.points[i].y, recent_pc_msg.points[i].z, true);
  }
  fillOctmapMsgFromOctmap();
}

void MapUtility::addRecentPCMsg(sensor_msgs::PointCloud& new_recent_pc_msg)
{
  recent_pc_msg.header = new_recent_pc_msg.header;
  recent_pc_msg.channels = new_recent_pc_msg.channels;
  recent_pc_msg.points.insert(end(recent_pc_msg.points), begin(new_recent_pc_msg.points), end(new_recent_pc_msg.points));
}

void MapUtility::addHistoryPCMsg(sensor_msgs::PointCloud& new_history_pc_msg)
{
  history_pc_msg.header = new_history_pc_msg.header;
  history_pc_msg.channels = new_history_pc_msg.channels;

  history_pc_msg.points.insert(history_pc_msg.points.end(), new_history_pc_msg.points.begin(), new_history_pc_msg.points.end());

  if(history_pc_msg.points.size() > history_size)
  {
    int delta = history_pc_msg.points.size() - history_size;
    history_pc_msg.points.erase(history_pc_msg.points.begin(), history_pc_msg.points.begin() + delta);
  }
  //ROS_INFO_STREAM("MapUtility::addHistoryPCData ADDED............");
}

void MapUtility::addLocalPCMsg(sensor_msgs::PointCloud& new_local_pc_msg)
{
  local_pc_msg.header = new_local_pc_msg.header;
  local_pc_msg.channels = new_local_pc_msg.channels;
  local_pc_msg.points.insert(end(local_pc_msg.points), begin(new_local_pc_msg.points), end(new_local_pc_msg.points));
}

void MapUtility::addLocalPCMsg(geometry_msgs::Point32 new_point)
{
  local_pc_msg.points.push_back(new_point);
}

void MapUtility::clearRecentPCMsg()
{
  recent_pc_msg.points.clear();
}

void MapUtility::clearHistoryPCMsg()
{
  history_pc_msg.points.clear();
}

void MapUtility::clearLocalPCMsg()
{
  local_pc_msg.points.clear();
}

bool MapUtility::isOccupied(double x, double y, double z)
{
  OcTreeNode* node = octmap -> search(x, y, z);
  if(node)
  {
    return octmap -> isNodeOccupied(node);
  }
  else
  {
    return false;
  }
}

bool MapUtility::isOccupied(geometry_msgs::Point po)
{
  OcTreeNode* node = octmap -> search(po.x, po.y, po.z);
  if(node)
  {
    return octmap -> isNodeOccupied(node);
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

  double free_rad = 2 * oct_resolution;

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
  double free_rad = 2 * oct_resolution;

  for(int i = 0; i < goal.size(); i++)
  {
    if( isInCube(po, goal[i].position, free_rad) )
    {
      return true;
    }
  }
  return false;
}

void MapUtility::addStaticObstacleByResolutionFactor2RecentPCMsg(geometry_msgs::Point po)
{
  vector<geometry_msgs::Point32> opc = extract_pc_from_node_center(po);
  for(int i = 0; i < opc.size(); i++)
  {
    recent_pc_msg.points.push_back(opc[i]);
  }
  recent_pc_msg.header.frame_id = frame_name;
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

  if( constraint_flag && (isOccupied(x, y, z) || isOccupiedByGoal(x, y, z, goal) || isInCube(po, robot_center, robot_free_rad + oct_resolution)) )
  {
    return false;
  }
  else
  {
    octmap -> updateNode(x, y, z, true);
    octmap -> setNodeColor(octmap -> coordToKey(x, y, z), color_RGB[0], color_RGB[1], color_RGB[2]);
    fillOctmapMsgFromOctmap();
    addStaticObstacleByResolutionFactor2RecentPCMsg(po);
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
  octmap -> clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(x_range[0], x_range[1]);
    rp.position.y = randdouble(y_range[0], y_range[1]);
    rp.position.z = randdouble(z_range[0], z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + oct_resolution)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(x_range[0], x_range[1]);
      rp.position.y = randdouble(y_range[0], y_range[1]);
      rp.position.z = randdouble(z_range[0], z_range[1]);
    }

    octmap -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    octmap -> setNodeColor(octmap -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);  
  }
  fillOctmapMsgFromOctmap();
  fillRecentPCMsgFromOctmap();
}

void MapUtility::createRandomStaticObstacleMap(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  octmap -> clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
    rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
    rp.position.z = randdouble(new_z_range[0], new_z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + oct_resolution)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
      rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
      rp.position.z = randdouble(new_z_range[0], new_z_range[1]);
    }

    octmap -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    octmap -> setNodeColor(octmap -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);
  }
  fillOctmapMsgFromOctmap();
  fillRecentPCMsgFromOctmap();
}

void MapUtility::addRandomStaticObstacle(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
    rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
    rp.position.z = randdouble(new_z_range[0], new_z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + oct_resolution)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
      rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
      rp.position.z = randdouble(new_z_range[0], new_z_range[1]);
    }

    octmap -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    octmap -> setNodeColor(octmap -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);
    addStaticObstacleByResolutionFactor2RecentPCMsg(rp.position);
  }
  fillOctmapMsgFromOctmap();
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

void MapUtility::publishOctmapMsg()
{
  octmap_msg.header.seq++;
  octmap_msg.header.stamp = ros::Time(0);
  octmap_msg_pub.publish(octmap_msg);
}

void MapUtility::publishRecentPCMsg()
{
  recent_pc_msg.header.seq++;
  recent_pc_msg.header.stamp = ros::Time(0);
  recent_pc_msg_pub.publish(recent_pc_msg);
}

void MapUtility::publishHistoryPCMsg()
{
  history_pc_msg.header.seq++;
  history_pc_msg.header.stamp = ros::Time(0);
  history_pc_msg_pub.publish(history_pc_msg);
}

void MapUtility::publishLocalPCMsg()
{
  local_pc_msg.header.seq++;
  local_pc_msg.header.stamp = ros::Time(0);
  local_pc_msg_pub.publish(local_pc_msg);
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
    map_file << "frame_name," + frame_name + "\n";
    map_file << "x_range," + to_string(x_range[0]) + "," + to_string(x_range[1]) + "\n";
    map_file << "y_range," + to_string(y_range[0]) + "," + to_string(y_range[1]) + "\n";
    map_file << "z_range," + to_string(z_range[0]) + "," + to_string(z_range[1]) + "\n";
    map_file << "oct_resolution," + to_string(oct_resolution) + "\n";
    map_file << "pc_resolution_factor," + to_string(pc_resolution_factor) + "\n";
    map_file << "max_occupancy_belief_value," + to_string(max_occupancy_belief_value) + "\n";

    map_file << "map,\n";

    for(octomap::ColorOcTree::iterator it = octmap -> begin(); it != octmap -> end(); ++it)
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

void MapUtility::loadMap(string filename)
{
  ifstream map_file("/home/akmandor/catkin_ws/src/tentabot/data/" + filename + ".csv");

  if( map_file.is_open() )
  {
    ROS_INFO_STREAM("" << filename << " is loading from the file...");

    octmap -> clear();

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