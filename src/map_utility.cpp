// LAST UPDATE: 2019.08.04
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION:

// LOCAL LIBRARIES:
#include "map_utility.h"

double MapUtility::randdouble(double from, double to)
{
  double f = (double)rand() / RAND_MAX;
  return from + f * (to - from);  
}

void MapUtility::createColorOcTree(double new_oct_resolution, sensor_msgs::PointCloud pcd, vector<int> color_RGB)
{
  //delete this -> octmap;
  this -> octmap = new ColorOcTree(new_oct_resolution);

  int pcd_size = pcd.points.size();

  for(int i = 0; i < pcd_size; i++)
  {
    this -> octmap -> updateNode(pcd.points[i].x, pcd.points[i].y, pcd.points[i].z, true);
    this -> octmap -> setNodeColor(this -> octmap -> coordToKey(pcd.points[i].x, pcd.points[i].y, pcd.points[i].z), color_RGB[0], color_RGB[1], color_RGB[2]);
  }
}

vector<geometry_msgs::Point32> MapUtility::extract_pc_from_node_center(geometry_msgs::Point center)
{
  double half_vdim = 0.5 * this -> oct_resolution;
  double pc_resolution = this -> oct_resolution / this -> pc_resolution_factor;
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
  this -> map_name = "";
  this -> frame_name = "";
  this -> x_range.clear();
  this -> y_range.clear();
  this -> z_range.clear();
  this -> oct_resolution = 0;
  this -> pc_resolution_factor = 0;
  this -> max_occupancy_belief_value = 0;
  this -> octmap = new ColorOcTree(0.5);
  this -> history_size = 500000;
}

MapUtility::MapUtility(NodeHandle& nh, GoalUtility& new_goal_util, vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, double new_oct_resolution, int new_pc_resolution_factor, string new_frame_name, string new_map_name)
{
  this -> map_name = new_map_name;
  this -> frame_name = new_frame_name;
  this -> goal_util = new_goal_util;
  this -> x_range = new_x_range;
  this -> y_range = new_y_range;
  this -> z_range = new_z_range;
  this -> oct_resolution = new_oct_resolution;
  this -> pc_resolution_factor = new_pc_resolution_factor;
  this -> max_occupancy_belief_value = 100;
  this -> octmap = new ColorOcTree(new_oct_resolution);
  this -> history_size = 500000;
  this -> fillMapVisu();
  this -> octmap_visu_pub = nh.advertise<octomap_msgs::Octomap>("OcTree_"+new_map_name, 100);
  this -> pcdata_visu_pub = nh.advertise<sensor_msgs::PointCloud>("PC_"+new_map_name, 100);
  this -> hpcdata_visu_pub = nh.advertise<sensor_msgs::PointCloud>("HistoryPC_"+new_map_name, 100);
}

MapUtility::MapUtility(NodeHandle& nh, GoalUtility& new_goal_util, vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, double new_oct_resolution, int new_pc_resolution_factor, string new_frame_name, string new_map_name, sensor_msgs::PointCloud new_pcd)
{
  this -> map_name = new_map_name;
  this -> frame_name = new_frame_name;
  this -> goal_util = new_goal_util;
  this -> x_range = new_x_range;
  this -> y_range = new_y_range;
  this -> z_range = new_z_range;
  this -> oct_resolution = new_oct_resolution;
  this -> pc_resolution_factor = new_pc_resolution_factor;
  this -> max_occupancy_belief_value = 100;
  this -> createColorOcTree(new_oct_resolution, new_pcd);
  this -> fillMapVisu();
  this -> fillPCData();
  this -> history_size = 500000;
  this -> fillHistoryPCData();
  this -> octmap_visu_pub = nh.advertise<octomap_msgs::Octomap>("OcTree_"+new_map_name, 100);
  this -> pcdata_visu_pub = nh.advertise<sensor_msgs::PointCloud>("PC_"+new_map_name, 100);
  this -> hpcdata_visu_pub = nh.advertise<sensor_msgs::PointCloud>("HistoryPC_"+new_map_name, 100);
}

MapUtility::MapUtility(NodeHandle& nh, GoalUtility& new_goal_util, double new_oct_resolution, string new_frame_name, string new_map_name)
{
  this -> map_name = new_map_name;
  this -> frame_name = new_frame_name;
  this -> goal_util = new_goal_util;
  this -> oct_resolution = new_oct_resolution;
  this -> max_occupancy_belief_value = 100;
  this -> octmap = new ColorOcTree(new_oct_resolution);
  this -> fillMapVisu();
  this -> fillPCData();
  this -> history_size = 500000;
  this -> fillHistoryPCData();
  this -> octmap_visu_pub = nh.advertise<octomap_msgs::Octomap>("OcTree_"+new_map_name, 100);
  this -> pcdata_visu_pub = nh.advertise<sensor_msgs::PointCloud>("PC_"+new_map_name, 100);
  this -> hpcdata_visu_pub = nh.advertise<sensor_msgs::PointCloud>("HistoryPC_"+new_map_name, 100);
}

MapUtility::MapUtility(const MapUtility& mu)
{
  this -> map_name = mu.map_name;
  this -> frame_name = mu.frame_name;
  this -> goal_util = mu.goal_util;
  this -> x_range = mu.x_range;
  this -> y_range = mu.y_range;
  this -> z_range = mu.z_range;
  this -> oct_resolution = mu.oct_resolution;
  this -> pc_resolution_factor = mu.pc_resolution_factor;
  this -> max_occupancy_belief_value = mu.max_occupancy_belief_value;    
  this -> octmap = mu.octmap;
  this -> fillMapVisu();
  this -> fillPCData();
  this -> history_size = mu.history_size;
  this -> fillHistoryPCData();
  this -> octmap_visu_pub = mu.octmap_visu_pub;
  this -> pcdata_visu_pub = mu.pcdata_visu_pub;
  this -> hpcdata_visu_pub = mu.hpcdata_visu_pub;
}

MapUtility::~MapUtility()
{
  //ROS_INFO( "Calling Destructor for MapUtility..." );

  delete (this -> octmap);
}

MapUtility& MapUtility::operator = (const MapUtility& mu) 
{ 
  this -> map_name = mu.map_name;
  this -> frame_name = mu.frame_name;
  this -> goal_util = mu.goal_util;
  this -> x_range = mu.x_range;
  this -> y_range = mu.y_range;
  this -> z_range = mu.z_range;
  this -> oct_resolution = mu.oct_resolution;
  this -> pc_resolution_factor = mu.pc_resolution_factor;
  this -> max_occupancy_belief_value = mu.max_occupancy_belief_value;      
  this -> octmap = mu.octmap;
  this -> fillMapVisu();
  this -> fillPCData();
  this -> history_size = mu.history_size;
  this -> fillHistoryPCData();
  this -> octmap_visu_pub = mu.octmap_visu_pub;
  this -> pcdata_visu_pub = mu.pcdata_visu_pub;
  this -> hpcdata_visu_pub = mu.hpcdata_visu_pub;
}

string MapUtility::getMapName()
{
  return this -> map_name;
}

string MapUtility::getFrameName()
{
  return this -> frame_name;
}   

GoalUtility& MapUtility::getGoalUtil()
{
  return this -> goal_util;
} 

vector<double> MapUtility::getXRange()
{
  return this -> x_range;
}

vector<double> MapUtility::getYRange()
{
  return this -> y_range;
}

vector<double> MapUtility::getZRange()
{
  return this -> z_range;
}

double MapUtility::getOctResolution()
{
  return this -> oct_resolution;
}

int MapUtility::getPCResolutionFactor()
{
  return this -> pc_resolution_factor;
}

double MapUtility::getMaxOccupancyBeliefValue()
{
  return this -> max_occupancy_belief_value;
}

octomap::ColorOcTree* MapUtility::getOctmap()
{
  return this -> octmap;
}

sensor_msgs::PointCloud& MapUtility::getPCData()
{
  return this -> pc_data;
}

int MapUtility::getHistorySize()
{
  return this -> history_size;
}

sensor_msgs::PointCloud& MapUtility::getHistoryPCData()
{
  return this -> history_pc_data;
}

octomap_msgs::Octomap MapUtility::getOctmapVisu()
{
  return this -> octomap_visu;
}

ros::Publisher MapUtility::getOctmapVisuPub()
{
  return this -> octmap_visu_pub;
}

void MapUtility::setMapName(string new_map_name)
{
  this -> map_name = new_map_name;
}

void MapUtility::setFrameName(string new_frame_name)
{
  this -> frame_name = new_frame_name;
}

void MapUtility::setGoalUtil(GoalUtility& new_goal_util)
{
  this -> goal_util = new_goal_util;
} 

void MapUtility::setXRange(double x0, double x1)
{
  this -> x_range[0] = x0;
  this -> x_range[1] = x1;
}

void MapUtility::setYRange(double y0, double y1)
{
  this -> y_range[0] = y0;
  this -> y_range[1] = y1;
}

void MapUtility::setZRange(double z0, double z1)
{
  this -> z_range[0] = z0;
  this -> z_range[1] = z1;
}

void MapUtility::setOctResolution(double new_oct_resolution)
{
  this -> oct_resolution = new_oct_resolution;
}

void MapUtility::setPCResolutionFactor(int new_pc_resolution_factor)
{
  this -> pc_resolution_factor = new_pc_resolution_factor;
}

void MapUtility::setMaxOccupancyBeliefValue(double new_max_occupancy_belief_value)
{
  this -> max_occupancy_belief_value = new_max_occupancy_belief_value;
}

void MapUtility::setPCData(sensor_msgs::PointCloud& new_pc_data)
{
  this -> pc_data = new_pc_data;
}

void MapUtility::setHistorySize(int new_hsize)
{
  this -> history_size = new_hsize;
}

void MapUtility::setHistoryPCData(sensor_msgs::PointCloud& new_hpc_data)
{
  this -> history_pc_data.header = new_hpc_data.header;
  this -> history_pc_data.channels = new_hpc_data.channels;
  this -> history_pc_data.points = new_hpc_data.points;
}

void MapUtility::addPCData(sensor_msgs::PointCloud& new_pc_data)
{
  this -> pc_data.header = new_pc_data.header;
  this -> pc_data.channels = new_pc_data.channels;
  this -> pc_data.points.insert(end(this -> pc_data.points), begin(new_pc_data.points), end(new_pc_data.points));
}

void MapUtility::addHistoryPCData(sensor_msgs::PointCloud& new_hpc_data)
{
  this -> history_pc_data.header = new_hpc_data.header;
  this -> history_pc_data.channels = new_hpc_data.channels;

  this -> history_pc_data.points.insert(this -> history_pc_data.points.end(), new_hpc_data.points.begin(), new_hpc_data.points.end());

  if(this -> history_pc_data.points.size() > this -> history_size)
  {
    int delta = this -> history_pc_data.points.size() - this -> history_size;
    this -> history_pc_data.points.erase(this -> history_pc_data.points.begin(), this -> history_pc_data.points.begin() + delta);
  }
  //ROS_INFO_STREAM("MapUtility::addHistoryPCData ADDED............");
}

void MapUtility::fillPCData()
{
  this -> pc_data.points.clear();
  
  for(octomap::ColorOcTree::iterator it = this -> octmap -> begin(); it != this -> octmap -> end(); ++it)
  {
    geometry_msgs::Point op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    //this -> history_pc_data.points.push_back(op);

    vector<geometry_msgs::Point32> opc = this -> extract_pc_from_node_center(op);
    for(int i = 0; i < opc.size(); i++)
    {
      this -> pc_data.points.push_back(opc[i]);
    }
  }

  this -> pc_data.header.frame_id = this -> frame_name;
}

void MapUtility::fillHistoryPCData()
{
  this -> history_pc_data.points.clear();
  
  for(octomap::ColorOcTree::iterator it = this -> octmap -> begin(); it != this -> octmap -> end(); ++it)
  {
    geometry_msgs::Point32 op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    this -> history_pc_data.points.push_back(op);
  }
  this -> history_pc_data.header.frame_id = this -> frame_name;
}

void MapUtility::fillOctMap()
{
  this -> octmap -> clear();

  for(int i = 0; i < this -> pc_data.points.size(); i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = this -> pc_data.points[i].x;
    rp.position.y = this -> pc_data.points[i].y;
    rp.position.z = this -> pc_data.points[i].z;

    this -> octmap -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
  }
  octomap_msgs::fullMapToMsg(*(this -> octmap), this -> octomap_visu);
  this -> fillMapVisu();
}

void MapUtility::addStaticObstacle2PCData(geometry_msgs::Point po)
{
  vector<geometry_msgs::Point32> opc = this -> extract_pc_from_node_center(po);
  for(int i = 0; i < opc.size(); i++)
  {
    this -> pc_data.points.push_back(opc[i]);
  }
  this -> pc_data.header.frame_id = this -> frame_name;
}

bool MapUtility::isOccupied(double x, double y, double z)
{
  OcTreeNode* node = this -> octmap -> search(x, y, z);
  if(node)
  {
    return this -> octmap -> isNodeOccupied(node);
  }
  else
  {
    return false;
  }
}

bool MapUtility::isOccupied(geometry_msgs::Point po)
{
  OcTreeNode* node = this -> octmap -> search(po.x, po.y, po.z);
  if(node)
  {
    return this -> octmap -> isNodeOccupied(node);
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

bool MapUtility::isOccupiedByGoal(double x, double y, double z)
{
  geometry_msgs::Point po;
  po.x = x;
  po.y = y;
  po.z = z;

  double free_rad = 2 * (this -> oct_resolution);

  int goal_count = this -> goal_util.getGoal().size();

  for(int i = 0; i < goal_count; i++)
  {
    if( isInCube(po, this -> goal_util.getGoal()[i].position, free_rad) )
    {
      return true;
    }
  }
  return false;
}

bool MapUtility::isOccupiedByGoal(geometry_msgs::Point po)
{
  double free_rad = 2 * (this -> oct_resolution);

  int goal_count = this -> goal_util.getGoal().size();

  for(int i = 0; i < goal_count; i++)
  {
    if( isInCube(po, this -> goal_util.getGoal()[i].position, free_rad) )
    {
      return true;
    }
  }
  return false;
}

bool MapUtility::addStaticObstacle(double x, double y, double z, bool constraint_flag, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  geometry_msgs::Point po;
  po.x = x;
  po.y = y;
  po.z = z;

  int goal_count = this -> goal_util.getGoal().size();

  if( constraint_flag && (isOccupied(x, y, z) || isOccupiedByGoal(x, y, z) || isInCube(po, robot_center, robot_free_rad + (this -> oct_resolution))) )
  {
    return false;
  }
  else
  {
    this -> octmap -> updateNode(x, y, z, true);
    this -> octmap -> setNodeColor(this -> octmap -> coordToKey(x, y, z), color_RGB[0], color_RGB[1], color_RGB[2]);
    octomap_msgs::fullMapToMsg(*(this -> octmap), this -> octomap_visu);
    this -> addStaticObstacle2PCData(po);
    this -> fillMapVisu();
    return true;
  }
}

vector<bool> MapUtility::addStaticObstacle(sensor_msgs::PointCloud pcd, bool constraint_flag, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  vector<bool> pc_add_result;
      
  int pcd_size = pcd.points.size();

  for(int i = 0; i < pcd_size; i++)
  {
    pc_add_result.push_back( addStaticObstacle(pcd.points[i].x, pcd.points[i].y, pcd.points[i].z, constraint_flag, robot_center, robot_free_rad, color_RGB) );
  }
  return pc_add_result;
}

void MapUtility::createRandomStaticObstacleMap(int num, bool constraint_flag, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  this -> octmap -> clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(this -> x_range[0], this -> x_range[1]);
    rp.position.y = randdouble(this -> y_range[0], this -> y_range[1]);
    rp.position.z = randdouble(this -> z_range[0], this -> z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position) || isInCube(rp.position, robot_center, robot_free_rad + (this -> oct_resolution))) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(this -> x_range[0], this -> x_range[1]);
      rp.position.y = randdouble(this -> y_range[0], this -> y_range[1]);
      rp.position.z = randdouble(this -> z_range[0], this -> z_range[1]);
    }

    this -> octmap -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    this -> octmap -> setNodeColor(this -> octmap -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);
    octomap_msgs::fullMapToMsg(*(this -> octmap), this -> octomap_visu);
  }
  this -> fillMapVisu();
  this -> fillPCData();
}

void MapUtility::createRandomStaticObstacleMap(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  this -> octmap -> clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
    rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
    rp.position.z = randdouble(new_z_range[0], new_z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position) || isInCube(rp.position, robot_center, robot_free_rad + (this -> oct_resolution))) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
      rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
      rp.position.z = randdouble(new_z_range[0], new_z_range[1]);
    }

    this -> octmap -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    this -> octmap -> setNodeColor(this -> octmap -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);
    octomap_msgs::fullMapToMsg(*(this -> octmap), this -> octomap_visu);
  }
  this -> fillMapVisu();
  //this -> fillPCData();
}

void MapUtility::addRandomStaticObstacle(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
    rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
    rp.position.z = randdouble(new_z_range[0], new_z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position) || isInCube(rp.position, robot_center, robot_free_rad + (this -> oct_resolution))) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
      rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
      rp.position.z = randdouble(new_z_range[0], new_z_range[1]);
    }

    this -> octmap -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    this -> octmap -> setNodeColor(this -> octmap -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);
    octomap_msgs::fullMapToMsg(*(this -> octmap), this -> octomap_visu);
    this -> addStaticObstacle2PCData(rp.position);
  }
  this -> fillMapVisu();
}

void MapUtility::fillMapVisu()
{
  this -> octomap_visu.header.frame_id = this -> frame_name;
  this -> octomap_visu.binary = false;
  this -> octomap_visu.id = this -> map_name;
  this -> octomap_visu.resolution = this -> oct_resolution;
  octomap_msgs::fullMapToMsg(*(this -> octmap), this -> octomap_visu);
}

void MapUtility::publishMap()
{
  this -> octomap_visu.header.seq++;
  this -> octomap_visu.header.stamp = ros::Time(0);
  this -> octmap_visu_pub.publish(this -> octomap_visu);

  this -> pc_data.header.seq++;
  this -> pc_data.header.stamp = ros::Time(0);
  this -> pcdata_visu_pub.publish(this -> pc_data);

  // PUBLISH THE GOAL
  this -> goal_util.publishGoal();

  //ros::spinOnce();
  //ros::Duration(0.01).sleep();
}

void MapUtility::saveMap(string filename)
{
  if(filename == "")
  {
    filename = this -> createFileName();
  }

  ofstream map_file;
  map_file.open ("/home/akmandor/catkin_ws/src/tentabot/data/" + filename + ".csv");

  if( map_file.is_open() )
  {
    map_file << "map_name," + this -> map_name + "\n";
    map_file << "frame_name," + this -> frame_name + "\n";
    map_file << "x_range," + to_string(this -> x_range[0]) + "," + to_string(this -> x_range[1]) + "\n";
    map_file << "y_range," + to_string(this -> y_range[0]) + "," + to_string(this -> y_range[1]) + "\n";
    map_file << "z_range," + to_string(this -> z_range[0]) + "," + to_string(this -> z_range[1]) + "\n";
    map_file << "oct_resolution," + to_string(this -> oct_resolution) + "\n";
    map_file << "pc_resolution_factor," + to_string(this -> pc_resolution_factor) + "\n";
    map_file << "max_occupancy_belief_value," + to_string(this -> max_occupancy_belief_value) + "\n";
    map_file << "goal,\n";

    geometry_msgs::Point po;
    int goal_cnt = this -> goal_util.getGoal().size();
    for(int i = 0; i < goal_cnt; i++)
    {
      po = this -> goal_util.getGoal()[i].position;
      map_file << to_string(po.x) + "," + to_string(po.y) + "," + to_string(po.z) + "\n";
    }

    map_file << "map,\n";

    for(octomap::ColorOcTree::iterator it = this -> octmap -> begin(); it != this -> octmap -> end(); ++it)
    {
      geometry_msgs::Point op;
      map_file << to_string(it.getCoordinate().x()) + "," + to_string(it.getCoordinate().y()) + "," + to_string(it.getCoordinate().z()) + "\n";
    }
    map_file.close();
  }
  else
  {
    cout << "Unable to open map file to save." << endl;
  }
}

void MapUtility::loadMap(string filename)
{
  ifstream map_file("/home/akmandor/catkin_ws/src/tentabot/data/" + filename + ".csv");

  if( map_file.is_open() )
  {
    cout << filename + " is loading from the file..." << endl;

    this -> octmap -> clear();
    this -> goal_util.clearGoal();
    this -> goal_util.clearGoalVisu();

    string line = "";
    bool goal_flag = false;
    bool map_flag = false;
    while( getline(map_file, line) )
    {
      vector<string> vec;
      boost::algorithm::split(vec, line, boost::is_any_of(","));

      if(vec[0] == "goal")
      {
        goal_flag = true;
        continue;
      }

      if(vec[0] == "map")
      {
        map_flag = true;
        goal_flag = false;
        continue;
      }

      if(goal_flag)
      {
        this -> goal_util.addGoalPoint( atof(vec[0].c_str()), atof(vec[1].c_str()), atof(vec[2].c_str()) );
      }

      if(map_flag)
      {          
        this -> addStaticObstacle( atof(vec[0].c_str()), atof(vec[1].c_str()), atof(vec[2].c_str()) );
      }
    }

    // Close the File
    map_file.close();
    cout << filename + " is loaded!" << endl;
  }
  else
  {
    cout << "Unable to open " + filename + " file to load." << endl;
  }
}

void MapUtility::createRandomMapSet(string mapset_name, int map_cnt, int map_occupancy_count, bool random_goal_flag)
{
  vector<double> goal_x_range;
  goal_x_range.push_back(this -> x_range[0] + 4);
  goal_x_range.push_back(this -> x_range[1] - 4);
  vector<double> goal_y_range;
  goal_y_range.push_back(this -> y_range[0] + 4);
  goal_y_range.push_back(this -> y_range[1] - 4);
  vector<double> goal_z_range;
  goal_z_range.push_back(this -> z_range[0] + 4);
  goal_z_range.push_back(this -> z_range[1] - 4);
  
  for (int i = 0; i < map_cnt; i++)
  {
    if(random_goal_flag)
    {
      this -> goal_util.setRandomGoal(goal_x_range, goal_y_range, goal_z_range, 1);
    }
    this -> createRandomStaticObstacleMap(map_occupancy_count);
    this -> saveMap("mapset/" + mapset_name + "/map" + to_string(i));
  }
}