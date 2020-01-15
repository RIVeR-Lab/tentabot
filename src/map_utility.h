// LAST UPDATE: 2019.08.04
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION:

// LOCAL LIBRARIES:
#include "goal_utility.h"

// OUTSOURCE LIBRARIES:
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <octomap_msgs/conversions.h>
#include <iostream>
#include <fstream>

// NAMESPACES:
using namespace std;
using namespace ros;
using namespace pcl;
using namespace octomap;

class MapUtility
{
  private:
  	string map_name;
    string frame_name;
 		vector<double> x_range;
  	vector<double> y_range;
  	vector<double> z_range;
  	double oct_resolution;
    double pc_resolution_factor;
  	double max_occupancy_belief_value;
  	octomap::ColorOcTree *octmap;
    sensor_msgs::PointCloud pc_data;
    int history_size;
    sensor_msgs::PointCloud history_pc_data;
    octomap_msgs::Octomap octomap_visu;
  	ros::Publisher octmap_visu_pub;
    ros::Publisher pcdata_visu_pub;
    ros::Publisher hpcdata_visu_pub;
    GoalUtility goal_util;
  	
    double randdouble(double from, double to);

    void createColorOcTree(double new_oct_resolution, sensor_msgs::PointCloud pc, vector<int> color_RGB=vector<int>{155,128,0});

    vector<geometry_msgs::Point32> extract_pc_from_node_center(geometry_msgs::Point center);

    string createFileName();

  public:
    MapUtility();

  	MapUtility(NodeHandle& nh, GoalUtility& new_goal_util, vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, double new_oct_resolution, int pc_resolution_factor, string new_frame_name, string new_map_name);

  	MapUtility(NodeHandle& nh, GoalUtility& new_goal_util, vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, double new_oct_resolution, int pc_resolution_factor, string new_frame_name, string new_map_name, sensor_msgs::PointCloud new_pcd);

    MapUtility(NodeHandle& nh, GoalUtility& new_goal_util, double new_oct_resolution, string new_frame_name, string new_map_name);

  	MapUtility(const MapUtility& mu);

  	~MapUtility();

  	MapUtility& operator = (const MapUtility& mu);

    string getMapName();

    string getFrameName();

    GoalUtility& getGoalUtil();

    vector<double> getXRange();

    vector<double> getYRange();

    vector<double> getZRange();

    double getOctResolution();

    int getPCResolutionFactor();

    double getMaxOccupancyBeliefValue();

    octomap::ColorOcTree* getOctmap();

    sensor_msgs::PointCloud& getPCData();

    int getHistorySize();

    sensor_msgs::PointCloud& getHistoryPCData();

    octomap_msgs::Octomap getOctmapVisu();

    ros::Publisher getOctmapVisuPub();

    ros::Publisher getPCDataVisuPub();

    void setMapName(string new_map_name);

    void setFrameName(string new_frame_name);

    void setGoalUtil(GoalUtility& new_goal_util);

    void setXRange(double x0, double x1);

    void setYRange(double y0, double y1);

    void setZRange(double z0, double z1);

    void setOctResolution(double new_oct_resolution);

    void setPCResolutionFactor(int new_pc_resolution_factor);

    void setMaxOccupancyBeliefValue(double new_max_occupancy_belief_value);

    void setPCData(sensor_msgs::PointCloud new_pcd);

    void setHistorySize(int new_hsize);

    void setHistoryPCData(sensor_msgs::PointCloud new_hpcd);

    void addPCData(sensor_msgs::PointCloud new_pc_data);

    void addHistoryPCData(sensor_msgs::PointCloud new_history_pcd);

    void fillPCData();

    void fillHistoryPCData();

    void fillOctMap();

    void addStaticObstacle2PCData(geometry_msgs::Point po);

    bool isOccupied(double x, double y, double z);

    bool isOccupied(geometry_msgs::Point po);

    bool isInCube(geometry_msgs::Point po, geometry_msgs::Point center, double rad);

    bool isOccupiedByGoal(double x, double y, double z);

    bool isOccupiedByGoal(geometry_msgs::Point po);

    bool addStaticObstacle(double x, double y, double z, bool constraint_flag=true, geometry_msgs::Point robot_center=geometry_msgs::Point(), double robot_free_rad=1, vector<int> color_RGB=vector<int>{155,128,0});

    vector<bool> addStaticObstacle(sensor_msgs::PointCloud pcd, bool constraint_flag=true, geometry_msgs::Point robot_center=geometry_msgs::Point(), double robot_free_rad=1, vector<int> color_RGB=vector<int>{155,128,0});

    void createRandomStaticObstacleMap(int num, bool constraint_flag=true, geometry_msgs::Point robot_center=geometry_msgs::Point(), double robot_free_rad=1, vector<int> color_RGB=vector<int>{155,128,0});

    void createRandomStaticObstacleMap(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag=true, geometry_msgs::Point robot_center=geometry_msgs::Point(), double robot_free_rad=1, vector<int> color_RGB=vector<int>{155,128,0});

    void addRandomStaticObstacle(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag=true, geometry_msgs::Point robot_center=geometry_msgs::Point(), double robot_free_rad=1, vector<int> color_RGB=vector<int>{155,128,0});

    void fillMapVisu();

    void publishMap();

    void saveMap(string filename="");

    void loadMap(string filename);

    void createRandomMapSet(string mapset_name, int map_cnt, int map_occupancy_count, bool random_goal_flag=true);
};