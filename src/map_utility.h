// LAST UPDATE: 2021.02.19
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION:

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
    int history_size;

  	octomap::ColorOcTree *octmap;
    octomap_msgs::Octomap octmap_msg;
    
    sensor_msgs::PointCloud recent_pc_msg;
    sensor_msgs::PointCloud history_pc_msg;
    sensor_msgs::PointCloud local_pc_msg;
    
  	ros::Publisher octmap_msg_pub;
    ros::Publisher recent_pc_msg_pub;
    ros::Publisher history_pc_msg_pub;
    ros::Publisher local_pc_msg_pub;
  	
    double randdouble(double from, double to);

    void createColorOcTree(double new_oct_resolution, sensor_msgs::PointCloud& new_pc, vector<int> color_RGB=vector<int>{155,128,0});

    vector<geometry_msgs::Point32> extract_pc_from_node_center(geometry_msgs::Point center);

    string createFileName();

  public:
    MapUtility();

  	MapUtility(NodeHandle& nh, 
               vector<double> new_x_range, 
               vector<double> new_y_range, 
               vector<double> new_z_range, 
               double new_oct_resolution, 
               int new_pc_resolution_factor, 
               string new_frame_name, 
               string new_map_name);

  	MapUtility(NodeHandle& nh, 
               vector<double> new_x_range, 
               vector<double> new_y_range, 
               vector<double> new_z_range, 
               double new_oct_resolution, 
               int new_pc_resolution_factor, 
               string new_frame_name, 
               string new_map_name, 
               sensor_msgs::PointCloud& new_pc);

    MapUtility(NodeHandle& nh, double new_oct_resolution, string new_frame_name, string new_map_name);

  	MapUtility(const MapUtility& mu);

  	~MapUtility();

  	MapUtility& operator = (const MapUtility& mu);

    string getMapName();

    string getFrameName();

    vector<double> getXRange();

    vector<double> getYRange();

    vector<double> getZRange();

    double getOctResolution();

    int getPCResolutionFactor();

    int getHistorySize();

    double getMaxOccupancyBeliefValue();

    octomap::ColorOcTree* getOctmap();

    octomap_msgs::Octomap& getOctmapMsg();

    sensor_msgs::PointCloud& getRecentPCMsg();

    sensor_msgs::PointCloud& getHistoryPCMsg();

    sensor_msgs::PointCloud& getLocalPCMsg();

    ros::Publisher getOctmapMsgPub();

    ros::Publisher getRecentPCMsgPub();

    ros::Publisher getHistoryPCMsgPub();

    ros::Publisher getLocalPCMsgPub();

    void setMapName(string new_map_name);

    void setFrameName(string new_frame_name);

    void setXRange(double x0, double x1);

    void setYRange(double y0, double y1);

    void setZRange(double z0, double z1);

    void setOctResolution(double new_oct_resolution);

    void setPCResolutionFactor(int new_pc_resolution_factor);

    void setMaxOccupancyBeliefValue(double new_max_occupancy_belief_value);

    void setHistorySize(int new_history_size);

    void setRecentPCMsg(sensor_msgs::PointCloud& new_recent_pc);

    void setHistoryPCMsg(sensor_msgs::PointCloud& new_history_pc);

    void setLocalPCMsg(sensor_msgs::PointCloud& new_local_pc);

    void fillOctmap(sensor_msgs::PointCloud pc_msg);

    void fillOctmapFromRecentPCMsg();

    void fillOctmapMsgFromOctmap();

    void fillOctmapMsgFromOctmap(string new_frame_name);

    void fillRecentPCMsgFromOctmap();

    void fillRecentPCMsgFromOctmapByResolutionFactor();

    void fillHistoryPCMsgFromOctmap();

    void fillLocalPCMsgFromOctmap();

    void addOctmap(sensor_msgs::PointCloud pc_msg);

    void addOctmapFromRecentPCMsg();

    void addRecentPCMsg(sensor_msgs::PointCloud& new_recent_pc);

    void addHistoryPCMsg(sensor_msgs::PointCloud& new_history_pc);

    void addLocalPCMsg(sensor_msgs::PointCloud& new_local_pc);

    void addLocalPCMsg(geometry_msgs::Point32 new_point);

    void clearRecentPCMsg();

    void clearHistoryPCMsg();

    void clearLocalPCMsg();

    bool isOccupied(double x, double y, double z);

    bool isOccupied(geometry_msgs::Point po);

    bool isInCube(geometry_msgs::Point po, geometry_msgs::Point center, double rad);

    bool isOccupiedByGoal(double x, double y, double z, vector<geometry_msgs::Pose> goal);

    bool isOccupiedByGoal(geometry_msgs::Point po, vector<geometry_msgs::Pose> goal);

    void addStaticObstacleByResolutionFactor2RecentPCMsg(geometry_msgs::Point po);

    bool addStaticObstacle(double x, 
                           double y, 
                           double z, 
                           bool constraint_flag=true, 
                           vector<geometry_msgs::Pose> goal=vector<geometry_msgs::Pose>{}, 
                           geometry_msgs::Point robot_center=geometry_msgs::Point(), 
                           double robot_free_rad=1, 
                           vector<int> color_RGB=vector<int>{155,128,0});

    vector<bool> addStaticObstacle(sensor_msgs::PointCloud& pcd, 
                                   bool constraint_flag=true, 
                                   vector<geometry_msgs::Pose> goal=vector<geometry_msgs::Pose>{}, 
                                   geometry_msgs::Point robot_center=geometry_msgs::Point(), 
                                   double robot_free_rad=1, 
                                   vector<int> color_RGB=vector<int>{155,128,0});

    void createRandomStaticObstacleMap(int num, 
                                       bool constraint_flag=true, 
                                       vector<geometry_msgs::Pose> goal=vector<geometry_msgs::Pose>{}, 
                                       geometry_msgs::Point robot_center=geometry_msgs::Point(), 
                                       double robot_free_rad=1, 
                                       vector<int> color_RGB=vector<int>{155,128,0});

    void createRandomStaticObstacleMap(vector<double> new_x_range, 
                                       vector<double> new_y_range, 
                                       vector<double> new_z_range, 
                                       int num, 
                                       bool constraint_flag=true, 
                                       vector<geometry_msgs::Pose> goal=vector<geometry_msgs::Pose>{}, 
                                       geometry_msgs::Point robot_center=geometry_msgs::Point(), 
                                       double robot_free_rad=1, 
                                       vector<int> color_RGB=vector<int>{155,128,0});

    void addRandomStaticObstacle(vector<double> new_x_range, 
                                 vector<double> new_y_range, 
                                 vector<double> new_z_range, 
                                 int num, 
                                 bool constraint_flag=true, 
                                 vector<geometry_msgs::Pose> goal=vector<geometry_msgs::Pose>{},
                                 geometry_msgs::Point robot_center=geometry_msgs::Point(), 
                                 double robot_free_rad=1, 
                                 vector<int> color_RGB=vector<int>{155,128,0});

    void createRandomMapSet(string mapset_name, int map_cnt, int map_occupancy_count);

    void publishOctmapMsg();

    void publishRecentPCMsg();

    void publishHistoryPCMsg();

    void publishLocalPCMsg();

    void saveMap(string filename="");

    void loadMap(string filename);
};