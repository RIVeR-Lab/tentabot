// LAST UPDATE: 2021.03.20
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

// --OUTSOURCE LIBRARIES--
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <octomap_msgs/conversions.h>
#include <iostream>
#include <fstream>

// --NAMESPACES--
using namespace std;
using namespace ros;
using namespace pcl;
using namespace octomap;

// DESCRIPTION: TODO...
class MapUtility
{
  public:

    // DESCRIPTION: TODO...
    MapUtility();

    // DESCRIPTION: TODO...
  	MapUtility(NodeHandle& nh, 
               vector<double> new_x_range, 
               vector<double> new_y_range, 
               vector<double> new_z_range, 
               double new_oct_resolution, 
               int new_pc_resolution_factor, 
               string new_frame_name, 
               string new_map_name);

    // DESCRIPTION: TODO...
  	MapUtility(NodeHandle& nh, 
               vector<double> new_x_range, 
               vector<double> new_y_range, 
               vector<double> new_z_range, 
               double new_oct_resolution, 
               int new_pc_resolution_factor, 
               string new_frame_name, 
               string new_map_name, 
               sensor_msgs::PointCloud& new_pc);

    // DESCRIPTION: TODO...
    MapUtility(NodeHandle& nh, double new_oct_resolution, string new_frame_name, string new_map_name);

    // DESCRIPTION: TODO...
  	MapUtility(const MapUtility& mu);

    // DESCRIPTION: TODO...
  	~MapUtility();

    // DESCRIPTION: TODO...
  	MapUtility& operator = (const MapUtility& mu);

    // DESCRIPTION: TODO...
    string getMapName();

    // DESCRIPTION: TODO...
    string getFrameName();

    // DESCRIPTION: TODO...
    vector<double> getXRange();

    // DESCRIPTION: TODO...
    vector<double> getYRange();

    // DESCRIPTION: TODO...
    vector<double> getZRange();

    // DESCRIPTION: TODO...
    double getOctResolution();

    // DESCRIPTION: TODO...
    int getPCResolutionFactor();

    // DESCRIPTION: TODO...
    int getHistorySize();

    // DESCRIPTION: TODO...
    double getMaxOccupancyBeliefValue();

    // DESCRIPTION: TODO...
    octomap::ColorOcTree* getOctmap();

    // DESCRIPTION: TODO...
    octomap_msgs::Octomap& getOctmapMsg();

    // DESCRIPTION: TODO...
    sensor_msgs::PointCloud& getRecentPCMsg();

    // DESCRIPTION: TODO...
    sensor_msgs::PointCloud& getHistoryPCMsg();

    // DESCRIPTION: TODO...
    sensor_msgs::PointCloud& getLocalPCMsg();

    // DESCRIPTION: TODO...
    ros::Publisher getOctmapMsgPub();

    // DESCRIPTION: TODO...
    ros::Publisher getRecentPCMsgPub();

    // DESCRIPTION: TODO...
    ros::Publisher getHistoryPCMsgPub();

    // DESCRIPTION: TODO...
    ros::Publisher getLocalPCMsgPub();

    // DESCRIPTION: TODO...
    void setMapName(string new_map_name);

    // DESCRIPTION: TODO...
    void setFrameName(string new_frame_name);

    // DESCRIPTION: TODO...
    void setXRange(double x0, double x1);

    // DESCRIPTION: TODO...
    void setYRange(double y0, double y1);

    // DESCRIPTION: TODO...
    void setZRange(double z0, double z1);

    // DESCRIPTION: TODO...
    void setOctResolution(double new_oct_resolution);

    // DESCRIPTION: TODO...
    void setPCResolutionFactor(int new_pc_resolution_factor);

    // DESCRIPTION: TODO...
    void setMaxOccupancyBeliefValue(double new_max_occupancy_belief_value);

    // DESCRIPTION: TODO...
    void setHistorySize(int new_history_size);

    // DESCRIPTION: TODO...
    void setRecentPCMsg(sensor_msgs::PointCloud& new_recent_pc);

    // DESCRIPTION: TODO...
    void setHistoryPCMsg(sensor_msgs::PointCloud& new_history_pc);

    // DESCRIPTION: TODO...
    void setLocalPCMsg(sensor_msgs::PointCloud& new_local_pc);

    // DESCRIPTION: TODO...
    void fillOctmap(sensor_msgs::PointCloud pc_msg);

    // DESCRIPTION: TODO...
    void fillOctmapFromRecentPCMsg();

    // DESCRIPTION: TODO...
    void fillOctmapMsgFromOctmap();

    // DESCRIPTION: TODO...
    void fillOctmapMsgFromOctmap(string new_frame_name);

    // DESCRIPTION: TODO...
    void fillRecentPCMsgFromOctmap();

    // DESCRIPTION: TODO...
    void fillRecentPCMsgFromOctmapByResolutionFactor();

    // DESCRIPTION: TODO...
    void fillHistoryPCMsgFromOctmap();

    // DESCRIPTION: TODO...
    void fillLocalPCMsgFromOctmap();

    // DESCRIPTION: TODO...
    void addOctmap(sensor_msgs::PointCloud pc_msg);

    // DESCRIPTION: TODO...
    void addOctmapFromRecentPCMsg();

    // DESCRIPTION: TODO...
    void addRecentPCMsg(sensor_msgs::PointCloud& new_recent_pc);

    // DESCRIPTION: TODO...
    void addHistoryPCMsg(sensor_msgs::PointCloud& new_history_pc);

    // DESCRIPTION: TODO...
    void addLocalPCMsg(sensor_msgs::PointCloud& new_local_pc);

    // DESCRIPTION: TODO...
    void addLocalPCMsg(geometry_msgs::Point32 new_point);

    // DESCRIPTION: TODO...
    void clearRecentPCMsg();

    // DESCRIPTION: TODO...
    void clearHistoryPCMsg();

    // DESCRIPTION: TODO...
    void clearLocalPCMsg();

    // DESCRIPTION: TODO...
    bool isOccupied(double x, double y, double z);

    // DESCRIPTION: TODO...
    bool isOccupied(geometry_msgs::Point po);

    // DESCRIPTION: TODO...
    bool isInCube(geometry_msgs::Point po, geometry_msgs::Point center, double rad);

    // DESCRIPTION: TODO...
    bool isOccupiedByGoal(double x, double y, double z, vector<geometry_msgs::Pose> goal);

    // DESCRIPTION: TODO...
    bool isOccupiedByGoal(geometry_msgs::Point po, vector<geometry_msgs::Pose> goal);

    // DESCRIPTION: TODO...
    void addStaticObstacleByResolutionFactor2RecentPCMsg(geometry_msgs::Point po);

    // DESCRIPTION: TODO...
    bool addStaticObstacle(double x, 
                           double y, 
                           double z, 
                           bool constraint_flag=true, 
                           vector<geometry_msgs::Pose> goal=vector<geometry_msgs::Pose>{}, 
                           geometry_msgs::Point robot_center=geometry_msgs::Point(), 
                           double robot_free_rad=1, 
                           vector<int> color_RGB=vector<int>{155,128,0});

    // DESCRIPTION: TODO...
    vector<bool> addStaticObstacle(sensor_msgs::PointCloud& pcd, 
                                   bool constraint_flag=true, 
                                   vector<geometry_msgs::Pose> goal=vector<geometry_msgs::Pose>{}, 
                                   geometry_msgs::Point robot_center=geometry_msgs::Point(), 
                                   double robot_free_rad=1, 
                                   vector<int> color_RGB=vector<int>{155,128,0});

    // DESCRIPTION: TODO...
    void createRandomStaticObstacleMap(int num, 
                                       bool constraint_flag=true, 
                                       vector<geometry_msgs::Pose> goal=vector<geometry_msgs::Pose>{}, 
                                       geometry_msgs::Point robot_center=geometry_msgs::Point(), 
                                       double robot_free_rad=1, 
                                       vector<int> color_RGB=vector<int>{155,128,0});

    // DESCRIPTION: TODO...
    void createRandomStaticObstacleMap(vector<double> new_x_range, 
                                       vector<double> new_y_range, 
                                       vector<double> new_z_range, 
                                       int num, 
                                       bool constraint_flag=true, 
                                       vector<geometry_msgs::Pose> goal=vector<geometry_msgs::Pose>{}, 
                                       geometry_msgs::Point robot_center=geometry_msgs::Point(), 
                                       double robot_free_rad=1, 
                                       vector<int> color_RGB=vector<int>{155,128,0});

    // DESCRIPTION: TODO...
    void addRandomStaticObstacle(vector<double> new_x_range, 
                                 vector<double> new_y_range, 
                                 vector<double> new_z_range, 
                                 int num, 
                                 bool constraint_flag=true, 
                                 vector<geometry_msgs::Pose> goal=vector<geometry_msgs::Pose>{},
                                 geometry_msgs::Point robot_center=geometry_msgs::Point(), 
                                 double robot_free_rad=1, 
                                 vector<int> color_RGB=vector<int>{155,128,0});

    // DESCRIPTION: TODO...
    void createRandomMapSet(string mapset_name, int map_cnt, int map_occupancy_count);

    // DESCRIPTION: TODO...
    void publishOctmapMsg();

    // DESCRIPTION: TODO...
    void publishRecentPCMsg();

    // DESCRIPTION: TODO...
    void publishHistoryPCMsg();

    // DESCRIPTION: TODO...
    void publishLocalPCMsg();

    // DESCRIPTION: TODO...
    void saveMap(string filename="");

    // DESCRIPTION: TODO...
    void loadMap(string filename);

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
    
    // DESCRIPTION: TODO...
    double randdouble(double from, double to);

    // DESCRIPTION: TODO...
    void createColorOcTree(double new_oct_resolution, sensor_msgs::PointCloud& new_pc, vector<int> color_RGB=vector<int>{155,128,0});

    // DESCRIPTION: TODO...
    vector<geometry_msgs::Point32> extract_pc_from_node_center(geometry_msgs::Point center);

    // DESCRIPTION: TODO...
    string createFileName();
};//END of class MapUtility