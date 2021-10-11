#ifndef MAP_UTILITY_H
#define MAP_UTILITY_H

// LAST UPDATE: 2021.10.05
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

// --OUTSOURCE LIBRARIES--
#include <boost/algorithm/string.hpp>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <fcl/fcl.h>
#include <fcl/geometry/shape/convex.h>
#include <laser_geometry/laser_geometry.h>

// --CUSTOM LIBRARIES--
#include "common_utility.h"
#include "tentabot/reset_map_utility.h"

// --NAMESPACES--
using namespace std;
using namespace ros;
using namespace octomap;

// DESCRIPTION: TODO...
class MapUtility
{
  public:

    // DESCRIPTION: TODO...
    MapUtility();

    // DESCRIPTION: TODO...
    MapUtility(NodeHandle& nh);

    // DESCRIPTION: TODO...
  	MapUtility(const MapUtility& mu);

    // DESCRIPTION: TODO...
  	~MapUtility();

    // DESCRIPTION: TODO...
  	MapUtility& operator = (const MapUtility& mu);

    // DESCRIPTION: TODO...
    string getWorldFrameName();

    // DESCRIPTION: TODO...
    string getMapName();

    // DESCRIPTION: TODO...
    string getMapPoseMsgName();

    // DESCRIPTION: TODO...
    string getMapFrameName();

    // DESCRIPTION: TODO...
    string getSensorPC2MsgName();

    // DESCRIPTION: TODO...
    string getSensorPC2Direction();

    // DESCRIPTION: TODO...
    string getSensorPC2FrameName();

    // DESCRIPTION: TODO...
    double getSensorPC2MinRange();

    // DESCRIPTION: TODO...
    double getSensorPC2MaxRange();

    // DESCRIPTION: TODO...
    double getSensorPC2MaxYaw();

    // DESCRIPTION: TODO...
    double getSensorPC2MaxPitch();

    // DESCRIPTION: TODO...
    string getSensorLaserMsgName();

    // DESCRIPTION: TODO...
    float getSensorLaserMaxRange();

    // DESCRIPTION: TODO...
    geometry_msgs::Pose getMeasuredSensorPC2Pose();

    // DESCRIPTION: TODO...
    geometry_msgs::Pose getSensorPC2Pose();

    // DESCRIPTION: TODO...
    geometry_msgs::Pose getMeasuredMapPose();

    // DESCRIPTION: TODO...
    geometry_msgs::Pose getMapPose();

    // DESCRIPTION: TODO...
    vector<double> getXRange();

    // DESCRIPTION: TODO...
    vector<double> getYRange();

    // DESCRIPTION: TODO...
    vector<double> getZRange();

    // DESCRIPTION: TODO...
    double getBBxXMax();

    // DESCRIPTION: TODO...
    double getBBxXMin();

    // DESCRIPTION: TODO...
    double getBBxYMax();

    // DESCRIPTION: TODO...
    double getBBxYMin();

    // DESCRIPTION: TODO...
    double getBBxZMax();

    // DESCRIPTION: TODO...
    double getBBxZMin();

    // DESCRIPTION: TODO...
    double getCropXMax();

    // DESCRIPTION: TODO...
    double getCropXMin();

    // DESCRIPTION: TODO...
    double getCropYMax();

    // DESCRIPTION: TODO...
    double getCropYMin();

    // DESCRIPTION: TODO...
    double getCropZMax();

    // DESCRIPTION: TODO...
    double getCropZMin();

    // DESCRIPTION: TODO...
    bool getFilterGround();

    // DESCRIPTION: TODO...
    double getFilterGroundThreshold();

    // DESCRIPTION: TODO...
    double getMapResolution();

    // DESCRIPTION: TODO...
    double getPCResolutionScale();

    // DESCRIPTION: TODO...
    double getMaxOccupancyBeliefValue();

    // DESCRIPTION: TODO...
    double getMapServerDt();

    // DESCRIPTION: TODO...
    bool getLocalMapFlag();

    // DESCRIPTION: TODO...
    bool getDynamicFlag();

    // DESCRIPTION: TODO...
    int getSkipCntResetSensorRange();

    // DESCRIPTION: TODO...
    shared_ptr<octomap::ColorOcTree> getOct();

    // DESCRIPTION: TODO...
    octomap_msgs::Octomap& getOctMsg();

    // DESCRIPTION: TODO...
    octomap::Pointcloud& getOctPC();

    // DESCRIPTION: TODO...
    sensor_msgs::PointCloud& getMeasuredPCMsg();

    // DESCRIPTION: TODO...
    sensor_msgs::PointCloud2& getMeasuredPC2Msg();

    // DESCRIPTION: TODO...
    sensor_msgs::PointCloud& getPCMsg();

    // DESCRIPTION: TODO...
    sensor_msgs::PointCloud2& getPC2Msg();

    // DESCRIPTION: TODO...
    ros::Publisher getOctMsgPub();

    // DESCRIPTION: TODO...
    ros::Publisher getPCMsgPub();

    // DESCRIPTION: TODO...
    ros::Publisher getPC2MsgPub();

    // DESCRIPTION: TODO...
    void setWorldFrameName(string new_world_frame_name);

    // DESCRIPTION: TODO...
    void setMapName(string new_map_name);

    // DESCRIPTION: TODO...
    void setMapFrameName(string new_map_frame_name);

    // DESCRIPTION: TODO...
    void setSensorPC2MsgName(string new_sensor_pc2_msg_name);

    // DESCRIPTION: TODO...
    void setSensorPC2Direction(string new_sensor_pc2_direction);

    // DESCRIPTION: TODO...
    void setSensorPC2FrameName(string new_sensor_pc2_frame_name);

    // DESCRIPTION: TODO...
    void setSensorPC2MinRange(double new_sensor_pc2_min_range);

    // DESCRIPTION: TODO...
    void setSensorPC2MaxRange(double new_sensor_pc2_max_range);

    // DESCRIPTION: TODO...
    void setSensorPC2MaxYaw(double new_sensor_pc2_max_yaw);

    // DESCRIPTION: TODO...
    void setSensorPC2MaxPitch(double new_sensor_pc2_max_pitch);

    // DESCRIPTION: TODO...
    void setSensorLaserMsgName(string new_sensor_laser_msg_name);

    // DESCRIPTION: TODO...
    void setSensorLaserMaxRange(float new_sensor_laser_max_range);

    // DESCRIPTION: TODO...
    void setMeasuredSensorPC2Pose(geometry_msgs::Pose new_measured_sensor_pc2_pose);

    // DESCRIPTION: TODO...
    void setSensorPC2Pose(geometry_msgs::Pose new_sensor_pc2_pose);

    // DESCRIPTION: TODO...
    void setMeasuredMapPose(geometry_msgs::Pose new_measured_map_pose);

    // DESCRIPTION: TODO...
    void setMapPose(geometry_msgs::Pose new_map_pose);

    // DESCRIPTION: TODO...
    void setXRange(double x0, double x1);

    // DESCRIPTION: TODO...
    void setYRange(double y0, double y1);

    // DESCRIPTION: TODO...
    void setZRange(double z0, double z1);

    // DESCRIPTION: TODO...
    void setBBxXMax(double new_bbx_x_max);

    // DESCRIPTION: TODO...
    void setBBxXMin(double new_bbx_x_min);

    // DESCRIPTION: TODO...
    void setBBxYMax(double new_bbx_y_max);

    // DESCRIPTION: TODO...
    void setBBxYMin(double new_bbx_y_min);

    // DESCRIPTION: TODO...
    void setBBxZMax(double new_bbx_z_max);

    // DESCRIPTION: TODO...
    void setBBxZMin(double new_bbx_z_min);

    // DESCRIPTION: TODO...
    void setCropXMax(double new_crop_x_max);

    // DESCRIPTION: TODO...
    void setCropXMin(double new_crop_x_min);

    // DESCRIPTION: TODO...
    void setCropYMax(double new_crop_y_max);

    // DESCRIPTION: TODO...
    void setCropYMin(double new_crop_y_min);

    // DESCRIPTION: TODO...
    void setCropZMax(double new_crop_z_max);

    // DESCRIPTION: TODO...
    void setCropZMin(double new_crop_z_min);

    // DESCRIPTION: TODO...
    void setFilterGround(bool new_filter_ground);

    // DESCRIPTION: TODO...
    void setFilterGroundThreshold(double new_filter_ground_threshold);

    // DESCRIPTION: TODO...
    void setMapResolution(double new_map_resolution);

    // DESCRIPTION: TODO...
    void setPCResolutionScale(double new_pc_resolution_scale);

    // DESCRIPTION: TODO...
    void setMaxOccupancyBeliefValue(double new_max_occupancy_belief_value);

    // DESCRIPTION: TODO...
    void setMapServerDt(double new_map_server_dt);

    // DESCRIPTION: TODO...
    void setLocalMapFlag(bool new_local_map_flag);

    // DESCRIPTION: TODO...
    void setDynamicFlag(bool new_dynamic_flag);

    // DESCRIPTION: TODO...
    void setSkipCntResetSensorRange(double new_skip_cnt_reset_sensor_range);

    // DESCRIPTION: TODO...
    void setOctPC(octomap::Pointcloud& new_oct_pc);

    // DESCRIPTION: TODO...
    void setMeasuredPCMsg(sensor_msgs::PointCloud& new_measured_pc);

    // DESCRIPTION: TODO...
    void setMeasuredPC2Msg(sensor_msgs::PointCloud2& new_measured_pc2);

    // DESCRIPTION: TODO...
    void setPCMsg(sensor_msgs::PointCloud& new_pc);

    // DESCRIPTION: TODO...
    void setPC2Msg(sensor_msgs::PointCloud2& new_pc2);

    // DESCRIPTION: TODO...
    void resetMap();

    // DESCRIPTION: TODO...
    void transformPoint(string frame_from,
                        string frame_to,
                        geometry_msgs::Point& p_to_msg);

    // DESCRIPTION: TODO...
    void createColorOcTree( double new_map_resolution, 
                            sensor_msgs::PointCloud& new_pc, 
                            vector<int> color_RGB=vector<int>{155,128,0});

    // DESCRIPTION: TODO...
    vector<geometry_msgs::Point32> extract_pc_from_node_center(geometry_msgs::Point center);

    // DESCRIPTION: TODO...
    void fillOct(sensor_msgs::PointCloud& pc_msg);

    // DESCRIPTION: TODO...
    void fillOctFromMeasuredPCMsg();

    // DESCRIPTION: TODO...
    void fillOctMsgFromOct();

    // DESCRIPTION: TODO...
    void fillPCMsgFromOct();

    // DESCRIPTION: TODO...
    void fillPCMsgFromOctByResolutionScale();

    // DESCRIPTION: TODO...
    void fillDebugArrayVisu(vector<tf::Vector3> v);

    // DESCRIPTION: TODO...
    void fillDebugVisu(vector<tf::Vector3> v);

    // DESCRIPTION: TODO...
    void insertToOctFromOctPC(octomap::point3d sensor_origin);

    // DESCRIPTION: TODO...
    void addToOct(sensor_msgs::PointCloud& pc_msg);

    // DESCRIPTION: TODO...
    void addToOctFromMeasuredPCMsg();

    // DESCRIPTION: TODO...
    void addToPCMsg(sensor_msgs::PointCloud& new_pc);

    // DESCRIPTION: TODO...
    void addToPCMsg(geometry_msgs::Point32 new_point);

    // DESCRIPTION: TODO...
    void clearMeasuredPCMsg();

    // DESCRIPTION: TODO...
    void clearPCMsg();

    // DESCRIPTION: TODO...
    bool isOccupied(double x, double y, double z);

    // DESCRIPTION: TODO...
    bool isOccupied(geometry_msgs::Point po);

    // DESCRIPTION: TODO...
    void setSensorRangeQuery(float query_point_resolution=0.01);

    // DESCRIPTION: TODO...
    void constructCameraSensorRange();

    // DESCRIPTION: TODO...
    void constructLaserSensorRange(float sensor_laser_z_range=0.3);

    // DESCRIPTION: TODO...
    bool isInSensorCameraRange(tf::Vector3 query, bool on_flag=false);

    // DESCRIPTION: TODO...
    bool isInSensorLaserRange(tf::Vector3 query, bool on_flag=false);

    // DESCRIPTION: TODO...
    bool isInCube(geometry_msgs::Point po, geometry_msgs::Point center, double rad);

    // DESCRIPTION: TODO...
    bool isOccupiedByGoal(double x, double y, double z, vector<geometry_msgs::Pose> goal);

    // DESCRIPTION: TODO...
    bool isOccupiedByGoal(geometry_msgs::Point po, vector<geometry_msgs::Pose> goal);

    // DESCRIPTION: TODO...
    void addStaticObstacleByResolutionScale2PCMsg(geometry_msgs::Point po);

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
    void crop(sensor_msgs::PointCloud2& cloud_in, 
              octomap::point3d lowerBound, 
              octomap::point3d upperBound, 
              octomap::Pointcloud& cloud_out, 
              bool keep_in=true);

    // DESCRIPTION: TODO...
    void cropOctPCFromPC2Msg( octomap::point3d lowerBound, 
                              octomap::point3d upperBound, 
                              bool keep_in=true);

    // DESCRIPTION: TODO...
    void updateOctPC();

    // DESCRIPTION: TODO...
    void pointCloud2ToOctomap(const sensor_msgs::PointCloud2& cloud, octomap::Pointcloud& octomapCloud);

    // DESCRIPTION: TODO...
    void publishOctMsg();

    // DESCRIPTION: TODO...
    void publishPCMsg();

    // DESCRIPTION: TODO...
    void publishPC2Msg();

    // DESCRIPTION: TODO...
    void publishDebugArrayVisu();

    // DESCRIPTION: TODO...
    void publishDebugVisu();

    // DESCRIPTION: TODO...
    void printDataSize();

    // DESCRIPTION: TODO...
    void saveMap(string filename="");

    // DESCRIPTION: TODO...
    void loadMap(string filename);

    // DESCRIPTION: TODO...
    void mapPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void mapOdometryCallback(const nav_msgs::Odometry& msg);

    // DESCRIPTION: TODO...
    void pc2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void update_states();

    // DESCRIPTION: TODO...
    void update_map();

    // DESCRIPTION: TODO...
    bool reset_map_utility(tentabot::reset_map_utility::Request& req, tentabot::reset_map_utility::Response& res);

    // DESCRIPTION: TODO...
    void mainCallback(const ros::TimerEvent& e);

  private:

    tf::TransformListener* tflistener;

    string world_frame_name;

    string map_name;
    string map_frame_name;

    string sensor_pc2_msg_name;
    string sensor_pc2_direction;
    string sensor_pc2_frame_name;

    string sensor_laser_msg_name;
    string sensor_laser_frame_name;
    
    geometry_msgs::Pose measured_sensor_pc2_pose;
    geometry_msgs::Pose measured_map_pose;
    geometry_msgs::Pose sensor_pc2_pose;
    geometry_msgs::Pose map_pose;

    tf::StampedTransform measured_transform_sensor_pc2_wrt_world;
    tf::StampedTransform transform_map_wrt_world;
    tf::StampedTransform transform_sensor_pc2_wrt_world;
    tf::StampedTransform transform_sensor_laser_wrt_world;

    vector<double> x_range; // NUA: deprecated, use bbx variables
    vector<double> y_range; // NUA: deprecated, use bbx variables
    vector<double> z_range; // NUA: deprecated, use bbx variables

    double bbx_x_max;
    double bbx_x_min;
    double bbx_y_max;
    double bbx_y_min;
    double bbx_z_max;
    double bbx_z_min;
    double crop_x_max;
    double crop_x_min;
    double crop_y_max;
    double crop_y_min;
    double crop_z_max;
    double crop_z_min;

    bool filter_ground;
    double filter_ground_threshold;

    double map_resolution;
    double pc_resolution_scale;
    double max_occupancy_belief_value;
    double map_server_dt;
    bool local_map_flag;
    bool dynamic_flag;
    int skip_cnt;
    int skip_cnt_reset_sensor_range;

    std::shared_ptr<fcl::Box<float>> query_sharedPtr;

    double sensor_pc2_min_range;
    double sensor_pc2_max_range;
    double sensor_pc2_max_yaw;
    double sensor_pc2_max_pitch;
    std::shared_ptr<fcl::Convex<float>> sensor_pc2_range_sharedPtr;
    fcl::CollisionRequest<float> sensor_pc2_range_request;
    fcl::CollisionResult<float> sensor_pc2_range_result;

    float sensor_laser_max_range;
    std::shared_ptr<fcl::Cylinder<float>> sensor_laser_range_sharedPtr;
    fcl::CollisionRequest<float> sensor_laser_range_request;
    fcl::CollisionResult<float> sensor_laser_range_result;

    laser_geometry::LaserProjection sensor_laser_projector;

    //octomap::ColorOcTree* oct;
    shared_ptr<octomap::ColorOcTree> oct;
    octomap_msgs::Octomap oct_msg;
    octomap::Pointcloud oct_pc;
    
    sensor_msgs::PointCloud measured_pc_msg;
    sensor_msgs::PointCloud2 measured_pc2_msg;
    sensor_msgs::LaserScan measured_laser_msg;
    sensor_msgs::PointCloud pc_msg;
    sensor_msgs::PointCloud2 pc2_msg;
    sensor_msgs::PointCloud2 laser_pc2_msg;

    visualization_msgs::MarkerArray debug_array_visu;
    visualization_msgs::Marker debug_visu;
    
    ros::Publisher oct_msg_pub;
    ros::Publisher pc_msg_pub;
    ros::Publisher pc2_msg_pub;
    ros::Publisher debug_array_visu_pub;
    ros::Publisher debug_visu_pub;

};//END of class MapUtility

#endif