#ifndef TENTABOT_H
#define TENTABOT_H

// LAST UPDATE: 2021.10.08
//
// AUTHOR: Neset Unver Akmandor
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

// --OUTSOURCE LIBRARIES--
#include <std_msgs/Float64MultiArray.h>
#include <thread>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// --CUSTOM LIBRARIES--
#include "common_utility.h"
#include "goal_utility.h"
#include "tentabot/rl_step.h"
#include "tentabot/update_goal.h"

// --NAMESPACES--
using namespace std;
using namespace ros;
using namespace octomap;

// DESCRIPTION: TODO...
class Tentabot
{
  public:

    // DESCRIPTION: TODO...
    struct OccupancyVoxel
    {
      int index = 0;                                // TODO: Review: index number of the support voxel in the linearized 3D grid, range: 0 <= index, E Z+
      double weight = 0;                            // TODO: Review: weight of the support voxel which is calculated based on the distance from the closest tentacle sample point, range: 0 <= weight, E R+ 
      int histbin = 0;                              // TODO: Review: occupancy histogram bin index on the tentacle whose location is the closest to the support voxel, range: 0 <= histbin, E Z+ 
      bool flag = false;                            // TODO: Review: flag that determines if the occupancy voxel is either classification voxel (false) or priority voxel (true)
    };

    // DESCRIPTION: TODO...
    struct EgoGrid
    {
      vector<geometry_msgs::Point> ovox_pos;        // TODO: Review: vector that keeps occupancy voxel positions in the ego-grid 
      vector<double> ovox_value;                    // TODO: Review: vector that keeps occupancy voxel values (obtained by navigation sensor) in the ego-grid
    };

    // DESCRIPTION: TODO...
    struct ProcessParams
    {
      bool visu_flag;                               // TODO: Review: flag to display simulation (setting and publishing markers etc.)
      bool online_tuning_flag;                      // TODO: Review: flag to auto-tune online parameters
      double time_limit;                            // TODO: Review: time limit for the navigation, unit: s, range: 0 < time_limit, E R+
      double nav_dt;                                // TODO: Review: frequency of ROS loop
      double goal_close_threshold;                  // TODO: Review: threshold distance from goal to robot position in the global coordinate frame, range: 0 <= goal_close_threshold, E R+
    };

    // DESCRIPTION: TODO...
    struct RobotParams
    {
      string world_name;
      string robot_name;                                        // TODO: Review: name of the robot
      string robot_frame_name;                                  // TODO: Review: name of the robot frame
      string sensor_frame_name;
      double robot_bbx_x_max;                                   // TODO: Review: width of the robot, unit: m, range: 0 < width, E R+
      double robot_bbx_x_min;                                   // TODO: Review: width of the robot, unit: m, range: 0 < width, E R+
      double robot_bbx_y_max;                                   // TODO: Review: length of the robot, unit: m, range: 0 < length, E R+
      double robot_bbx_y_min;                                   // TODO: Review: length of the robot, unit: m, range: 0 < length, E R+
      double robot_bbx_z_max;                                   // TODO: Review: height of the robot, unit: m, range: 0 < height, E R+
      double robot_bbx_z_min;                                   // TODO: Review: height of the robot, unit: m, range: 0 < height, E R+
      double dummy_max_lat_velo;                                // TODO: Review: max forward lateral velocity of the robot, unit: m/s, range: 0 < dummy_max_lat_velo, E R+
      double dummy_max_lat_acc;                                 // TODO: Review: max forward lateral acceleration of the robot, unit: m/s^2, range: 0 < dummy_max_lat_acc, E R+
      double dummy_max_yaw_velo;                                // TODO: Review: max yaw angle velocity of the robot, unit: rad/s, range: 0 < dummy_max_yaw_velo, E R+
      double dummy_max_yaw_acc;                                 // TODO: Review: max yaw angle acceleration of the robot, unit: rad/s^2, range: 0 < dummy_max_yaw_acc, E R+
      geometry_msgs::Pose init_robot_pose;                      // TODO: Review: position and orientation of the robot's initial pose with respect to global coordinate system
      string robot_pose_control_msg;
      string robot_velo_control_msg;
      string odometry_msg;
      string map_msg;
    };

    // DESCRIPTION: TODO...
    struct OffTuningParams
    {
      string tentacle_data_path;
      vector<vector<geometry_msgs::Point>> tentacle_data;
      vector<vector<double>> velocity_control_data;
      double max_occupancy_belief_value;
      double pdist_x_max;                                 // priority distance, unit: m, range: 0 < cdist, E R+
      double pdist_x_min;                                 // priority distance, unit: m, range: 0 < cdist, E R+
      double pdist_y_max;                                 // priority distance, unit: m, range: 0 < cdist, E R+
      double pdist_y_min;                                 // priority distance, unit: m, range: 0 < cdist, E R+
      double pdist_z_max;                                 // priority distance, unit: m, range: 0 < cdist, E R+
      double pdist_z_min;                                 // priority distance, unit: m, range: 0 < cdist, E R+
      double sdist_x_max;                                 // support distance, unit: m, range: cdist < sdist, E R+
      double sdist_x_min;                                 // support distance, unit: m, range: cdist < sdist, E R+
      double sdist_y_max;                                 // support distance, unit: m, range: cdist < sdist, E R+
      double sdist_y_min;                                 // support distance, unit: m, range: cdist < sdist, E R+
      double sdist_z_max;                                 // support distance, unit: m, range: cdist < sdist, E R+
      double sdist_z_min;                                 // support distance, unit: m, range: cdist < sdist, E R+
      double sweight_max;                                 // max weight of support cells (= for priority cells) which affects drivability of a tentacle calculation based on closest tentacle sample point, range: 0 < cweight_max, E R+ 
      double sweight_scale;                               // parameter to adjust weight of support cells which affects drivability of a tentacle calculation based on closest tentacle sample point, range: 0 < cweight_scale, E R+ 
      double egrid_vdim;                                  // dimension of the voxel in the ego-grid, unit: m, range: 0 < cdim, E R+
    };

    // DESCRIPTION: TODO...
    struct OnTuningParams
    {
      int tbin_obs_cnt_threshold;                   // threshold of number of obstacle on the tentacle bin, range: 0 <= obs_num_threshold, E Z+
      double crash_dist_scale;                      // crash distance on the selected tentacle, range: 0 <= crash_dist, E R+
      double clear_scale;                           // parameter to adjust the weight of the clearance value while selecting best tentacle, range: 0 <= clear_scale, E R+
      double clutter_scale;                         // parameter to adjust the weight of the clutterness value while selecting best tentacle, range: 0 <= clutter_scale, E R+,
      double close_scale;                           // parameter to adjust the weight of the closeness value while selecting best tentacle, range: 0 <= close_scale, E R+
      double smooth_scale;                          // parameter to adjust the weight of the smoothness value while selecting best tentacle, range: 0 <= smooth_scale, E R+
      
    };

    // DESCRIPTION: TODO...
    struct HeuristicParams
    {
      vector<double> occupancy_set;
      vector<int> navigability_set;
      vector<double> clearance_set;
      vector<double> clutterness_set;
      vector<double> closeness_set;
      vector<double> smoothness_set;
    };

    // DESCRIPTION: TODO...
    struct StatusParams
    {
      vector<double> tentacle_length_data;
      geometry_msgs::Point tentacle_bbx_min;
      geometry_msgs::Point tentacle_bbx_max;
      int egrid_vnum_x_max;                               // number of voxel in the ego-grid along x direction, range: 1 <= cnumx, E Z+
      int egrid_vnum_x_min;                               // number of voxel in the ego-grid along -x direction, range: 1 <= cnumx, E Z+
      int egrid_vnum_y_max;                               // number of voxel in the ego-grid along y direction, range: 1 <= cnumy, E Z+  
      int egrid_vnum_y_min;                               // number of voxel in the ego-grid along -y direction, range: 1 <= cnumy, E Z+  
      int egrid_vnum_z_max;                               // number of voxel in the ego-grid along z direction, range: 1 <= cnumz, E Z+ 
      int egrid_vnum_z_min;                               // number of voxel in the ego-grid along -z direction, range: 1 <= cnumz, E Z+
      double egrid_dist_x_max;
      double egrid_dist_x_min;
      double egrid_dist_y_max;
      double egrid_dist_y_min;
      double egrid_dist_z_max;
      double egrid_dist_z_min;
      EgoGrid ego_grid_data;
      vector<vector<OccupancyVoxel>> support_vox_data;
      vector<vector<double>> sample_weight_data;
      vector<double> tentacle_weight_data;

      octomap_msgs::Octomap measured_map_msg;
      geometry_msgs::Pose measured_robot_pose;            // TODO: Review: most recent position and orientation of the robot with respect to global coordinate system

      string map_frame_name;
      std::shared_ptr<octomap::ColorOcTree> tmap;
      tf::StampedTransform transform_robot_wrt_world;
      geometry_msgs::Pose robot_pose;                      // TODO: Review: previous position and orientation of the robot with respect to global coordinate system
      geometry_msgs::Pose prev_robot_pose;                 // TODO: Review: position and orientation of the robot with respect to global coordinate system
      ros::Time prev_time;
      double dt;
      double lat_speed;
      double desired_lat_speed;
      double lat_speed_weight;
      double yaw_velo;
      geometry_msgs::PoseStamped command_pose;       // TODO: Review: position and orientation command of the robot with respect to global coordinate system
      geometry_msgs::Twist command_velo;

      std::vector<int> tcrash_bin;
      bool navigability_flag;
      int best_tentacle;
      int ex_best_tentacle;
      int ex_best_sample;
      
      int nav_result;
      double nav_length;
      double nav_duration;                                      // TODO: Review: navigation duration
      
      ros::Publisher command_pose_pub;
      ros::Publisher command_velo_pub;

      int counter;
      bool navexit_flag;                                        // TODO: Review: flag to exit from the navigation loop  
    };

    // DESCRIPTION: TODO...
    struct VisuParams
    {
      ros::Publisher robot_visu_pub;                            // TODO: Review: publisher for robot visualization 
      ros::Publisher tentacle_visu_pub;
      ros::Publisher best_tentacle_visu_pub;
      ros::Publisher tsamp_visu_pub;
      ros::Publisher support_vox_visu_pub;
      ros::Publisher occupancy_pc_pub;
      ros::Publisher path_visu_pub;
      ros::Publisher command_visu_pub;
      ros::Publisher debug_visu_pub;
      ros::Publisher debug_array_visu_pub;

      visualization_msgs::Marker robot_visu;                    // TODO: Review: marker for robot visualization
      visualization_msgs::MarkerArray tentacle_visu;
      visualization_msgs::MarkerArray best_tentacle_visu;
      visualization_msgs::MarkerArray tsamp_visu;
      visualization_msgs::MarkerArray support_vox_visu;
      sensor_msgs::PointCloud occupancy_pc;
      visualization_msgs::Marker path_visu;
      visualization_msgs::Marker command_visu;
      visualization_msgs::Marker debug_visu;
      visualization_msgs::MarkerArray debug_array_visu;
    };

    // DESCRIPTION: Constructor
    Tentabot( NodeHandle& nh,
              tf::TransformListener* listener,
              GoalUtility& gu,
              RobotParams& rp,
              ProcessParams& pp,
              OffTuningParams& offtp,
              OnTuningParams& ontp,
              string data_path);
    
    // DESCRIPTION: Destructor
    ~Tentabot();

    // DESCRIPTION: TODO...
    ProcessParams getProcessParams();

    // DESCRIPTION: TODO...
    RobotParams getRobotParams();

    // DESCRIPTION: TODO...
    OffTuningParams getOffTuningParams();

    // DESCRIPTION: TODO...
    OnTuningParams getOnTuningParams();

    // DESCRIPTION: TODO...
    HeuristicParams getHeuristicParams();

    // DESCRIPTION: TODO...
    StatusParams getStatusParams();

    // DESCRIPTION: TODO...
    VisuParams getVisuParams();

    // DESCRIPTION: TODO...
    void setProcessParams(ProcessParams new_process_param);

    // DESCRIPTION: TODO...
    void setRobotParams(RobotParams new_robot_param);

    // DESCRIPTION: TODO...
    void setOffTuningParams(OffTuningParams new_off_tuning_param);

    // DESCRIPTION: TODO...
    void setOnTuningParams(OnTuningParams new_on_tuning_param);

    // DESCRIPTION: TODO...
    void setHeuristicParams(HeuristicParams new_heuristic_param);

    // DESCRIPTION: TODO...
    void setStatusParams(StatusParams new_status_param);

    // DESCRIPTION: TODO...
    void setVisuParams(VisuParams new_visu_param);

    // DESCRIPTION: TODO...
    void setDataPath(string data_path);

    // DESCRIPTION: TODO...
    void clearTentacleData();

    // DESCRIPTION: TODO...
    void clearSupportVoxData();

    // DESCRIPTION: TODO...
    void publishTentabot();

    // DESCRIPTION: TODO...
    void publishRobot();

    // DESCRIPTION: TODO...
    void publishTentacleTsampSupportvox();

    // DESCRIPTION: TODO...
    void publishOccupancy();

    // DESCRIPTION: TODO...
    void publishPath();

    // DESCRIPTION: TODO...
    void publishCommand();

    // DESCRIPTION: TODO...
    void publishDebugVisu();

    // DESCRIPTION: TODO...
    void publishDebugArrayVisu();

    // DESCRIPTION: TODO...
    void transformPoint(string frame_from,
                        string frame_to,
                        geometry_msgs::Point& p_to_msg);

    // DESCRIPTION: TODO...
    void transformPoint(string frame_from, 
                        geometry_msgs::Point& p_from_msg, 
                        string frame_to, 
                        geometry_msgs::Point& p_to_msg);

    // DESCRIPTION: TODO...
    void transformPoint(string frame_from, 
                        geometry_msgs::Point32& p_from_msg, 
                        string frame_to, 
                        geometry_msgs::Point32& p_to_msg);

    // DESCRIPTION: TODO...
    void transformOrientation(string frame_from, 
                              geometry_msgs::Quaternion& q_from_msg, 
                              string frame_to, 
                              geometry_msgs::Quaternion& q_to_msg);

    // DESCRIPTION: TODO...
    void transformOrientation(string frame_from, 
                              double roll_from, 
                              double pitch_from, 
                              double yaw_from, 
                              string frame_to, 
                              geometry_msgs::Quaternion& q_to_msg);

    // DESCRIPTION: TODO...
    void transformPC2(const sensor_msgs::PointCloud2& pc2_from, sensor_msgs::PointCloud2& pc2_to);

    // DESCRIPTION: TODO...
    void mapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

    // DESCRIPTION: TODO...
    //void pc2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void odometryCallback(const nav_msgs::Odometry& msg);

    // DESCRIPTION: TODO...
    void mainCallback(const ros::TimerEvent& e);

    // DESCRIPTION: TODO...
    bool rl_step(tentabot::rl_step::Request &req, tentabot::rl_step::Response &res);

    // DESCRIPTION: TODO...
    bool update_goal(tentabot::update_goal::Request &req, tentabot::update_goal::Response &res);

  private:

    tf::TransformListener* tflistener;
    //MapUtility map_util;
    GoalUtility goal_util;
    RobotParams robot_param;
    ProcessParams process_param;
    OffTuningParams off_tuning_param;
    OnTuningParams on_tuning_param;
    HeuristicParams heuristic_param;
    StatusParams status_param;
    VisuParams visu_param;
    string data_path;
    string data_tag;
    ofstream input_data_stream;
    ofstream pre_data_stream;
    ofstream mid_data_stream;
    ofstream post_data_stream;
    
    // DESCRIPTION: TODO...
    int toIndex(double pos, int grid_vnum);

    // DESCRIPTION: TODO...
    int toIndex(double pos, int grid_vnum_min, int grid_vnum_max);

    // DESCRIPTION: TODO...
    int toLinIndex(tf::Vector3 po);

    // DESCRIPTION: TODO...
    int toLinIndex(geometry_msgs::Point po);

    // DESCRIPTION: TODO...
    int toLinIndex(geometry_msgs::Point32 po);

    // DESCRIPTION: TODO...
    void toPoint(int ind, geometry_msgs::Point& po);

    // DESCRIPTION: TODO...
    void toPoint(int ind, geometry_msgs::Point32& po);

    // DESCRIPTION: TODO...
    void initialize(NodeHandle& nh);

    // DESCRIPTION: TODO...
    vector<OccupancyVoxel> extract_priority_support_voxels_by_bbx(vector<geometry_msgs::Point>& traj);

    // DESCRIPTION: TODO...
    vector<OccupancyVoxel> extract_priority_support_voxels_by_radius(vector<geometry_msgs::Point>& traj);

    // DESCRIPTION: TODO...
    void calculate_tentacle_length_and_bbx_data();

    // DESCRIPTION: TODO...
    void construct_ego_grid_data();

    // DESCRIPTION: TODO...
    void construct_priority_support_voxel_data();

    // DESCRIPTION: TODO...
    void construct_sample_weight_data();

    // DESCRIPTION: TODO...
    void construct_tentacle_weight_data();

    // DESCRIPTION: TODO...
    double calculate_between_tentacle_closeness(vector<geometry_msgs::Point>& tentacle1, vector<geometry_msgs::Point>& tentacle2);

    // DESCRIPTION: TODO...
    void sort_tentacles(string sort_type="y");

    // DESCRIPTION: TODO...
    void fillRobotVisu();

    // DESCRIPTION: TODO...
    void fillTentacleTsampSupportvoxVisu();

    // DESCRIPTION: TODO...
    void fillBestTentacleVisu();

    // DESCRIPTION: TODO...
    void fillPathVisu();

    // DESCRIPTION: TODO...
    void fillCommandVisu();

    // DESCRIPTION: TODO...
    void fillDebugVisu(geometry_msgs::Point p, string frame_name);

    // DESCRIPTION: TODO...
    void fillDebugArrayVisu(vector<geometry_msgs::Point> v, string frame_name);

    // DESCRIPTION: TODO...
    void interpol(geometry_msgs::Point p, double dist, tf::Vector3& pint);

    // DESCRIPTION: TODO...
    geometry_msgs::Point interpol(geometry_msgs::Point p, double dist);

    // DESCRIPTION: TODO...
    geometry_msgs::Point interpol(geometry_msgs::Point p1, 
                                  geometry_msgs::Point p2, 
                                  double dist_from_p1);

    // DESCRIPTION: TODO...
    vector<geometry_msgs::Point> equadistant(geometry_msgs::Point p1, 
                                             geometry_msgs::Point p2, 
                                             int num_btw);

    // DESCRIPTION: TODO...UPDATE LOCAL MAP
    //void update_local_map();

    // DESCRIPTION: TODO...
    bool isOccupied(double x, double y, double z);

    // DESCRIPTION: TODO...
    void update_planning_states();

    // DESCRIPTION: TODO...UPDATE OCCUPANCY GRID DATA OF THE ROBOT
    void update_ego_grid_data();

    // DESCRIPTION: TODO...
    void update_heuristic_values();

    // DESCRIPTION: TODO...SELECT THE BEST TENTACLE
    void select_best_tentacle();

    // DESCRIPTION: TODO...MOVE THE ROBOT
    void debug_rotate();

    // DESCRIPTION: TODO...MOVE THE ROBOT
    void send_motion_command_by_pose_control();

    // DESCRIPTION: TODO...MOVE THE ROBOT
    void send_motion_command_by_velocity_control();

}; // END of class Tentabot

#endif