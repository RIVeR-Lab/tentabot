// LAST UPDATE: 2021.03.20
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
#include <tf/transform_broadcaster.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <map>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d_optimization.h>

// --CUSTOM LIBRARIES--
#include "common_utility.h"
#include "map_utility.h"
#include "goal_utility.h"

// --NAMESPACES--
using namespace std;
using namespace ros;

// --GLOBAL VARIABLES--
#define PI 3.141592653589793
#define INF std::numeric_limits<double>::infinity()
#define INFINT std::numeric_limits<int>::max()
#define FMAX std::numeric_limits<float>::max()
#define FMIN std::numeric_limits<float>::min()

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
      bool navexit_flag;                            // TODO: Review: flag to exit from the navigation loop  
      double goal_close_threshold;                  // TODO: Review: threshold distance from goal to robot position in the global coordinate frame, range: 0 <= goal_close_threshold, E R+
      int counter;
    };

    // DESCRIPTION: TODO...
    struct NavSensor
    {
      vector<string> name = vector<string>(5);
      double freq;                                  // TODO: Review: robot's navigation sensor (lidar, camera, etc.) sampling time, unit: s, range: 0 < nav_sensor_freq, E R+
      double resolution;                            // TODO: Review: resolution of robot's navigation sensor (lidar, camera, etc.), unit: m, range: 0 < nav_sensor_resolution, E R+
      vector<double> range_x = vector<double>(2);   // TODO: Review: maximum range of robot's navigation sensor (lidar, camera, etc.) in x, unit: m, range: 0 < nav_sensor_max_range_x, E R+
      vector<double> range_y = vector<double>(2);   // TODO: Review: maximum range of robot's navigation sensor (lidar, camera, etc.) in y, unit: m, range: 0 < nav_sensor_max_range_y, E R+
      vector<double> range_z = vector<double>(2);   // TODO: Review: maximum range of robot's navigation sensor (lidar, camera, etc.) in z, unit: m, range: 0 < nav_sensor_max_range_z, E R+
      geometry_msgs::Pose pose_wrt_robot;           // TODO: Review: position and orientation of the navigation sensor of the robot with respect to the robot's coordinate system
      string frame_name;                            // TODO: Review: name of the navigation sensor frame
    };

    // DESCRIPTION: TODO...
    struct RobotParams
    {
      double width;                                             // TODO: Review: width of the robot, unit: m, range: 0 < width, E R+
      double length;                                            // TODO: Review: length of the robot, unit: m, range: 0 < length, E R+
      double height;                                            // TODO: Review: height of the robot, unit: m, range: 0 < height, E R+
      double dummy_max_lat_velo;                                // TODO: Review: max forward lateral velocity of the robot, unit: m/s, range: 0 < dummy_max_lat_velo, E R+
      double dummy_max_lat_acc;                                 // TODO: Review: max forward lateral acceleration of the robot, unit: m/s^2, range: 0 < dummy_max_lat_acc, E R+
      double dummy_max_yaw_velo;                                // TODO: Review: max yaw angle velocity of the robot, unit: rad/s, range: 0 < dummy_max_yaw_velo, E R+
      double dummy_max_yaw_acc;                                 // TODO: Review: max yaw angle acceleration of the robot, unit: rad/s^2, range: 0 < dummy_max_yaw_acc, E R+
      double dummy_max_pitch_velo;                              // TODO: Review: max pitch angle velocity of the robot, unit: rad/s, range: 0 < dummy_max_pitch_velo, E R+
      double dummy_max_roll_velo;                               // TODO: Review: max roll angle velocity of the robot, unit: rad/s, range: 0 < dummy_max_roll_velo, E R+
      geometry_msgs::Pose init_robot_pose;                      // TODO: Review: position and orientation of the robot's initial pose with respect to global coordinate system
      string robot_frame_name;                                  // TODO: Review: name of the robot frame
      string robot_name;                                        // TODO: Review: name of the robot
      vector< vector<geometry_msgs::Point> > tentacle_data;
      vector<string> right_left_data;
      vector< vector<OccupancyVoxel> > support_vox_data;
      NavSensor nav_sensor;
    };

    // DESCRIPTION: TODO...
    struct OffTuningParams
    {
      int tyaw_cnt;                                 // TODO: Review: number of tentacles along yaw direction, range: 1 <= tyaw_cnt, E Z+
      int tpitch_cnt;                               // TODO: Review: number of tentacles along pitch direction, range: 1 <= tpitch_cnt, E Z+
      int troll_cnt;                                // TODO: Review: number of tentacles along roll direction, range: 1 <= troll_cnt, E Z+
      int tsamp_cnt;                                // TODO: Review: number of sample points on the tentacle, range: 1 <= tsamp_cnt, E Z+
      int tlat_velo_cnt;                            // TODO: Review: number of sample between zero to max lateral velocity, range: 1 <= tlat_velo_cnt, E Z+
      int tyaw_velo_cnt;                            // TODO: Review: number of sample between zero to max yaw angle velocity, range: 1 <= tyaw_velo_cnt, E Z+
      int tpitch_velo_cnt;                          // TODO: Review: number of sample between zero to max pitch angle velocity, range: 1 <= tpitch_velo_cnt, E Z+
      int troll_velo_cnt;                           // TODO: Review: number of sample between zero to max roll angle velocity, range: 1 <= troll_velo_cnt, E Z+
      double tlen;
      double tyaw;
      double tpitch;
      double troll;
      string tentacle_type;       
      string tyaw_samp_type;                        // parameter to adjust yaw angle sampling type of tentacles  
      string tpitch_samp_type;                      // parameter to adjust pitch angle sampling type of tentacles 
      string troll_samp_type;                       // parameter to adjust roll angle sampling type of tentacles
      double pdist;                                 // priority distance, unit: m, range: 0 < cdist, E R+
      double sdist;                                 // support distance, unit: m, range: cdist < sdist, E R+
      double sweight_max;                           // max weight of support cells (= for priority cells) which affects drivability of a tentacle calculation based on closest tentacle sample point, range: 0 < cweight_max, E R+ 
      double sweight_scale;                         // parameter to adjust weight of support cells which affects drivability of a tentacle calculation based on closest tentacle sample point, range: 0 < cweight_scale, E R+ 
      double egrid_vdim;                            // dimension of the voxel in the ego-grid, unit: m, range: 0 < cdim, E R+
      int egrid_vnumx;                              // number of voxel in the ego-grid along x direction, range: 1 <= cnumx, E Z+
      int egrid_vnumy;                              // number of voxel in the ego-grid along y direction, range: 1 <= cnumy, E Z+  
      int egrid_vnumz;                              // number of voxel in the ego-grid along z direction, range: 1 <= cnumz, E Z+ 
    };

    // DESCRIPTION: TODO...
    struct OnTuningParams
    {
      int tbin_window;                              // number of tentacle bins in sliding window to decide whether the number of obstacles (histogram) correlate within consecutive bins, range: 1 <= bin_window, E Z+
      int tbin_obs_cnt_threshold;                   // threshold of number of obstacle on the tentacle bin, range: 0 <= obs_num_threshold, E Z+
      double clear_scale;                           // parameter to adjust the weight of the clearance value while selecting best tentacle, range: 0 <= clear_scale, E R+
      double clutter_scale;                         // parameter to adjust the weight of the clutterness value while selecting best tentacle, range: 0 <= clutter_scale, E R+,
      double close_scale;                           // parameter to adjust the weight of the closeness value while selecting best tentacle, range: 0 <= close_scale, E R+
      double smooth_scale;                          // parameter to adjust the weight of the smoothness value while selecting best tentacle, range: 0 <= smooth_scale, E R+
      double crash_dist_scale;                      // crash distance on the selected tentacle, range: 0 <= crash_dist, E R+
    };

    // DESCRIPTION: TODO...
    struct HeuristicParams
    {
      vector<int> navigability_set;
      vector<double> clearance_set;
      vector<double> clutterness_set;
      vector<double> closeness_set;
      vector<double> smoothness_set;
    };

    // DESCRIPTION: TODO...
    struct StatusParams
    {
      geometry_msgs::Pose prev_robot_pose;                      // TODO: Review: previous position and orientation of the robot with respect to global coordinate system
      geometry_msgs::Pose robot_pose;                           // TODO: Review: position and orientation of the robot with respect to global coordinate system
      geometry_msgs::PoseStamped robot_pose_command;            // TODO: Review: position and orientation command of the robot with respect to global coordinate system
      EgoGrid ego_grid_data;
      std::vector<int> tcrash_bin;
      bool navigability_flag;
      int best_tentacle;
      int ex_best_tentacle;
      int ex_best_sample;
      int nav_result;
      double nav_length;
      double nav_duration;                                      // TODO: Review: navigation duration
      ros::Time prev_action_time;
      ros::Publisher command_pub;
      ros::Publisher command_point_pub;
      ros::Time tmp_time;
      int speed_counter;
      double dummy_current_speed;
    };

    // DESCRIPTION: TODO...
    struct VisuParams
    {
      ros::Publisher robot_visu_pub;                            // TODO: Review: publisher for robot visualization 
      ros::Publisher tentacle_visu_pub;
      ros::Publisher tsamp_visu_pub;
      ros::Publisher support_vox_visu_pub;
      ros::Publisher occupancy_pc_pub;
      ros::Publisher command_visu_pub;
      ros::Publisher path_visu_pub;
      ros::Publisher next_pub;

      visualization_msgs::Marker robot_visu;                    // TODO: Review: marker for robot visualization
      visualization_msgs::MarkerArray tentacle_visu;
      visualization_msgs::MarkerArray opt_tentacle_visu;
      visualization_msgs::MarkerArray tsamp_visu;
      visualization_msgs::MarkerArray support_vox_visu;
      sensor_msgs::PointCloud occupancy_pc;
      visualization_msgs::Marker path_visu;
      visualization_msgs::Marker command_visu;
    };

    // DESCRIPTION: Constructor
    Tentabot(NodeHandle& nh, 
             tf::TransformListener* listener, 
             ProcessParams& pp, 
             RobotParams& rp, 
             OffTuningParams& offtp, 
             OnTuningParams& ontp, 
             MapUtility& mu, 
             GoalUtility& gu);
    
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
    ofstream& getNavParamBench();

    // DESCRIPTION: TODO...
    ofstream& getNavPreBench();

    // DESCRIPTION: TODO...
    ofstream& getNavProcessBench();

    // DESCRIPTION: TODO...
    ofstream& getNavResultBench();

    // DESCRIPTION: TODO...
    ofstream& getRLBench();

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
    void setEgoGridData();

    // DESCRIPTION: TODO...
    void setOnTuning(bool flag);

    // DESCRIPTION: TODO...
    void print_vecvecd(vector< vector<double> > vecvec);

    // DESCRIPTION: TODO...
    void print_vecvecPoint(vector< vector<geometry_msgs::Point> > vecvec);

    // DESCRIPTION: TODO...
    void clearTentacleData();

    // DESCRIPTION: TODO...
    void clearSupportVoxData();

    // DESCRIPTION: TODO...
    void publishTentabot();

    // DESCRIPTION: TODO...UPDATE OCCUPANCY GRID CELL FOR THE ROBOT
    void updateOccupancyVox();

    // DESCRIPTION: TODO...
    void updateHeuristicFunctions();

    // DESCRIPTION: TODO...SELECT THE BEST TENTACLE
    void selectBestTentacle();

    // DESCRIPTION: TODO...
    void hoverTentabotAtZ1(double x, double y);

    // DESCRIPTION: TODO...MOVE THE ROBOT
    void moveTentabot_extend();

    // DESCRIPTION: TODO...
    void sendCommandCallback(const ros::TimerEvent& e);

    // DESCRIPTION: TODO...
    void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void odometryCallback(const geometry_msgs::Pose::ConstPtr& msg);

  private:

    ProcessParams process_param;
    RobotParams robot_param;
    OffTuningParams off_tuning_param;
    OnTuningParams on_tuning_param;
    HeuristicParams heuristic_param;
    StatusParams status_param;
    GoalUtility goal_util;
    MapUtility map_util;
    VisuParams visu_param;
    ofstream nav_param_bench;
    ofstream nav_pre_bench;
    ofstream nav_process_bench;
    ofstream nav_result_bench;
    ofstream rl_bench;
    tf::TransformListener* tflistener;
    vector<geometry_msgs::Point> command_history;

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
    int toIndex(double pos, int grid_vnum);

    // DESCRIPTION: TODO...
    int toLinIndex(geometry_msgs::Point po);

    // DESCRIPTION: TODO...
    int toLinIndex(geometry_msgs::Point32 po);

    // DESCRIPTION: TODO...
    void toPoint(int ind, geometry_msgs::Point& po);

    // DESCRIPTION: TODO...
    void toPoint(int ind, geometry_msgs::Point32& po);

    // DESCRIPTION: TODO...
    bool isInsideRectCuboid(geometry_msgs::Point32 po);

    // DESCRIPTION: TODO...
    bool isInsideRectCuboid(geometry_msgs::Point32 po, geometry_msgs::Point center);

    // DESCRIPTION: TODO...
    bool isInsideTriangle(double x, double y, double edge_length, double half_angle);

    // DESCRIPTION: TODO...
    vector<geometry_msgs::Point> arc_by_radi_ang(geometry_msgs::Point from, 
                                                 double radius, 
                                                 double angle, 
                                                 int sample_cnt);

    // DESCRIPTION: TODO...
    vector<geometry_msgs::Point> line_by_len_ang(geometry_msgs::Point from, 
                                                 double length, 
                                                 double angle, 
                                                 int sample_cnt);

    // DESCRIPTION: TODO...
    geometry_msgs::Point rotate3d(geometry_msgs::Point po, 
                                  double ang, 
                                  string rot_type);

    // DESCRIPTION: TODO...
    vector<OccupancyVoxel> voxel_extractor(vector<geometry_msgs::Point>& archy);

    // DESCRIPTION: TODO...
    void arc_extender_voxel_extractor(vector<geometry_msgs::Point>& planar_tentacle_data, 
                                      double yaw_sample, 
                                      vector<double> extend_ang_samples, 
                                      string rot_type, 
                                      double yaw_offset=0);

    // TODO: Do it for the roll as well.
    // DESCRIPTION: TODO...CONSTRUCT TENTACLES AND EXTRACT SUPPORT/PRIORITY VOXELS (DEPRECATED)
    void construct_tentacle_pitchExt();

    // DESCRIPTION: TODO...
    void construct_tentacle_extend(bool no_restriction=false);

    // DESCRIPTION: TODO...
    void fillRobotVisu();

    // DESCRIPTION: TODO...
    void publishRobot();

    // DESCRIPTION: TODO...
    void fillTentacleTsampSupportvoxVisu();

    // DESCRIPTION: TODO...
    void publishTentacleTsampSupportvox();

    // DESCRIPTION: TODO...
    void publishOccupancy();

    // DESCRIPTION: TODO...
    void fillPathVisu();

    // DESCRIPTION: TODO...
    void fillCommandVisu();

    // DESCRIPTION: TODO...
    void publishPath();

    // DESCRIPTION: TODO...
    void publishCommand();

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
}; // END of class Tentabot