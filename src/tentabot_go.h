// LAST UPDATE: 2021.03.01
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION:
//
// REFERENCES:
// [1] F. von Hundelshausen, M. Himmelsbach, F. Hecker, A. Mueller, and 
//     H.-J. Wuensche. Driving with tentacles: Integral structures for 
//     sensing and motion. Journal of Field Robotics, 25(9):640–673, 2008.

// OUTSOURCE LIBRARIES:
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

// CUSTOM LIBRARIES:
#include "/home/akmandor/catkin_ws/src/tentabot/src/map_utility.h"
#include "/home/akmandor/catkin_ws/src/tentabot/src/goal_utility.h"

// GLOBAL VARIABLES:
#define PI 3.141592653589793
#define INF std::numeric_limits<double>::infinity()
#define INFINT std::numeric_limits<int>::max()
#define FMAX std::numeric_limits<float>::max()
#define FMIN std::numeric_limits<float>::min()

// STRUCTURES:
struct OccupancyVoxel
{
  int index = 0;                                // index number of the support voxel in the linearized 3D grid, range: 0 <= index, E Z+
  double weight = 0;                            // weight of the support voxel which is calculated based on the distance from the closest tentacle sample point, range: 0 <= weight, E R+ 
  int histbin = 0;                              // occupancy histogram bin index on the tentacle whose location is the closest to the support voxel, range: 0 <= histbin, E Z+ 
  bool flag = false;                            // flag that determines if the occupancy voxel is either classification voxel (false) or priority voxel (true)
};

struct EgoGrid
{
  vector<geometry_msgs::Point> ovox_pos;        // vector that keeps occupancy voxel positions in the ego-grid 
  vector<double> ovox_value;                    // vector that keeps occupancy voxel values (obtained by navigation sensor) in the ego-grid
};

struct ProcessParams
{
  bool visu_flag;                               // flag to display simulation (setting and publishing markers etc.)
  bool online_tuning_flag;                      // flag to auto-tune online parameters
  double time_limit;                            // time limit for the navigation, unit: s, range: 0 < time_limit, E R+
  double nav_dt;                          			// frequency of ROS loop
  bool navexit_flag;                            // flag to exit from the navigation loop  
  double goal_close_threshold;                  // threshold distance from goal to robot position in the global coordinate frame, range: 0 <= goal_close_threshold, E R+
  int counter;
};

struct NavSensor
{
	vector<string> name = vector<string>(5);
  double freq;                                  // robot's navigation sensor (lidar, camera, etc.) sampling time, unit: s, range: 0 < nav_sensor_freq, E R+
  double resolution;                            // resolution of robot's navigation sensor (lidar, camera, etc.), unit: m, range: 0 < nav_sensor_resolution, E R+
  vector<double> range_x = vector<double>(2);   // maximum range of robot's navigation sensor (lidar, camera, etc.) in x, unit: m, range: 0 < nav_sensor_max_range_x, E R+
  vector<double> range_y = vector<double>(2);   // maximum range of robot's navigation sensor (lidar, camera, etc.) in y, unit: m, range: 0 < nav_sensor_max_range_y, E R+
  vector<double> range_z = vector<double>(2);   // maximum range of robot's navigation sensor (lidar, camera, etc.) in z, unit: m, range: 0 < nav_sensor_max_range_z, E R+
  geometry_msgs::Pose pose_wrt_robot;           // position and orientation of the navigation sensor of the robot with respect to the robot's coordinate system
  string frame_name;                            // name of the navigation sensor frame
};

struct RobotParams
{
  double width;                                 						// width of the robot, unit: m, range: 0 < width, E R+
  double length;                                						// length of the robot, unit: m, range: 0 < length, E R+
  double height;                                						// height of the robot, unit: m, range: 0 < height, E R+
  double dummy_max_lat_velo;                          						// max forward lateral velocity of the robot, unit: m/s, range: 0 < dummy_max_lat_velo, E R+
  double dummy_max_lat_acc;                                       // max forward lateral acceleration of the robot, unit: m/s^2, range: 0 < dummy_max_lat_acc, E R+
  double dummy_max_yaw_velo;                          						// max yaw angle velocity of the robot, unit: rad/s, range: 0 < dummy_max_yaw_velo, E R+
  double dummy_max_yaw_acc;                                       // max yaw angle acceleration of the robot, unit: rad/s^2, range: 0 < dummy_max_yaw_acc, E R+
  double dummy_max_pitch_velo;                        						// max pitch angle velocity of the robot, unit: rad/s, range: 0 < dummy_max_pitch_velo, E R+
  double dummy_max_roll_velo;                         						// max roll angle velocity of the robot, unit: rad/s, range: 0 < dummy_max_roll_velo, E R+
  geometry_msgs::Pose init_robot_pose;          						// position and orientation of the robot's initial pose with respect to global coordinate system
  string robot_frame_name;                      						// name of the robot frame
  string robot_name;                            						// name of the robot
  vector< vector<geometry_msgs::Point> > tentacle_data;
  vector<string> right_left_data;
  vector< vector<OccupancyVoxel> > support_vox_data;
  NavSensor nav_sensor;
};

struct OffTuningParams
{
  int tyaw_cnt;                                 // number of tentacles along yaw direction, range: 1 <= tyaw_cnt, E Z+
  int tpitch_cnt;                               // number of tentacles along pitch direction, range: 1 <= tpitch_cnt, E Z+
  int troll_cnt;                                // number of tentacles along roll direction, range: 1 <= troll_cnt, E Z+
  int tsamp_cnt;                                // number of sample points on the tentacle, range: 1 <= tsamp_cnt, E Z+
  int tlat_velo_cnt;                            // number of sample between zero to max lateral velocity, range: 1 <= tlat_velo_cnt, E Z+
  int tyaw_velo_cnt;                            // number of sample between zero to max yaw angle velocity, range: 1 <= tyaw_velo_cnt, E Z+
  int tpitch_velo_cnt;                          // number of sample between zero to max pitch angle velocity, range: 1 <= tpitch_velo_cnt, E Z+
  int troll_velo_cnt;                           // number of sample between zero to max roll angle velocity, range: 1 <= troll_velo_cnt, E Z+
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

struct OnTuningParams
{
  int tbin_window;                              // number of tentacle bins in sliding window to decide whether the number of obstacles (histogram) correlate within consecutive bins, range: 1 <= bin_window, E Z+
  int tbin_obs_cnt_threshold;                   // threshold of number of obstacle on the tentacle bin, range: 0 <= obs_num_threshold, E Z+
  double clear_scale;                           // parameter to adjust the weight of the clearance value while selecting best tentacle, range: 0 <= clear_scale, E R+
  double clutter_scale;                            // parameter to adjust the weight of the clutterness value while selecting best tentacle, range: 0 <= clutter_scale, E R+, NUA TODO: Change to clutterness
  double close_scale;                           // parameter to adjust the weight of the closeness value while selecting best tentacle, range: 0 <= close_scale, E R+
  double smooth_scale;                          // parameter to adjust the weight of the smoothness value while selecting best tentacle, range: 0 <= smooth_scale, E R+
  double crash_dist;                            // crash distance on the selected tentacle, range: 0 <= crash_dist, E R+
};

struct HeuristicParams
{
  vector<int> navigability_set;
  vector<double> clearance_set;
  vector<double> clutterness_set;
  vector<double> closeness_set;
  vector<double> smoothness_set;
};

struct StatusParams
{
  geometry_msgs::Pose prev_robot_pose;                   		// previous position and orientation of the robot with respect to global coordinate system
  geometry_msgs::Pose robot_pose;                           // position and orientation of the robot with respect to global coordinate system
  geometry_msgs::PoseStamped robot_pose_command;            // position and orientation command of the robot with respect to global coordinate system
  EgoGrid ego_grid_data;
  std::vector<int> tcrash_bin;
  bool navigability_flag;
  int best_tentacle;
  int ex_best_tentacle;
  int ex_best_sample;
  int nav_result;
  double nav_length;
  double nav_duration;                                      // navigation duration
  ros::Time prev_action_time;
  ros::Publisher command_pub;
  ros::Publisher command_point_pub;
  ros::Time tmp_time;
  int speed_counter;
  double dummy_current_speed;
};

struct VisuParams
{
  ros::Publisher robot_visu_pub;                						// publisher for robot visualization 
  ros::Publisher tentacle_visu_pub;
  ros::Publisher opt_tentacle_visu_pub;
  ros::Publisher tsamp_visu_pub;
  ros::Publisher support_vox_visu_pub;
  ros::Publisher occupancy_pc_pub;
  ros::Publisher command_visu_pub;
  ros::Publisher path_visu_pub;
  ros::Publisher next_pub;

  visualization_msgs::Marker robot_visu;        						// marker for robot visualization
  visualization_msgs::MarkerArray tentacle_visu;
  visualization_msgs::MarkerArray opt_tentacle_visu;
  visualization_msgs::MarkerArray tsamp_visu;
  visualization_msgs::MarkerArray support_vox_visu;
  sensor_msgs::PointCloud occupancy_pc;
  visualization_msgs::Marker path_visu;
  visualization_msgs::Marker command_visu;
};

// GENERAL FUNCTIONS:
vector<double> sampling_func(double mini, double maxi, int snum, string stype="linear", bool included=true)
{
  if(snum <= 2)
  { 
    vector<double> v;
    if (included)
    {
      v.push_back(mini);
      v.push_back(maxi);
    }
    return v;
  }
  else
  {
    int i;
    double srise;
    double srate;
    double s;
    vector<double> v;
    
    if (included)
    {
      v.resize(snum);
    }
    else
    {
      v.resize(snum-2);
    }
    
    if(stype == "increase")
    {
      if (included)
      {
        v[0] = mini;
      }
      s = mini;
      srise = (maxi - mini) / (snum-1);
      for(i = 0; i < snum-2; i++)
      {
        srate = 2 * srise * (i+1) / (snum-2);
        s = s + srate;  
        
        if (included)
        {
          v[i+1] = s;
        }
        else
        {
          v[i] = s;
        }
      }
      if (included)
      {
        v[snum-1] = maxi;
      }
    }
    else if(stype == "decrease")
    {
      int border = snum-3;
      if (included)
      {
        v.push_back(mini);
        border = snum-2;
      }
      
      s = maxi;
      srise = (maxi - mini) / (snum-1);
      for(i = 0; i < border; i++)
      {
        srate = 2 * srise * (i+1) / (snum-2);
        s = s - srate;  
        v[snum-2-i] = s;
      }
      if (included)
      {
        v[snum-1] = maxi;
      }
    }
    else
    {
      if (included)
      {
        v[0] = mini;
      }
      
      s = mini;
      srise = (maxi - mini) / (snum-1);
      for(i = 0; i < snum-2; i++)
      {
        s = s + srise;
        if (included)
        {
          v[i+1] = s;
        }
        else
        {
          v[i] = s;
        }
      }
      if (included)
      {
        v[snum-1] = maxi;
      }
    }
    return v;
  }
}

vector<double> find_limits(vector<geometry_msgs::Point>& archy)
{
  vector<double> v(6);  // [0]: min_x, [1]: max_x, [2]: min_y, [3]: max_y, [4]: min_z, [5]: max_z
  v[0] = archy[0].x;
  v[1] = archy[0].x;
  v[2] = archy[0].y;
  v[3] = archy[0].y;
  v[4] = archy[0].z;
  v[5] = archy[0].z;

  for(int i = 1; i < archy.size(); i++)
  {
    if(v[0] > archy[i].x)
    {
      v[0] = archy[i].x;
    }

    if(v[1] < archy[i].x)
    {
      v[1] = archy[i].x;
    }

    if(v[2] > archy[i].y)
    {
      v[2] = archy[i].y;
    }

    if(v[3] < archy[i].y)
    {
      v[3] = archy[i].y;
    }

    if(v[4] > archy[i].z)
    {
      v[4] = archy[i].z;
    }

    if(v[5] < archy[i].z)
    {
      v[5] = archy[i].z;
    }
  }
  return v;
}

double find_norm(geometry_msgs::Point p)
{
  return ( sqrt( pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2) ) );
}

double find_Euclidean_distance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return ( sqrt( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2) ) );
}

double find_Euclidean_distance(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
  return ( sqrt( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2) ) );
}

vector<double> find_closest_dist(vector<geometry_msgs::Point>& archy, geometry_msgs::Point cell_center)
{
  double temp_dist;
  vector<double> v(2);
  v[0] = find_Euclidean_distance(archy[0], cell_center);
  v[1] = 0;
  for(int i = 1; i < archy.size(); i++)
  {
    temp_dist = find_Euclidean_distance(archy[i], cell_center);
    if(v[0] > temp_dist)
    {
      v[0] = temp_dist;
      v[1] = i;
    }
  }
  return v;
}

void print(vector<int>& vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    if(i == vsize-1)
    {
      cout << i << ") " << vec[i] << endl;;
    }
    else
    {
      cout << i << ") " << vec[i] << ", ";
    }   
  }
}

void print(vector<double>& vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    if(i == vsize-1)
    {
      cout << i << ") " << vec[i] << endl;;
    }
    else
    {
      cout << i << ") " << vec[i] << ", ";
    }   
  }
}

void print(vector< vector<int> >& vecvec)
{
  int count = 0;
  int vsize1 = vecvec.size();
  for(int i = 0; i < vsize1; i++)
  {
    int vsize2 = vecvec[i].size();
    for(int j = 0; j < vsize2; j++)
    {
      cout << count << ") " << vecvec[i][j] << endl;
      count++;
    }
  }
}

void print(vector<geometry_msgs::Point> vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    cout << i << ": (" << vec[i].x << ", " << vec[i].y << ", " << vec[i].z << ")" << endl; 
  }
}

void print(vector<geometry_msgs::Point32> vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    cout << i << ": (" << vec[i].x << ", " << vec[i].y << ", " << vec[i].z << ")" << endl; 
  }
}

void print(geometry_msgs::Point po)
{
  cout << "(" << po.x << ", " << po.y << ", " << po.z << ")" << endl; 
}



void vec2msg(vector< vector<double> >& vecvec, std_msgs::Float64MultiArray& vecvec_msg)
{
  int width = vecvec[0].size();
  int height = vecvec.size();
  for(int i = 0; i < height; i++)
  {
    for(int j = 0; j < width; j++)
    {
      vecvec_msg.data.push_back(vecvec[i][j]);
    }
  }
}

void vec2msg(vector<double>& vec, std_msgs::Float64MultiArray& vec_msg)
{
  int vec_size = vec.size();
  for(int i = 0; i < vec_size; i++)
  {
    vec_msg.data.push_back(vec[i]);
  }
}

string createFileName()
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

class Tentabot
{
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

    ewok::PolynomialTrajectory3D<10>::Ptr polynomial_trajectory;
    ewok::UniformBSpline3DOptimization<6>::Ptr spline_optimization;
    ros::Publisher traj_marker_pub, trajectory_pub, current_traj_pub, tmp_pub;
    std::vector<geometry_msgs::Point> command_history;
    visualization_msgs::MarkerArray global_trajectory_marker;
    visualization_msgs::MarkerArray current_trajectory_marker;
    visualization_msgs::Marker tmp_marker;
    
    void fillTmpMarker()
    {
      // SET TMP POINTS VISUALIZATION SETTINGS
      tmp_marker.ns = "tmp";
      tmp_marker.id = 666;
      tmp_marker.header.frame_id = robot_param.robot_frame_name;
      tmp_marker.type = visualization_msgs::Marker::SPHERE_LIST;
      tmp_marker.action = visualization_msgs::Marker::ADD;
      tmp_marker.pose.orientation.w = 1.0;
      tmp_marker.scale.x = 0.2;
      tmp_marker.scale.y = 0.2;
      tmp_marker.scale.z = 0.2;
      tmp_marker.color.r = 1.0;
      tmp_marker.color.g = 0.0;
      tmp_marker.color.b = 0.0;
      tmp_marker.color.a = 1;
    }

    void transformPoint(string frame_from, geometry_msgs::Point& p_from_msg, string frame_to, geometry_msgs::Point& p_to_msg)
    {
      tf::Point p_from_tf;
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
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
      }

      tf::pointStampedTFToMsg(p_to_stamped_tf, p_to_stamped_msg);
      p_to_msg = p_to_stamped_msg.point;
    }

    void transformPoint(string frame_from, geometry_msgs::Point32& p_from_msg, string frame_to, geometry_msgs::Point32& p_to_msg)
    {
      geometry_msgs::Point tmp_p_from;
      tmp_p_from.x = p_from_msg.x;
      tmp_p_from.y = p_from_msg.y;
      tmp_p_from.z = p_from_msg.z;
      geometry_msgs::Point tmp_p_to;

      transformPoint(frame_from, tmp_p_from, frame_to, tmp_p_to);

      p_to_msg.x = tmp_p_to.x;
      p_to_msg.y = tmp_p_to.y;
      p_to_msg.z = tmp_p_to.z;
    }

    void transformOrientation(string frame_from, geometry_msgs::Quaternion& q_from_msg, string frame_to, geometry_msgs::Quaternion& q_to_msg)
    {
      tf::Quaternion q_from_tf;
      tf::quaternionMsgToTF(q_from_msg, q_from_tf);
      tf::Stamped<tf::Quaternion> q_from_stamped_tf(q_from_tf, ros::Time(0), frame_from);
      tf::Stamped<tf::Quaternion> q_to_stamped_tf;
      geometry_msgs::QuaternionStamped q_to_stamped_msg;

      try
      {
        tflistener -> transformQuaternion(frame_to, q_from_stamped_tf, q_to_stamped_tf);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
      }

      tf::quaternionStampedTFToMsg(q_to_stamped_tf, q_to_stamped_msg);
      q_to_msg = q_to_stamped_msg.quaternion;
    }

    void transformOrientation(string frame_from, double roll_from, double pitch_from, double yaw_from, string frame_to, geometry_msgs::Quaternion& q_to_msg)
    {
      tf::Quaternion q_from_tf;
      q_from_tf.setRPY(roll_from, pitch_from, yaw_from);
      tf::Stamped<tf::Quaternion> q_from_stamped_tf(q_from_tf, ros::Time(0), frame_from);
      tf::Stamped<tf::Quaternion> q_to_stamped_tf;
      geometry_msgs::QuaternionStamped q_to_stamped_msg;

      try
      {
        tflistener -> transformQuaternion(frame_to, q_from_stamped_tf, q_to_stamped_tf);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
      }

      tf::quaternionStampedTFToMsg(q_to_stamped_tf, q_to_stamped_msg);
      q_to_msg = q_to_stamped_msg.quaternion;
    }

		int toIndex(double pos, int grid_vnum)
		{   
		  return (0.5 * grid_vnum + floor(pos / off_tuning_param.egrid_vdim));
		}

		int toLinIndex(geometry_msgs::Point po)
		{
      int grid_vnumx = off_tuning_param.egrid_vnumx;
      int grid_vnumy = off_tuning_param.egrid_vnumy;
      int grid_vnumz = off_tuning_param.egrid_vnumz;

		  int ind_x = toIndex(po.x, grid_vnumx);
		  int ind_y = toIndex(po.y, grid_vnumy);
		  int ind_z = toIndex(po.z, grid_vnumz);

		  return (ind_x + ind_y * grid_vnumx + ind_z * grid_vnumx * grid_vnumy);
		}

    int toLinIndex(geometry_msgs::Point32 po)
    {
      int grid_vnumx = off_tuning_param.egrid_vnumx;
      int grid_vnumy = off_tuning_param.egrid_vnumy;
      int grid_vnumz = off_tuning_param.egrid_vnumz;

      int ind_x = toIndex(po.x, grid_vnumx);
      int ind_y = toIndex(po.y, grid_vnumy);
      int ind_z = toIndex(po.z, grid_vnumz);

      return (ind_x + ind_y * grid_vnumx + ind_z * grid_vnumx * grid_vnumy);
    }

		void toPoint(int ind, geometry_msgs::Point& po)
		{
      int grid_vnumx = off_tuning_param.egrid_vnumx;
      int grid_vnumy = off_tuning_param.egrid_vnumy;
      int grid_vnumz = off_tuning_param.egrid_vnumz;
      double grid_vdim = off_tuning_param.egrid_vdim;
 
		  po.x = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) % grid_vnumx  - 0.5 * grid_vnumx) + 0.5 * grid_vdim;
		  po.y = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) / grid_vnumx  - 0.5 * grid_vnumy) + 0.5 * grid_vdim;
		  po.z = grid_vdim * (ind / (grid_vnumx * grid_vnumy) - 0.5 * grid_vnumz) + 0.5 * grid_vdim;
		}

    void toPoint(int ind, geometry_msgs::Point32& po)
    {
      int grid_vnumx = off_tuning_param.egrid_vnumx;
      int grid_vnumy = off_tuning_param.egrid_vnumy;
      int grid_vnumz = off_tuning_param.egrid_vnumz;
      double grid_vdim = off_tuning_param.egrid_vdim;
 
      po.x = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) % grid_vnumx  - 0.5 * grid_vnumx) + 0.5 * grid_vdim;
      po.y = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) / grid_vnumx  - 0.5 * grid_vnumy) + 0.5 * grid_vdim;
      po.z = grid_vdim * (ind / (grid_vnumx * grid_vnumy) - 0.5 * grid_vnumz) + 0.5 * grid_vdim;
    }

    bool isInsideRectCuboid(geometry_msgs::Point32 po)
    {
      double radx = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumx;
      double rady = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumy;
      double radz = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumz;

      return (po.x >= -radx) && (po.x < radx) && (po.y >= -rady) && (po.y < rady) && (po.z >= -radz) && (po.z < radz);
    }

    bool isInsideRectCuboid(geometry_msgs::Point32 po, geometry_msgs::Point center)
    {
      double radx = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumx;
      double rady = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumy;
      double radz = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumz;

      return (po.x >= center.x - radx) && (po.x < center.x + radx) && (po.y >= center.y - rady) && (po.y < center.y + rady) && (po.z >= center.z - radz) && (po.z < center.z + radz);
    }

    bool isInsideTriangle(double x, double y, double edge_length, double half_angle)
    {
      double angle = atan2(y, x);
      return (abs(angle) <= abs(half_angle)) && (abs(x) <= edge_length * cos(abs(angle)));
    }

    vector<geometry_msgs::Point> arc_by_radi_ang(geometry_msgs::Point from, double radius, double angle, int sample_cnt)
    {
      vector<geometry_msgs::Point> arc;
      double delta_ang = angle / sample_cnt;
      double curr_ang = delta_ang;

      for(int i = 0; i < sample_cnt; i++)
      {
        geometry_msgs::Point apo;
        apo.x = from.x - (radius * abs(angle) / angle) + radius * cos(abs(curr_ang)) * abs(angle) / angle;
        apo.y = from.y + radius * sin(abs(curr_ang));
        arc.push_back(apo);
        curr_ang += delta_ang;
      }
      return arc;
    }

    vector<geometry_msgs::Point> line_by_len_ang(geometry_msgs::Point from, double length, double angle, int sample_cnt)
    {
      vector<geometry_msgs::Point> line;
      double line_dt = length / sample_cnt;

      for(int i = 0; i < sample_cnt; i++)
      {
        geometry_msgs::Point apo;
        apo.x = from.x - line_dt * (i + 1) * sin(abs(angle)) * abs(angle) / angle;
        apo.y = from.y + line_dt * (i + 1) * cos(abs(angle));
        apo.z = 0;
        line.push_back(apo);
      }  
      return line;
    }

    geometry_msgs::Point rotate3d(geometry_msgs::Point po, double ang, string rot_type)
    { 
      geometry_msgs::Point robot_paramo;
      tf::Matrix3x3 rotx(1, 0, 0, 0, cos(ang), -sin(ang), 0, sin(ang), cos(ang));
      tf::Matrix3x3 roty(cos(ang), 0, sin(ang), 0, 1, 0, -sin(ang), 0, cos(ang));
      tf::Matrix3x3 rotz(cos(ang), -sin(ang), 0, sin(ang), cos(ang), 0, 0, 0, 1);
      tf::Vector3 p(po.x, po.y, po.z);
      tf::Vector3 robot_param;
      
      if(rot_type == "pitch")
      {
        robot_param = rotx * p;
      }
      else if(rot_type == "roll")
      {
        robot_param = roty * p;
      }
      else //yaw
      {
        robot_param = rotz * p;
      }
      robot_paramo.x = robot_param.x();
      robot_paramo.y = robot_param.y();
      robot_paramo.z = robot_param.z();
      return robot_paramo;
    }

    vector<OccupancyVoxel> voxel_extractor(vector<geometry_msgs::Point>& archy)
    {
      vector<OccupancyVoxel> svg;

      vector<double> archy_limits = find_limits(archy);
      int min_vox_ind_x = toIndex(archy_limits[0] - off_tuning_param.sdist, off_tuning_param.egrid_vnumx);
      int max_vox_ind_x = toIndex(archy_limits[1] + off_tuning_param.sdist, off_tuning_param.egrid_vnumx);

      if(min_vox_ind_x < 0)
      {
        min_vox_ind_x = 0;
      }

      if(max_vox_ind_x >= off_tuning_param.egrid_vnumx)
      {
        max_vox_ind_x = off_tuning_param.egrid_vnumx - 1;
      }

      int min_vox_ind_y = toIndex(archy_limits[2] - off_tuning_param.sdist, off_tuning_param.egrid_vnumy);
      int max_vox_ind_y = toIndex(archy_limits[3] + off_tuning_param.sdist, off_tuning_param.egrid_vnumy);

      if(min_vox_ind_y < 0)
      {
        min_vox_ind_y = 0;
      }

      if(max_vox_ind_y >= off_tuning_param.egrid_vnumy)
      {
        max_vox_ind_y = off_tuning_param.egrid_vnumy - 1;
      }

      int min_vox_ind_z = toIndex(archy_limits[4] - off_tuning_param.sdist, off_tuning_param.egrid_vnumz);
      int max_vox_ind_z = toIndex(archy_limits[5] + off_tuning_param.sdist, off_tuning_param.egrid_vnumz);

      if(min_vox_ind_z < 0)
      {
        min_vox_ind_z = 0;
      }

      if(max_vox_ind_z >= off_tuning_param.egrid_vnumz)
      {
        max_vox_ind_z = off_tuning_param.egrid_vnumz - 1;
      }

      geometry_msgs::Point cell_center;        
      int gind;
      vector <double> cd;           // [0]: closest distance, [1]: closest tentacle point index
      for(int i = min_vox_ind_x; i <= max_vox_ind_x; i++)
      {
        for(int j = min_vox_ind_y; j <= max_vox_ind_y; j++)
        {
          for(int r = min_vox_ind_z; r <= max_vox_ind_z; r++)
          {
            OccupancyVoxel sv;
            gind = i + j * off_tuning_param.egrid_vnumx + r * off_tuning_param.egrid_vnumx * off_tuning_param.egrid_vnumy;
            cell_center = status_param.ego_grid_data.ovox_pos[gind];
            cd = find_closest_dist(archy, cell_center);

            if(cd[0] <= off_tuning_param.sdist)
            {
              sv.index = gind;     
              sv.histbin = (int) cd[1];
              if(cd[0] <= off_tuning_param.pdist)
              {
                sv.weight = off_tuning_param.sweight_max;
                sv.flag = true;
              }
              else
              {
                sv.weight = off_tuning_param.sweight_max / (off_tuning_param.sweight_scale * (cd[0] - off_tuning_param.pdist));
                sv.flag = false;
              }
              svg.push_back(sv);
            }
          }
        }
      }
      return svg;
    }

    void arc_extender_voxel_extractor(vector<geometry_msgs::Point>& planar_tentacle_data, double yaw_sample, vector<double> ang_samples, string rot_type, double yaw_offset=0)
    {
      for(int i = 0; i < ang_samples.size(); i++)
      {
        vector <geometry_msgs::Point> newt;
        for(int j = 0; j < planar_tentacle_data.size(); j++)
        {
          newt.push_back( rotate3d( rotate3d(planar_tentacle_data[j], ang_samples[i], rot_type), yaw_offset, "yaw" ) );
        }
        this -> robot_param.tentacle_data.push_back(newt);
        this -> robot_param.support_vox_data.push_back(voxel_extractor(newt));
      }

      if (yaw_sample > 0)
      {
        robot_param.right_left_data.push_back("L");
      }
      else if (yaw_sample < 0)
      {
        robot_param.right_left_data.push_back("R");
      }
      else
      {
        robot_param.right_left_data.push_back("M");
      }
    }

    // NUA TODO: Do it for the roll as well.
    // DESCRIPTION: CONSTRUCT TENTACLES AND EXTRACT SUPPORT/PRIORITY VOXELS
    void construct_tentacle_pitchExt()
    {
      clearTentacleData();
      clearSupportVoxData();

      vector<geometry_msgs::Point> planar_tentacle_data;

      vector<double> yaw_samples;
      if(this -> off_tuning_param.tyaw_cnt > 1)
      {
        int half_tyaw_cnt = 0.5 * (this -> off_tuning_param.tyaw_cnt + 1);
        double half_yaw_angle = 0.5 * this -> off_tuning_param.tyaw;

        yaw_samples = sampling_func(0, half_yaw_angle, half_tyaw_cnt, this -> off_tuning_param.tyaw_samp_type);

        for(int i = 1; i < half_tyaw_cnt; i++)
        {
          yaw_samples.push_back(-yaw_samples[i]);
        }
      }
      else
      {
        yaw_samples.push_back(0);
      }

      vector<double> pitch_samples;
      if(this -> off_tuning_param.tpitch_cnt > 1)
      {
        int half_tpitch_cnt = 0.5 * (this -> off_tuning_param.tpitch_cnt + 1);
        double half_pitch_angle = 0.5 * this -> off_tuning_param.tpitch;

        pitch_samples = sampling_func(0, half_pitch_angle, half_tpitch_cnt, this -> off_tuning_param.tpitch_samp_type);
        for(int i = 1; i < half_tpitch_cnt; i++)
        {
          pitch_samples.push_back(-pitch_samples[i]);
        }
      }
      else
      {
        pitch_samples.push_back(0);
      }

      double radi;
      double delta_len = this -> off_tuning_param.tlen / this -> off_tuning_param.tsamp_cnt;
      double yaw_offset = -0.5 * PI;

      // TENTACLE 0 (Contains single sampling point at the center of the robot)
      vector <geometry_msgs::Point> newt;
      geometry_msgs::Point tp;
      tp.x = 0.0;
      tp.y = 0.0;
      tp.z = 0.0;
      newt.push_back(tp);
      this -> robot_param.tentacle_data.push_back(newt);
      this -> robot_param.support_vox_data.push_back(voxel_extractor(newt));
      robot_param.right_left_data.push_back("M");

      // ALL OTHER TENTACLES
      for(int y = 0; y < yaw_samples.size(); y++)
      {
        if(yaw_samples[y] == 0)
        {
          for(int p = 0; p < this -> off_tuning_param.tsamp_cnt; p++)
          {
            geometry_msgs::Point tp;
            tp.x = 0;
            tp.y = (p+1) * delta_len;
            tp.z = 0;
            planar_tentacle_data.push_back(tp);
          }
        }
        else
        {          
          geometry_msgs::Point arc_center;
          arc_center.x = 0;
          arc_center.y = 0;
          arc_center.z = 0;
          if(this -> off_tuning_param.tentacle_type == "circular")
          {
            radi = this -> off_tuning_param.tlen / (2 * abs(yaw_samples[y]));
            planar_tentacle_data = arc_by_radi_ang(arc_center, radi, 2 * yaw_samples[y], this -> off_tuning_param.tsamp_cnt);
          }
          else if(this -> off_tuning_param.tentacle_type == "linear")
          {
            planar_tentacle_data = line_by_len_ang(arc_center, this -> off_tuning_param.tlen, yaw_samples[y], this -> off_tuning_param.tsamp_cnt);
          }
        }

        // EXTEND TENTACLES ALONG PITCH ANGLE AND EXTRACT SUPPORT/PRIORITY CELLS
        arc_extender_voxel_extractor(planar_tentacle_data, yaw_samples[y], pitch_samples, "pitch", yaw_offset);
      }
    }

    void construct_tentacle_extend(bool no_restriction=false)
    {
      clearTentacleData();
      clearSupportVoxData();

      // SAMPLE YAW
      vector<double> yaw_samples;
      int half_tyaw_cnt = 0.5 * (off_tuning_param.tyaw_cnt + 1);
      double half_yaw_angle = 0.5 * off_tuning_param.tyaw;
      double init_yaw_angle = 0;

      if(off_tuning_param.tyaw_cnt > 1)
      {
        yaw_samples = sampling_func(init_yaw_angle, half_yaw_angle, half_tyaw_cnt, off_tuning_param.tyaw_samp_type);

        for(int i = 1; i < half_tyaw_cnt; i++)
        {
          yaw_samples.push_back(-yaw_samples[i]);
        }
      }
      else
      {
        yaw_samples.push_back(init_yaw_angle);
      }

      // SAMPLE PITCH
      vector<double> pitch_samples;
      int half_tpitch_cnt = 0.5 * (off_tuning_param.tpitch_cnt + 1);
      double half_pitch_angle = 0.5 * off_tuning_param.tpitch;
      double init_pitch_angle = 0;

      if(off_tuning_param.tpitch_cnt > 1)
      {
        pitch_samples = sampling_func(init_pitch_angle, half_pitch_angle, half_tpitch_cnt, off_tuning_param.tpitch_samp_type);
        for(int i = 1; i < half_tpitch_cnt; i++)
        {
          pitch_samples.push_back(-pitch_samples[i]);
        }
      }
      else
      {
        pitch_samples.push_back(init_pitch_angle);
      }

      // CONSTRUCT TENTACLE (SAMPLED TRAJECTORY) DATA
      int pcount = 0;
      int tcount = 0;
      double x;
      double y;
      double z;
      double ds = 1;
      double delta_dist = 1 * off_tuning_param.pdist;

      // TENTACLE 0 (Contains single sampling point at the center of the robot)
      vector<geometry_msgs::Point> planar_tentacle_data0;
      geometry_msgs::Point p0;
      p0.x = 0;
      p0.y = 0;
      p0.z = 0;
      planar_tentacle_data0.push_back(p0);
      robot_param.tentacle_data.push_back(planar_tentacle_data0);
      robot_param.support_vox_data.push_back(voxel_extractor(planar_tentacle_data0));
      robot_param.right_left_data.push_back("M");

      // ALL OTHER TENTACLES
      for (int j = 0; j < pitch_samples.size(); ++j)
      {
        for (int i = 0; i < yaw_samples.size(); ++i)
        {
          //ROS_INFO("tentacle %d", tcount);

          vector<geometry_msgs::Point> planar_tentacle_data;
          pcount = 0;
          x = 0;
          y = 0;
          z = 0;

          if (no_restriction)
          {
            while( (pcount+1) * delta_dist <= off_tuning_param.tlen )
            {
              if (off_tuning_param.tentacle_type == "circular")
              {
                ds = (pcount+1);
              }

              if (abs(yaw_samples[i]) > 0.5*PI)
              {
                x += -1 * delta_dist * cos(ds* ( PI - abs(yaw_samples[i]) ) ) * cos(pitch_samples[j]);
                y += delta_dist * sin(ds*yaw_samples[i]) * cos(pitch_samples[j]);
                z += delta_dist * sin(pitch_samples[j]);
              }
              else
              {
                x += delta_dist * cos(ds*yaw_samples[i]) * cos(pitch_samples[j]);
                y += delta_dist * sin(ds*yaw_samples[i]) * cos(pitch_samples[j]);
                z += delta_dist * sin(pitch_samples[j]);
              }

              geometry_msgs::Point p;
              p.x = x;
              p.y = y;
              p.z = z;
              planar_tentacle_data.push_back(p);

              pcount++;
            }
            tcount++;
          }
          else
          {
            while(isInsideTriangle(x, y, off_tuning_param.tlen, half_yaw_angle) && abs(ds*yaw_samples[i]) <= PI)
            {
              //ROS_INFO("p %d -> x: %f y: %f", pcount, x, y);
              
              if (off_tuning_param.tentacle_type == "circular")
              {
                ds = (pcount+1);
              }

              x += delta_dist * cos(ds*yaw_samples[i]) * cos(pitch_samples[j]);
              y += delta_dist * sin(ds*yaw_samples[i]) * cos(pitch_samples[j]);
              z += delta_dist * sin(pitch_samples[j]);

              geometry_msgs::Point p;
              p.x = x;
              p.y = y;
              p.z = z;
              planar_tentacle_data.push_back(p);

              pcount++;
            }
            tcount++;
          }

          // ADD TENTACLE DATA
          robot_param.tentacle_data.push_back(planar_tentacle_data);

          if (yaw_samples[i] > 0)
          {
            robot_param.right_left_data.push_back("L");
          }
          else if (yaw_samples[i] < 0)
          {
            robot_param.right_left_data.push_back("R");
          }
          else
          {
            robot_param.right_left_data.push_back("M");
          }

          // EXTRACT SUPPORT/PRIORITY CELLS
          robot_param.support_vox_data.push_back(voxel_extractor(planar_tentacle_data));
        }
      }
    }

    /*
    void construct_tentacle_trajectoryOpt()
    {
      // SAMPLE YAW
      vector<double> yaw_samples;
      int half_tyaw_cnt = 0.5 * (off_tuning_param.tyaw_cnt + 1);
      double half_yaw_angle = 0.5 * off_tuning_param.tyaw;

      if(off_tuning_param.tyaw_cnt > 1)
      {
        yaw_samples = sampling_func(0, half_yaw_angle, half_tyaw_cnt, off_tuning_param.tyaw_samp_type);

        for(int i = 1; i < half_tyaw_cnt; i++)
        {
          yaw_samples.push_back(-yaw_samples[i]);
        }
      }
      else
      {
        yaw_samples.push_back(0);
      }

      // SAMPLE PITCH
      vector<double> pitch_samples;
      int half_tpitch_cnt = 0.5 * (off_tuning_param.tpitch_cnt + 1);
      double half_pitch_angle = 0.5 * off_tuning_param.tpitch;

      if(off_tuning_param.tpitch_cnt > 1)
      {
        pitch_samples = sampling_func(0, half_pitch_angle, half_tpitch_cnt, off_tuning_param.tpitch_samp_type);
        for(int i = 1; i < half_tpitch_cnt; i++)
        {
          pitch_samples.push_back(-pitch_samples[i]);
        }
      }
      else
      {
        pitch_samples.push_back(0);
      }

      // SAMPLE TENTACLE LENGTH
      vector<double> tlen_samples;
      tlen_samples = sampling_func(0, off_tuning_param.tlen, half_tyaw_cnt);

      // INITIALIZE TRAJECTORY
      const Eigen::Vector4d limits(15.0, 10.0, 0, 0);

      // CONSTRUCT TENTACLE (SAMPLED TRAJECTORY) DATA
      int tcount = 0;
      int mid_counter = 0;
      //vector<geometry_msgs::Point> first, last;

      for (int i = 0; i < yaw_samples.size(); ++i)
      {
        for (int j = 0; j < pitch_samples.size(); ++j)
        {
          ROS_INFO_STREAM("tentacle " << tcount);

          geometry_msgs::Point p_first;
          p_first.x = off_tuning_param.pdist * cos(yaw_samples[i]) * cos(pitch_samples[j]);
          p_first.y = off_tuning_param.pdist * sin(yaw_samples[i]) * cos(pitch_samples[j]);
          p_first.z = off_tuning_param.pdist * sin(pitch_samples[j]);

          //tmp_marker.points.push_back(p_first);
          //first.push_back(p_first);

          geometry_msgs::Point p_last;
          p_last.x = off_tuning_param.tlen * cos(yaw_samples[i]) * cos(pitch_samples[j]);
          p_last.y = off_tuning_param.tlen * sin(yaw_samples[i]) * cos(pitch_samples[j]);
          p_last.z = off_tuning_param.tlen * sin(pitch_samples[j]);
          
          if (off_tuning_param.tentacle_type == "parametric")
          {
            ROS_INFO_STREAM("TENTACLE IS PARAMETRIC");
            /*
            geometry_msgs::Point p_middle;
            p_middle.x = off_tuning_param.tlen * cos(yaw_samples[0]) * cos(pitch_samples[j]);
            p_middle.y = off_tuning_param.tlen * sin(yaw_samples[0]) * cos(pitch_samples[j]);
            p_middle.z = off_tuning_param.tlen * sin(pitch_samples[j]);
            * /

            geometry_msgs::Point p_left_first;
            p_left_first.x = off_tuning_param.pdist * cos(yaw_samples[half_tyaw_cnt-1]) * cos(pitch_samples[j]);
            p_left_first.y = off_tuning_param.pdist * sin(yaw_samples[half_tyaw_cnt-1]) * cos(pitch_samples[j]);
            p_left_first.z = off_tuning_param.pdist * sin(pitch_samples[j]);

            geometry_msgs::Point p_left_last;
            p_left_last.x = off_tuning_param.tlen * cos(yaw_samples[half_tyaw_cnt-1]) * cos(pitch_samples[j]);
            p_left_last.y = off_tuning_param.tlen * sin(yaw_samples[half_tyaw_cnt-1]) * cos(pitch_samples[j]);
            p_left_last.z = off_tuning_param.tlen * sin(pitch_samples[j]);

            geometry_msgs::Point p_right_first;
            p_right_first.x = off_tuning_param.pdist * cos(yaw_samples.back()) * cos(pitch_samples[j]);
            p_right_first.y = off_tuning_param.pdist * sin(yaw_samples.back()) * cos(pitch_samples[j]);
            p_right_first.z = off_tuning_param.pdist * sin(pitch_samples[j]);

            geometry_msgs::Point p_right_last;
            p_right_last.x = off_tuning_param.tlen * cos(yaw_samples.back()) * cos(pitch_samples[j]);
            p_right_last.y = off_tuning_param.tlen * sin(yaw_samples.back()) * cos(pitch_samples[j]);
            p_right_last.z = off_tuning_param.tlen * sin(pitch_samples[j]);

            if (half_tyaw_cnt % 2 == 0)
            {
              int num_btw = 0.5 * (half_tyaw_cnt - 2);
              ROS_INFO_STREAM("EVEN -> num_btw: " << num_btw);
              vector<double> yaw_samples_middle = sampling_func(0, half_yaw_angle, num_btw+2, "linear", false);

              if (yaw_samples[i] > 0)
              {
                ROS_INFO("EVEN -> >0 yaw_samples[%d]: %f", i, yaw_samples[i]*180/PI);
                vector<geometry_msgs::Point> tlen_samples_middle = equadistant(p_left_last, p_left_first, num_btw);
                ROS_INFO_STREAM("mid_counter 1: " << mid_counter);

                if (mid_counter < yaw_samples_middle.size())
                { 
                  ROS_INFO_STREAM("EVEN -> 1");
                  p_last.x = off_tuning_param.tlen * cos(yaw_samples_middle[mid_counter]) * cos(pitch_samples[j]);
                  p_last.y = off_tuning_param.tlen * sin(yaw_samples_middle[mid_counter]) * cos(pitch_samples[j]);
                  p_last.z = off_tuning_param.tlen * sin(pitch_samples[j]);

                  mid_counter++;
                }
                else if (mid_counter >= yaw_samples_middle.size() && mid_counter < (yaw_samples_middle.size() + tlen_samples_middle.size()) )
                {
                  ROS_INFO_STREAM("EVEN -> 2");
                  p_last = tlen_samples_middle[mid_counter - yaw_samples_middle.size()];

                  mid_counter++;
                }
                else
                {
                  ROS_INFO_STREAM("EVEN -> 3");
                  p_last = p_left_first;
                  mid_counter = 0;
                }
              }
              else if (yaw_samples[i] < 0)
              {
                ROS_INFO("EVEN -> <0 yaw_samples[%d]: %f", i, yaw_samples[i]*180/PI);

                vector<geometry_msgs::Point> tlen_samples_middle = equadistant(p_right_last, p_right_first, num_btw);
                ROS_INFO_STREAM("mid_counter 2: " << mid_counter);

                if (mid_counter < yaw_samples_middle.size())
                {
                  ROS_INFO_STREAM("EVEN -> 4");
                  yaw_samples_middle[mid_counter] *= -1;
                  p_last.x = off_tuning_param.tlen * cos(yaw_samples_middle[mid_counter]) * cos(pitch_samples[j]);
                  p_last.y = off_tuning_param.tlen * sin(yaw_samples_middle[mid_counter]) * cos(pitch_samples[j]);
                  p_last.z = off_tuning_param.tlen * sin(pitch_samples[j]);

                  mid_counter++;
                }
                else if (mid_counter >= yaw_samples_middle.size() && mid_counter < (yaw_samples_middle.size() + tlen_samples_middle.size()) )
                {
                  ROS_INFO_STREAM("EVEN -> 5");
                  p_last = tlen_samples_middle[mid_counter - yaw_samples_middle.size()];

                  mid_counter++;
                }
                else
                {
                  ROS_INFO_STREAM("EVEN -> 6");
                  p_last = p_right_first;
                  mid_counter = 0;
                }
                
                ROS_INFO_STREAM("mid_counter 2 END ");
              }
            }
            else
            {
              int num_btw = 0.5 * (half_tyaw_cnt - 3);
              ROS_INFO_STREAM("ODD -> num_btw: " << num_btw);
              vector<double> yaw_samples_middle = sampling_func(0, half_yaw_angle, num_btw+2, "linear", false);

              if (yaw_samples[i] > 0)
              {
                vector<geometry_msgs::Point> tlen_samples_middle = equadistant(p_left_first, p_left_last, num_btw);

                if (mid_counter < yaw_samples_middle.size())
                { 
                  p_last.x = off_tuning_param.tlen * cos(yaw_samples_middle[mid_counter]) * cos(pitch_samples[j]);
                  p_last.y = off_tuning_param.tlen * sin(yaw_samples_middle[mid_counter]) * cos(pitch_samples[j]);
                  p_last.z = off_tuning_param.tlen * sin(pitch_samples[j]);

                  mid_counter++;
                }
                else if (mid_counter == yaw_samples_middle.size())
                {
                  p_last = p_left_last;
                }
                else if (mid_counter > yaw_samples_middle.size() && mid_counter < (yaw_samples_middle.size() + tlen_samples_middle.size()) )
                {
                  p_last = tlen_samples_middle[mid_counter - yaw_samples_middle.size()];

                  mid_counter++;
                }

                if (mid_counter >= (yaw_samples_middle.size() + tlen_samples_middle.size()))
                {
                  mid_counter = 0;
                }
              }
              else if (yaw_samples[i] < 0)
              {
                vector<geometry_msgs::Point> tlen_samples_middle = equadistant(p_right_last, p_right_first, num_btw);

                if (mid_counter < yaw_samples_middle.size())
                { 
                  yaw_samples_middle[mid_counter] *= -1;
                  p_last.x = off_tuning_param.tlen * cos(yaw_samples_middle[mid_counter]) * cos(pitch_samples[j]);
                  p_last.y = off_tuning_param.tlen * sin(yaw_samples_middle[mid_counter]) * cos(pitch_samples[j]);
                  p_last.z = off_tuning_param.tlen * sin(pitch_samples[j]);

                  mid_counter++;
                }
                else if (mid_counter == yaw_samples_middle.size())
                {
                  p_last = p_right_last;
                }
                else if (mid_counter >= yaw_samples_middle.size() && mid_counter < (yaw_samples_middle.size() + tlen_samples_middle.size()) )
                {
                  p_last = tlen_samples_middle[mid_counter - yaw_samples_middle.size()];

                  mid_counter++;
                }
                
                if (mid_counter >= (yaw_samples_middle.size() + tlen_samples_middle.size()))
                {
                  mid_counter = 0;
                }
              }
            }
          }
          //last.push_back(p_last);

          ROS_INFO_STREAM("tcount: " << tcount);

          ewok::Polynomial3DOptimization<10> po(limits);
          typename ewok::Polynomial3DOptimization<10>::Vector3Array path;
          path.push_back(Eigen::Vector3d(0, 0, 0));
          path.push_back(Eigen::Vector3d(p_first.x, p_first.y, p_first.z));
          path.push_back(Eigen::Vector3d(p_last.x, p_last.y, p_last.z));

          if(tcount = 1)
          {
            //tmp_marker.points.push_back(p_first);
            tmp_marker.points.push_back(p_last);
          }

          ewok::PolynomialTrajectory3D<10>::Ptr polynomial_trajectory = po.computeTrajectory(path);

          ROS_INFO("trajectory end point -> x: %f y: %f z: %f", p_last.x, p_last.y, p_last.z);
          
          vector<geometry_msgs::Point> single_tentacle = polynomial_trajectory -> getSegmentsPoints(0, process_param.nav_dt);
          vector<geometry_msgs::Point> single_tentacle_pruned;

          double total_tpath = 0;
          int k = 0;
          while(total_tpath < off_tuning_param.tlen && k < single_tentacle.size()-2)
          {
            total_tpath += find_Euclidean_distance(single_tentacle[k], single_tentacle[k+1]);
            single_tentacle_pruned.push_back(single_tentacle[k+1]);
            k++;
          }

          robot_param.tentacle_data.push_back(single_tentacle);
          robot_param.support_vox_data.push_back(voxel_extractor(single_tentacle));
          ROS_INFO("---------------------------------------------------------------------");

          tcount++;
          //single_tentacle_pruned.clear();
          //path.clear();
        }
      }
    }
    */

    void fillRobotVisu()
    {
      this -> visu_param.robot_visu.ns = this -> robot_param.robot_name;
      this -> visu_param.robot_visu.action = visualization_msgs::Marker::ADD;
      this -> visu_param.robot_visu.pose.position.x = 0;
      this -> visu_param.robot_visu.pose.position.y = 0;
      this -> visu_param.robot_visu.pose.position.z = 0;
      this -> visu_param.robot_visu.pose.orientation.x = 0;
      this -> visu_param.robot_visu.pose.orientation.y = 0;
      this -> visu_param.robot_visu.pose.orientation.z = 0;
      this -> visu_param.robot_visu.pose.orientation.w = 1;
      this -> visu_param.robot_visu.id = 888;
      this -> visu_param.robot_visu.type = visualization_msgs::Marker::SPHERE;
      this -> visu_param.robot_visu.scale.x = this -> robot_param.width;
      this -> visu_param.robot_visu.scale.y = this -> robot_param.length;
      this -> visu_param.robot_visu.scale.z = this -> robot_param.height;
      this -> visu_param.robot_visu.color.r = 0.128;
      this -> visu_param.robot_visu.color.g = 0.0;
      this -> visu_param.robot_visu.color.b = 0.128;
      this -> visu_param.robot_visu.color.a = 1.0;
      this -> visu_param.robot_visu.header.frame_id = this -> robot_param.robot_frame_name;
    }

    void publishRobot()
    {
      if(this -> process_param.visu_flag == true)
      {
        this -> visu_param.robot_visu.header.seq++;
        this -> visu_param.robot_visu.header.stamp = ros::Time(0);
        this -> visu_param.robot_visu_pub.publish(visu_param.robot_visu);
      }
    }

    void fillTentacleTsampSupportvoxVisu()
    {
      int tentacle_cnt = this -> robot_param.tentacle_data.size();
      int tsamp_cnt;

      // VISUALIZE TENTACLES, SUPPORT/PRIORITY VOXELS
      for(int k = 0; k < tentacle_cnt; k++)
      {
        // SET TENTACLE VISUALIZATION SETTINGS
        visualization_msgs::Marker tentacle_line_strip;
        tentacle_line_strip.ns = "tentacle" + to_string(k);
        tentacle_line_strip.id = k;
        tentacle_line_strip.header.frame_id = this -> robot_param.robot_frame_name;
        tentacle_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        tentacle_line_strip.action = visualization_msgs::Marker::ADD;
        tentacle_line_strip.pose.orientation.w = 1.0;
        tentacle_line_strip.scale.x = 0.02;
        tentacle_line_strip.color.r = 1.0;
        tentacle_line_strip.color.g = 1.0;
        tentacle_line_strip.color.b = 1.0;
        tentacle_line_strip.color.a = 1.0;

        // SET SAMPLING POINTS VISUALIZATION SETTINGS
        visualization_msgs::Marker tentacle_tsamp;
        tentacle_tsamp.ns = "sampling_points" + to_string(k);;
        tentacle_tsamp.id = k;
        tentacle_tsamp.header.frame_id = this -> robot_param.robot_frame_name;
        tentacle_tsamp.type = visualization_msgs::Marker::SPHERE_LIST;
        tentacle_tsamp.action = visualization_msgs::Marker::ADD;
        tentacle_tsamp.pose.orientation.w = 1.0;
        tentacle_tsamp.scale.x = 0.05;
        tentacle_tsamp.scale.y = 0.05;
        tentacle_tsamp.scale.z = 0.05;
        tentacle_tsamp.color.r = 0.0;
        tentacle_tsamp.color.g = 0.0;
        tentacle_tsamp.color.b = 0.0;
        tentacle_tsamp.color.a = 1;

        tsamp_cnt = this -> robot_param.tentacle_data[k].size();
        for(int p = 0; p < tsamp_cnt; p++)
        {
          if (tsamp_cnt > 1)
          {
            tentacle_line_strip.points.push_back(this -> robot_param.tentacle_data[k][p]);
          }
          else
          {
            tentacle_line_strip.points.push_back(this -> robot_param.tentacle_data[k][p]);
            tentacle_line_strip.points.push_back(this -> robot_param.tentacle_data[k][p]);
          }

          tentacle_tsamp.points.push_back(this -> robot_param.tentacle_data[k][p]);
        }
        this -> visu_param.tentacle_visu.markers.push_back(tentacle_line_strip);
        this -> visu_param.tsamp_visu.markers.push_back(tentacle_tsamp);

        // SET SUPPORT/PRIORITY VOXEL VISUALIZATION SETTINGS
        visualization_msgs::Marker support_vox_points;
        support_vox_points.ns = "support_voxel" + to_string(k);
        support_vox_points.id = k;
        support_vox_points.header.frame_id = this -> robot_param.robot_frame_name;
        support_vox_points.type = visualization_msgs::Marker::CUBE_LIST;
        support_vox_points.action = visualization_msgs::Marker::ADD;
        support_vox_points.pose.orientation.w = 1.0;
        support_vox_points.scale.x = this -> off_tuning_param.egrid_vdim;
        support_vox_points.scale.y = this -> off_tuning_param.egrid_vdim;
        support_vox_points.scale.z = this -> off_tuning_param.egrid_vdim;

        for(int s = 0; s < this -> robot_param.support_vox_data[k].size(); s++)
        {
          geometry_msgs::Point po;
          toPoint(robot_param.support_vox_data[k][s].index, po);

          std_msgs::ColorRGBA sv_point_color;
          support_vox_points.points.push_back(po);

          sv_point_color.g = 0;
          sv_point_color.a = 0.05;
          if(this -> robot_param.support_vox_data[k][s].flag)
          { 
            sv_point_color.r = 1.0;   
            sv_point_color.b = 0;
          }
          else
          {
            sv_point_color.r = 0.5;
            sv_point_color.b = 0.5;
          }
          support_vox_points.colors.push_back(sv_point_color);
        }
        this -> visu_param.support_vox_visu.markers.push_back(support_vox_points);
      }
    }

    void publishTentacleTsampSupportvox()
    {
      if(this -> process_param.visu_flag == true)
      {
        int tentacle_cnt = this -> robot_param.tentacle_data.size();

        for(int k = 0; k < tentacle_cnt; k++)
        {
          // UPDATE SEQUENCE AND STAMP FOR TENTACLES
          this -> visu_param.tentacle_visu.markers[k].header.seq++;
          this -> visu_param.tentacle_visu.markers[k].header.stamp = ros::Time::now();

          // UPDATE SEQUENCE AND STAMP FOR SUPPORT CELLS
          this -> visu_param.support_vox_visu.markers[k].header.seq++;
          this -> visu_param.support_vox_visu.markers[k].header.stamp = ros::Time::now();

          // UPDATE SEQUENCE AND STAMP FOR SAMPLING POINTS ON TENTACLES 
          this -> visu_param.tsamp_visu.markers[k].header.seq++;
          this -> visu_param.tsamp_visu.markers[k].header.stamp = ros::Time::now();
        }

        this -> visu_param.tentacle_visu_pub.publish(this -> visu_param.tentacle_visu);
        this -> visu_param.opt_tentacle_visu_pub.publish(this -> visu_param.opt_tentacle_visu);
        this -> visu_param.tsamp_visu_pub.publish(this -> visu_param.tsamp_visu);
        this -> visu_param.support_vox_visu_pub.publish(this -> visu_param.support_vox_visu);
      }
    }

    void publishOccupancy()
    {
      if(this -> process_param.visu_flag == true)
      {
        this -> visu_param.occupancy_pc.header.seq++;
        this -> visu_param.occupancy_pc.header.stamp = ros::Time::now();
        this -> visu_param.occupancy_pc_pub.publish(this -> visu_param.occupancy_pc);
      }
    }

    void fillPathVisu()
    {
      if(this -> process_param.visu_flag == true)
      {
        this -> visu_param.path_visu.ns = "path2glory";
        this -> visu_param.path_visu.id = 753;
        this -> visu_param.path_visu.type = visualization_msgs::Marker::SPHERE_LIST;
        this -> visu_param.path_visu.action = visualization_msgs::Marker::ADD;
        this -> visu_param.path_visu.pose.orientation.w = 1.0;
        this -> visu_param.path_visu.scale.x = 0.2;
        this -> visu_param.path_visu.scale.y = 0.2;
        this -> visu_param.path_visu.scale.z = 0.2;
        this -> visu_param.path_visu.color.r = 1.0;
        this -> visu_param.path_visu.color.g = 0.1;
        this -> visu_param.path_visu.color.b = 1.0;
        this -> visu_param.path_visu.color.a = 1.0;
      }
    }

    void fillCommandVisu()
    {
      if(this -> process_param.visu_flag == true)
      {
        this -> visu_param.command_visu.ns = "commandante";
        this -> visu_param.command_visu.id = 1928;
        this -> visu_param.command_visu.type = visualization_msgs::Marker::SPHERE_LIST;
        this -> visu_param.command_visu.action = visualization_msgs::Marker::ADD;
        this -> visu_param.command_visu.pose.orientation.w = 1.0;
        this -> visu_param.command_visu.scale.x = 0.2;
        this -> visu_param.command_visu.scale.y = 0.2;
        this -> visu_param.command_visu.scale.z = 0.2;
        this -> visu_param.command_visu.color.r = 0.05;
        this -> visu_param.command_visu.color.g = 1.0;
        this -> visu_param.command_visu.color.b = 1.0;
        this -> visu_param.command_visu.color.a = 1.0;
      }
    }

    void publishPath()
    {
      if(this -> process_param.visu_flag == true)
      {
        this -> visu_param.path_visu.header.frame_id = this -> map_util.getFrameName();
        this -> visu_param.path_visu.header.seq++;
        //this -> visu_param.path_visu.header.stamp = ros::Time(0);
        this -> visu_param.path_visu.header.stamp = ros::Time::now();

        this -> visu_param.path_visu_pub.publish(this -> visu_param.path_visu);
      }
    }

    void publishCommand()
    {
      if(this -> process_param.visu_flag == true)
      {
        this -> visu_param.command_visu.header.frame_id = this -> map_util.getFrameName();
        this -> visu_param.command_visu.header.seq++;
        //this -> visu_param.command_visu.header.stamp = ros::Time(0);
        this -> visu_param.command_visu.header.stamp = ros::Time::now();

        this -> visu_param.command_visu_pub.publish(this -> visu_param.command_visu);
      }
    }

    geometry_msgs::Point interpol(geometry_msgs::Point p, double dist)
    {
      double p_norm = find_norm(p);
      double m;

      //ROS_INFO_STREAM("pnorm: " << p_norm);

      if (p_norm == 0)
      {
        m = 0.0;
      }
      else if (p_norm > dist)
      {
        m = dist / p_norm;
      }
      else
      {
        m = 1.0;
      }
      
      geometry_msgs::Point pm;
      pm.x = m * p.x;
      pm.y = m * p.y;
      pm.z = m * p.z;

      return pm;
    }

    geometry_msgs::Point interpol(geometry_msgs::Point p1, geometry_msgs::Point p2, double dist_from_p1)
    {
      geometry_msgs::Point v;
      v.x = p2.x - p1.x;
      v.y = p2.y - p1.y;
      v.z = p2.z - p1.z;

      double v_norm = find_norm(v);
      double m;

      if (v_norm == 0)
      {
        m = 0.0;
      }
      else if (v_norm > dist_from_p1)
      {
        m = dist_from_p1 / v_norm;
      }
      else
      {
        m = 1.0;
      }

      geometry_msgs::Point pm;
      pm.x = p1.x + m * v.x;
      pm.y = p1.y + m * v.y;
      pm.z = p1.z + m * v.z;

      return pm;
    }

    vector<geometry_msgs::Point> equadistant(geometry_msgs::Point p1, geometry_msgs::Point p2, int num_btw)
    {
      vector<geometry_msgs::Point> p_vector;
      double t;
      double delta_t = find_Euclidean_distance(p1, p2) / (10 * (num_btw + 1));

      for (int i = 0; i < num_btw; ++i)
      {
        t = delta_t * (i+1);

        geometry_msgs::Point p;
        p.x = p1.x + (p2.x - p1.x) * t;
        p.y = p1.y + (p2.y - p1.y) * t;
        p.z = p1.z + (p2.z - p1.z) * t;
        p_vector.push_back(p);
      }
      return p_vector;
    }

  public:
    Tentabot(NodeHandle& nh, tf::TransformListener* listener, ProcessParams& pp, RobotParams& rp, OffTuningParams& offtp, OnTuningParams& ontp, MapUtility& mu, GoalUtility& gu)                                                    // constructor
    {
      cout << "Welcome to Tentabot 3D Navigation Simulation! I hope you'll enjoy the ride..." << endl;

      this -> tflistener = new tf::TransformListener;
      this -> tflistener = listener;

      this -> setProcessParams(pp);
      this -> process_param.counter = 0;
      this -> setRobotParams(rp);
      this -> setOffTuningParams(offtp);
      this -> setOnTuningParams(ontp);
      ros::Time t1 = ros::Time::now();
      this -> setEgoGridData();
      ros::Time t2 = ros::Time::now();
      this -> map_util = mu;
      this -> goal_util = gu;

      ros::Time t3 = ros::Time::now();
      construct_tentacle_extend(false);
      //construct_tentacle_pitchExt();
      //construct_tentacle_trajectoryOpt();
      ros::Time t4 = ros::Time::now();
      //this -> optimizeTentacles();
      //this -> initializeBSplineOptimization();

      this -> status_param.robot_pose.position.x = rp.init_robot_pose.position.x;
      this -> status_param.robot_pose.position.y = rp.init_robot_pose.position.y;
      this -> status_param.robot_pose.position.z = rp.init_robot_pose.position.z;
      this -> status_param.robot_pose.orientation.x = rp.init_robot_pose.orientation.x;
      this -> status_param.robot_pose.orientation.y = rp.init_robot_pose.orientation.y;
      this -> status_param.robot_pose.orientation.z = rp.init_robot_pose.orientation.z;
      this -> status_param.robot_pose.orientation.w = rp.init_robot_pose.orientation.w;
      this -> status_param.prev_robot_pose =this -> status_param.robot_pose;
      this -> status_param.tcrash_bin.resize(this -> off_tuning_param.tyaw_cnt * this -> off_tuning_param.tpitch_cnt);
      this -> status_param.navigability_flag = false;
      this -> status_param.best_tentacle = -1;
      this -> status_param.ex_best_tentacle = -1;
      this -> status_param.ex_best_sample = -1;
      this -> status_param.nav_result = 8;
      this -> status_param.nav_length = 0;
      this -> status_param.command_pub = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 10);
      this -> status_param.command_point_pub = nh.advertise<geometry_msgs::Point>("command/point", 10);
      this -> status_param.nav_duration = 0;
      this -> status_param.speed_counter = 0;
      this -> status_param.dummy_current_speed = 1.0;

      if(this -> process_param.visu_flag == true)
      {
        this -> fillRobotVisu();
        this -> fillTentacleTsampSupportvoxVisu();
        this -> visu_param.occupancy_pc.header.frame_id = this -> robot_param.robot_frame_name;
        this -> fillPathVisu();
        this -> fillCommandVisu();
        this -> fillTmpMarker();
        tmp_pub = nh.advertise<visualization_msgs::Marker>("tmp", 100);

        this -> visu_param.robot_visu_pub = nh.advertise<visualization_msgs::Marker>(this -> robot_param.robot_name, 100);
        this -> visu_param.tentacle_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("tentacles", 100);
        this -> visu_param.opt_tentacle_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("optimized_tentacles", 100);
        this -> visu_param.tsamp_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("sampling_points", 100);
        this -> visu_param.support_vox_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("support_voxels", 100);
        this -> visu_param.occupancy_pc_pub = nh.advertise<sensor_msgs::PointCloud>("ego_occupancy_pc", 100);
        this -> visu_param.path_visu_pub = nh.advertise<visualization_msgs::Marker>("path2glory", 100);
        this -> visu_param.command_visu_pub = nh.advertise<visualization_msgs::Marker>("commandante", 100);
        this -> visu_param.next_pub = nh.advertise<visualization_msgs::Marker>("nextPub", 100);

        traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("global_trajectory", 1, true);
        current_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("optimal_trajectory", 1, true);
      }

      string rand_filename = createFileName();
      
      // WRITE PARAMETERS
      string bench_param_filename = "/home/akmandor/catkin_ws/src/tentabot/benchmark/tnav/" + rand_filename + "_param_" + this -> map_util.getMapName() + ".csv";
      this -> nav_param_bench.open(bench_param_filename);
      this -> nav_param_bench << "nav_dt," + to_string(this -> process_param.nav_dt) + "\n";
      this -> nav_param_bench << "dummy_max_lat_velo," + to_string(this -> robot_param.dummy_max_lat_velo) + "\n";
      this -> nav_param_bench << "dummy_max_lat_acc," + to_string(this -> robot_param.dummy_max_lat_acc) + "\n";
      this -> nav_param_bench << "dummy_max_yaw_velo," + to_string(this -> robot_param.dummy_max_yaw_velo) + "\n";
      this -> nav_param_bench << "dummy_max_yaw_acc," + to_string(this -> robot_param.dummy_max_yaw_acc) + "\n";
      this -> nav_param_bench << "tyaw_cnt," + to_string(this -> off_tuning_param.tyaw_cnt) + "\n";
      this -> nav_param_bench << "tpitch_cnt," + to_string(this -> off_tuning_param.tpitch_cnt) + "\n";
      this -> nav_param_bench << "tsamp_cnt," + to_string(this -> off_tuning_param.tsamp_cnt) + "\n";
      this -> nav_param_bench << "tlen," + to_string(this -> off_tuning_param.tlen) + "\n";
      this -> nav_param_bench << "tyaw," + to_string(this -> off_tuning_param.tyaw) + "\n";
      this -> nav_param_bench << "tpitch," + to_string(this -> off_tuning_param.tpitch) + "\n";
      this -> nav_param_bench << "pdist," + to_string(this -> off_tuning_param.pdist) + "\n";
      this -> nav_param_bench << "sdist," + to_string(this -> off_tuning_param.sdist) + "\n";
      this -> nav_param_bench << "egrid_vdim," + to_string(this -> off_tuning_param.egrid_vdim) + "\n";
      this -> nav_param_bench << "egrid_vnumx," + to_string(this -> off_tuning_param.egrid_vnumx) + "\n";
      this -> nav_param_bench << "egrid_vnumy," + to_string(this -> off_tuning_param.egrid_vnumy) + "\n";
      this -> nav_param_bench << "egrid_vnumz," + to_string(this -> off_tuning_param.egrid_vnumz) + "\n";
      this -> nav_param_bench << "crash_dist," + to_string(this -> on_tuning_param.crash_dist) + "\n";
      this -> nav_param_bench << "clear_scale," + to_string(this -> on_tuning_param.clear_scale) + "\n";
      this -> nav_param_bench << "clutter_scale," + to_string(this -> on_tuning_param.clutter_scale) + "\n";
      this -> nav_param_bench << "close_scale," + to_string(this -> on_tuning_param.close_scale) + "\n";
      this -> nav_param_bench << "smooth_scale," + to_string(this -> on_tuning_param.smooth_scale) + "\n";
      this -> nav_param_bench.close();

      // WRITE PRE BENCHMARKS
      string pre_bench_filename = "/home/akmandor/catkin_ws/src/tentabot/benchmark/tnav/" + rand_filename + "_pre_" + this -> map_util.getMapName() + ".csv";
      this -> nav_pre_bench.open(pre_bench_filename);
      this -> nav_pre_bench << "grid[s],tentacle_voxel[s]\n";
      this -> nav_pre_bench << to_string( (t2-t1).toSec() ) + ",";
      this -> nav_pre_bench << to_string( (t4-t3).toSec() ) + "\n";
      this -> nav_pre_bench.close();

      // WRITE PROCESS BENCHMARKS
      string process_bench_filename = "/home/akmandor/catkin_ws/src/tentabot/benchmark/tnav/" + rand_filename + "_process_" + this -> map_util.getMapName() + ".csv";
      this -> nav_process_bench.open(process_bench_filename);
      this -> nav_process_bench << "upGVox[ns],upHeur[ns],selectT[ns],moveT[ns]\n";
    }
    
    ~Tentabot()                                // destructor
    {
      //ROS_INFO( "Calling Destructor for Tentabot..." );
      delete this -> tflistener;
      this -> nav_process_bench.close();
      this -> nav_result_bench.close();
      this -> rl_bench.close();
    } 

    // GETTER FUNCTIONS
    ProcessParams getProcessParams()
    {
      return this -> process_param;
    }

    RobotParams getRobotParams()
    {
      return this -> robot_param;
    }

    OffTuningParams getOffTuningParams()
    {
      return this -> off_tuning_param;
    }

    OnTuningParams getOnTuningParams()
    {
      return this -> on_tuning_param;
    }

    HeuristicParams getHeuristicParams()
    {
      return this -> heuristic_param;
    }

    StatusParams getStatusParams()
    {
      return this -> status_param;
    }

    VisuParams getVisuParams()
    {
      return this -> visu_param;
    }

    ofstream& getNavParamBench()
    {
      return this -> nav_param_bench;
    }

    ofstream& getNavPreBench()
    {
      return this -> nav_pre_bench;
    }

    ofstream& getNavProcessBench()
    {
      return this -> nav_process_bench;
    }

    ofstream& getNavResultBench()
    {
      return this -> nav_result_bench;
    }

    ofstream& getRLBench()
    {
      return this -> rl_bench;
    }

    // SETTER FUNCTIONS
    void setProcessParams(ProcessParams new_process_param) // 5 params
    {
      this -> process_param.visu_flag = new_process_param.visu_flag;
      this -> process_param.online_tuning_flag = new_process_param.online_tuning_flag;
      this -> process_param.time_limit = new_process_param.time_limit;
      this -> process_param.nav_dt = new_process_param.nav_dt;
      this -> process_param.navexit_flag = new_process_param.navexit_flag;
      this -> process_param.goal_close_threshold = new_process_param.goal_close_threshold;
      this -> process_param.counter = new_process_param.counter;
    }

    void setRobotParams(RobotParams new_robot_param)      // 19 params
    {
      this -> robot_param.width = new_robot_param.width;
      this -> robot_param.length = new_robot_param.length;
      this -> robot_param.height = new_robot_param.height;
      this -> robot_param.dummy_max_lat_velo = new_robot_param.dummy_max_lat_velo; 
      this -> robot_param.dummy_max_yaw_velo = new_robot_param.dummy_max_yaw_velo; 
      this -> robot_param.dummy_max_pitch_velo = new_robot_param.dummy_max_pitch_velo;
      this -> robot_param.dummy_max_roll_velo = new_robot_param.dummy_max_roll_velo;
      this -> robot_param.init_robot_pose = new_robot_param.init_robot_pose;
      this -> robot_param.robot_frame_name = new_robot_param.robot_frame_name;
      this -> robot_param.robot_name = new_robot_param.robot_name;
      this -> robot_param.tentacle_data = new_robot_param.tentacle_data;
      this -> robot_param.right_left_data = new_robot_param.right_left_data;
      this -> robot_param.support_vox_data = new_robot_param.support_vox_data;
      this -> robot_param.nav_sensor.freq = new_robot_param.nav_sensor.freq;
      this -> robot_param.nav_sensor.resolution = new_robot_param.nav_sensor.resolution;
      this -> robot_param.nav_sensor.range_x = new_robot_param.nav_sensor.range_x;
      this -> robot_param.nav_sensor.range_y = new_robot_param.nav_sensor.range_y;
      this -> robot_param.nav_sensor.range_z = new_robot_param.nav_sensor.range_z;
      this -> robot_param.nav_sensor.pose_wrt_robot = new_robot_param.nav_sensor.pose_wrt_robot;
      this -> robot_param.nav_sensor.frame_name = new_robot_param.nav_sensor.frame_name;
      
    }

    void setOffTuningParams(OffTuningParams new_off_tuning_param) // 24 params
    {
      this -> off_tuning_param.tyaw_cnt = new_off_tuning_param.tyaw_cnt;
      this -> off_tuning_param.tpitch_cnt = new_off_tuning_param.tpitch_cnt;
      this -> off_tuning_param.troll_cnt = new_off_tuning_param.troll_cnt;
      this -> off_tuning_param.tsamp_cnt = new_off_tuning_param.tsamp_cnt;
      this -> off_tuning_param.tlat_velo_cnt = new_off_tuning_param.tlat_velo_cnt;
      this -> off_tuning_param.tyaw_velo_cnt = new_off_tuning_param.tyaw_velo_cnt;
      this -> off_tuning_param.tpitch_velo_cnt = new_off_tuning_param.tpitch_velo_cnt;
      this -> off_tuning_param.troll_velo_cnt = new_off_tuning_param.troll_velo_cnt;
      this -> off_tuning_param.tlen = new_off_tuning_param.tlen;
      this -> off_tuning_param.tyaw = new_off_tuning_param.tyaw;
      this -> off_tuning_param.tpitch = new_off_tuning_param.tpitch;
      this -> off_tuning_param.troll = new_off_tuning_param.troll;
      this -> off_tuning_param.tentacle_type = new_off_tuning_param.tentacle_type;
      this -> off_tuning_param.tyaw_samp_type = new_off_tuning_param.tyaw_samp_type;
      this -> off_tuning_param.tpitch_samp_type = new_off_tuning_param.tpitch_samp_type;
      this -> off_tuning_param.troll_samp_type = new_off_tuning_param.troll_samp_type;
      this -> off_tuning_param.pdist = new_off_tuning_param.pdist;
      this -> off_tuning_param.sdist = new_off_tuning_param.sdist;
      this -> off_tuning_param.sweight_max = new_off_tuning_param.sweight_max;
      this -> off_tuning_param.sweight_scale = new_off_tuning_param.sweight_scale;
      this -> off_tuning_param.egrid_vdim = new_off_tuning_param.egrid_vdim;
      this -> off_tuning_param.egrid_vnumx = new_off_tuning_param.egrid_vnumx;
      this -> off_tuning_param.egrid_vnumy = new_off_tuning_param.egrid_vnumy;
      this -> off_tuning_param.egrid_vnumz = new_off_tuning_param.egrid_vnumz;
    }

    void setOnTuningParams(OnTuningParams new_on_tuning_param)    // 7 params
    {
      this -> on_tuning_param.tbin_window = new_on_tuning_param.tbin_window;
      this -> on_tuning_param.tbin_obs_cnt_threshold = new_on_tuning_param.tbin_obs_cnt_threshold;
      this -> on_tuning_param.clear_scale = new_on_tuning_param.clear_scale;
      this -> on_tuning_param.clutter_scale = new_on_tuning_param.clutter_scale;
      this -> on_tuning_param.close_scale = new_on_tuning_param.close_scale;
      this -> on_tuning_param.smooth_scale = new_on_tuning_param.smooth_scale;
      this -> on_tuning_param.crash_dist = new_on_tuning_param.crash_dist;
    }

    void setHeuristicParams(HeuristicParams new_heuristic_param)
    {
      this -> heuristic_param.navigability_set = new_heuristic_param.navigability_set;
      this -> heuristic_param.clearance_set = new_heuristic_param.clearance_set;
      this -> heuristic_param.clutterness_set = new_heuristic_param.clutterness_set;
      this -> heuristic_param.closeness_set = new_heuristic_param.closeness_set;
      this -> heuristic_param.smoothness_set = new_heuristic_param.smoothness_set;
    }

    void setStatusParams(StatusParams new_status_param)
    {
      this -> status_param.prev_robot_pose = new_status_param.prev_robot_pose;
      this -> status_param.robot_pose = new_status_param.robot_pose;
      this -> status_param.robot_pose_command = new_status_param.robot_pose_command;
      this -> status_param.ego_grid_data.ovox_pos = new_status_param.ego_grid_data.ovox_pos;
      this -> status_param.ego_grid_data.ovox_value = new_status_param.ego_grid_data.ovox_value;
      this -> status_param.best_tentacle = new_status_param.best_tentacle;
      this -> status_param.tcrash_bin = new_status_param.tcrash_bin;
      this -> status_param.navigability_flag = new_status_param.navigability_flag;
      this -> status_param.ex_best_tentacle = new_status_param.ex_best_tentacle;
      this -> status_param.ex_best_sample = new_status_param.ex_best_sample;
      this -> status_param.nav_result = new_status_param.nav_result;
      this -> status_param.nav_length = new_status_param.nav_length;
      this -> status_param.prev_action_time = new_status_param.prev_action_time;
      this -> status_param.command_pub = new_status_param.command_pub;
      this -> status_param.command_point_pub = new_status_param.command_point_pub;
      this -> status_param.nav_duration = new_status_param.nav_duration;
      this -> status_param.speed_counter = new_status_param.speed_counter;
      this -> status_param.dummy_current_speed = new_status_param.dummy_current_speed;
    }

    void setVisuParams(VisuParams new_visu_param)
    {
      this -> visu_param.robot_visu_pub = new_visu_param.robot_visu_pub;
      this -> visu_param.tentacle_visu_pub = new_visu_param.tentacle_visu_pub;
      this -> visu_param.opt_tentacle_visu_pub = new_visu_param.opt_tentacle_visu_pub;
      this -> visu_param.tsamp_visu_pub = new_visu_param.tsamp_visu_pub;
      this -> visu_param.support_vox_visu_pub = new_visu_param.support_vox_visu_pub;
      this -> visu_param.path_visu_pub = new_visu_param.path_visu_pub;
      this -> visu_param.command_visu_pub = new_visu_param.command_visu_pub;
      
      this -> visu_param.robot_visu = new_visu_param.robot_visu;
      this -> visu_param.tentacle_visu = new_visu_param.tentacle_visu;
      this -> visu_param.opt_tentacle_visu = new_visu_param.opt_tentacle_visu;
      this -> visu_param.tsamp_visu = new_visu_param.tsamp_visu;
      this -> visu_param.support_vox_visu = new_visu_param.support_vox_visu;
      this -> visu_param.occupancy_pc.header = new_visu_param.occupancy_pc.header;
      this -> visu_param.occupancy_pc.points = new_visu_param.occupancy_pc.points;
      this -> visu_param.occupancy_pc.channels = new_visu_param.occupancy_pc.channels;
      this -> visu_param.path_visu = new_visu_param.path_visu;
      this -> visu_param.command_visu = new_visu_param.command_visu;
    }

    void setEgoGridData()
    {
      int total_voxel_cnt = off_tuning_param.egrid_vnumx * off_tuning_param.egrid_vnumy * off_tuning_param.egrid_vnumz;
      
      status_param.ego_grid_data.ovox_pos.resize(total_voxel_cnt);
      
      for(int v = 0; v < total_voxel_cnt; v++)
      {
        geometry_msgs::Point po;
        toPoint(v, po);
        status_param.ego_grid_data.ovox_pos[v] = po;
      }

      status_param.ego_grid_data.ovox_value.resize(total_voxel_cnt);
    }

    void setOnTuning(bool flag)
    {
      this -> process_param.online_tuning_flag = flag;
    }

    void print_vecvecd(vector< vector<double> > vecvec)
    {
      int count = 0;
      int vsize1 = vecvec.size();
      for(int i = 0; i < vsize1; i++)
      {
        int vsize2 = vecvec[i].size();
        for(int j = 0; j < vsize2; j++)
        {
          cout << count << ") " << 180*vecvec[i][j]/PI << endl;
          count++;
        }
      }
    }

    void print_vecvecPoint(vector< vector<geometry_msgs::Point> > vecvec)
    {
      int count = 0;
      int vsize1 = vecvec.size();
      for(int i = 0; i < vsize1; i++)
      {
        int vsize2 = vecvec[i].size();
        for(int j = 0; j < vsize2; j++)
        {
          cout << count << ": (" << vecvec[i][j].x << ", " << vecvec[i][j].y << ", " << vecvec[i][j].z << ")" << endl;
          count++;
        }
      }
    }

    void clearTentacleData()
    {
      int tentacle_cnt = this -> robot_param.tentacle_data.size();

      for(int i = 0; i < tentacle_cnt; i++)
      {
        this -> robot_param.tentacle_data[i].clear();
      }

      this -> robot_param.tentacle_data.clear();

      robot_param.right_left_data.clear();
    }

    void clearSupportVoxData()
    {
      int svdnum = this -> robot_param.support_vox_data.size();

      for(int i = 0; i < svdnum; i++)
      {
        this -> robot_param.support_vox_data[i].clear();
      }

      this -> robot_param.support_vox_data.clear();
    }

    void publishTentabot()
    {
      if(process_param.visu_flag == true)
      {
        goal_util.publishGoal();

        map_util.publishOctmapMsg();

        map_util.publishRecentPCMsg();

        //map_util.publishLocalPCMsg();

        publishRobot();

        publishTentacleTsampSupportvox();

        publishOccupancy();

        publishPath();

        publishCommand();
      }
    }

    // DESCRIPTION: UPDATE OCCUPANCY GRID CELL FOR THE ROBOT
    void updateOccupancyVox()
    {
      map_util.addOctmapFromRecentPCMsg();

      // UPDATE THE LINEAR OCCUPANCY GRID AROUND THE ROBOT
      int total_voxel_cnt = status_param.ego_grid_data.ovox_pos.size();
      status_param.ego_grid_data.ovox_value.clear(); 
      status_param.ego_grid_data.ovox_value.resize(total_voxel_cnt);
      
      if (process_param.visu_flag == true)
      {
        visu_param.occupancy_pc.points.clear();
      }

      int vox_index;

      double radx = 0.25 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumx;
      double rady = 0.25 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumy;
      double radz = 0.25 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumz;

      octomap::point3d octmap_min(status_param.robot_pose.position.x - radx, status_param.robot_pose.position.y - rady, status_param.robot_pose.position.z - radz);
      octomap::point3d octmap_max(status_param.robot_pose.position.x + radx, status_param.robot_pose.position.y + rady, status_param.robot_pose.position.z + radz);

      map_util.getOctmap() -> setBBXMin(octmap_min);
      map_util.getOctmap() -> setBBXMax(octmap_max);
      for(octomap::ColorOcTree::iterator it = map_util.getOctmap() -> begin(); it != map_util.getOctmap() -> end(); ++it)
      {
        if (map_util.getOctmap() -> inBBX(it.getKey()))
        {
          geometry_msgs::Point op_wrt_world;
          op_wrt_world.x = it.getX();
          op_wrt_world.y = it.getY();
          op_wrt_world.z = it.getZ();

          // TRANSFORM POINT CLOUD WRT WORLD TO ROBOT FRAME
          geometry_msgs::Point op_wrt_robot;
          transformPoint(map_util.getFrameName(), op_wrt_world, robot_param.robot_frame_name, op_wrt_robot);

          vox_index = toLinIndex(op_wrt_robot);

          if ( vox_index >= 0 && vox_index < total_voxel_cnt )
          {
            if (status_param.ego_grid_data.ovox_value[vox_index] != map_util.getMaxOccupancyBeliefValue())
            {
              status_param.ego_grid_data.ovox_value[vox_index] = map_util.getMaxOccupancyBeliefValue();

              if (process_param.visu_flag == true)
              {
                geometry_msgs::Point32 po;
                po.x = op_wrt_robot.x;
                po.y = op_wrt_robot.y;
                po.z = op_wrt_robot.z;

                visu_param.occupancy_pc.points.push_back(po);
              }
            }
          }
        }
        
        else
        {
          map_util.getOctmap() -> deleteNode(it.getKey());
        }
      }
    }

    void updateHeuristicFunctions()
    {
      geometry_msgs::Pose active_goal = goal_util.getActiveGoal();
      int tentacle_cnt = robot_param.tentacle_data.size();
      int tsamp_cnt;
      double delta_len = off_tuning_param.pdist;
      double crash_dist_new;
      double tlen_k;

      double dist2goal = find_Euclidean_distance(active_goal.position, status_param.robot_pose.position);
      int sindex;
      double crash_tlen_k;

      vector<int> tentacle_bin;
      double total_weight;
      double total_weighted_occupancy;
      
      double temp_len;
      double clutterness_value_avg;
      
      /*geometry_msgs::Point ex_best_point_wrt_world;
      /if(status_param.ex_best_tentacle > 0 && status_param.ex_best_sample > 0)
      {
        transformPoint(robot_param.robot_frame_name, robot_param.tentacle_data[status_param.ex_best_tentacle][status_param.ex_best_sample], map_util.getFrameName(), ex_best_point_wrt_world);
      }
      */
      double right_left_weight;

      double max_closeness = 0;
      double max_smoothness_dist = 0;

      // CLEAR NAVIGABILITY, CLEARANCE, clutterness AND CLOSENESS SETS
      heuristic_param.navigability_set.clear();  
      heuristic_param.navigability_set.resize(tentacle_cnt);
      
      heuristic_param.clearance_set.clear();
      heuristic_param.clearance_set.resize(tentacle_cnt);

      heuristic_param.clutterness_set.clear();
      heuristic_param.clutterness_set.resize(tentacle_cnt);
        
      heuristic_param.closeness_set.clear();
      heuristic_param.closeness_set.resize(tentacle_cnt);

      heuristic_param.smoothness_set.clear();
      heuristic_param.smoothness_set.resize(tentacle_cnt);

      // FOR EACH TENTACLE
      for(int k = 0; k < tentacle_cnt; k++)
      {     
        tsamp_cnt = robot_param.tentacle_data[k].size();

        // CALCULATE TOTAL WEIGHT AND TOTAL WEIGHTED OCCUPANCY FOR EACH TENTACLE
        total_weight = 0;
        total_weighted_occupancy = 0;
        tentacle_bin.clear();
        tentacle_bin.resize(tsamp_cnt);

        for(int s = 0; s < robot_param.support_vox_data[k].size(); s++)
        {
          total_weight += robot_param.support_vox_data[k][s].weight;
          total_weighted_occupancy += robot_param.support_vox_data[k][s].weight * status_param.ego_grid_data.ovox_value[robot_param.support_vox_data[k][s].index];
            
          if(robot_param.support_vox_data[k][s].flag && status_param.ego_grid_data.ovox_value[robot_param.support_vox_data[k][s].index] > 0)
          {
            tentacle_bin[robot_param.support_vox_data[k][s].histbin] += 1;       
          }
        }

        // DETERMINE NAVIGABILITY OF THE TENTACLE, 1: NAVIGABLE, 0: NON-NAVIGABLE, -1: TEMPORARILY NAVIGABLE
        int b = 0;
        bool navigability_end_flag = false;
        tlen_k = robot_param.tentacle_data[k].size() * delta_len;
        crash_dist_new = tlen_k * (on_tuning_param.crash_dist * status_param.dummy_current_speed / robot_param.dummy_max_lat_velo);
        if (crash_dist_new < 3*delta_len)
        {
          crash_dist_new = 3*delta_len;
        }
        else if (crash_dist_new > tlen_k)
        {
          crash_dist_new = tlen_k;
        }

        heuristic_param.navigability_set[k] = 1;

        if(process_param.visu_flag == true)
        {
          visu_param.tentacle_visu.markers[k].color.r = 0.0;
          visu_param.tentacle_visu.markers[k].color.g = 1.0;
          visu_param.tentacle_visu.markers[k].color.b = 0.0;
        }

        temp_len = off_tuning_param.tlen;
        status_param.tcrash_bin[k] = tsamp_cnt - 1;

        while (!navigability_end_flag && (b < tsamp_cnt))
        {
          if (tentacle_bin[b] >= on_tuning_param.tbin_obs_cnt_threshold)
          {
            temp_len = delta_len * (b + 1);

            //if(temp_len >= on_tuning_param.crash_dist)
            if (temp_len > crash_dist_new)
            {
              heuristic_param.navigability_set[k] = -1;

              if(process_param.visu_flag == true)
              {
                visu_param.tentacle_visu.markers[k].color.r = 0.0;
                visu_param.tentacle_visu.markers[k].color.g = 0.0;
                visu_param.tentacle_visu.markers[k].color.b = 1.0;
              }
            }
            else
            {
              heuristic_param.navigability_set[k] = 0;

              if(process_param.visu_flag == true)
              {
                visu_param.tentacle_visu.markers[k].color.r = 1.0;
                visu_param.tentacle_visu.markers[k].color.g = 0.0;
                visu_param.tentacle_visu.markers[k].color.b = 0.0;
              }
            }

            if (b == 0)
            {
              status_param.tcrash_bin[k] = b;
            }
            else
            {
              status_param.tcrash_bin[k] = b-1;
            }
            navigability_end_flag = true;
          }
          b++;
        }

        if(heuristic_param.navigability_set[0] == 0)
        {
          heuristic_param.navigability_set[k] == 0;
        }

        // DETERMINE CLEARANCE VALUE
        if(heuristic_param.navigability_set[k] == 1)
        {
          heuristic_param.clearance_set[k] = 0;
        }
        else
        {
          heuristic_param.clearance_set[k] = 1 - temp_len / (delta_len * tsamp_cnt);
        }

        // DETERMINE NEARBY CLUTTERNESS VALUE
        clutterness_value_avg = total_weighted_occupancy / total_weight;
        heuristic_param.clutterness_set[k] = (2 / (1 + exp(-1 * clutterness_value_avg))) - 1;

        // DETERMINE TARGET CLOSENESS VALUE
        crash_tlen_k = status_param.tcrash_bin[k] * delta_len;
        if (dist2goal > crash_tlen_k)
        {
          sindex = status_param.tcrash_bin[k];
        }
        else
        {
          sindex = (robot_param.tentacle_data[k].size()-1) * dist2goal / crash_tlen_k;
        }

        geometry_msgs::Point crash_po_wrt_global;
        transformPoint(robot_param.robot_frame_name, robot_param.tentacle_data[k][sindex], map_util.getFrameName(), crash_po_wrt_global);
        //ROS_INFO_STREAM("Tentacle " << k);
        //print(robot_param.tentacle_data[k][0]);
        //cout << "" << endl;

        heuristic_param.closeness_set[k] = find_Euclidean_distance(active_goal.position, crash_po_wrt_global);

        if(max_closeness < heuristic_param.closeness_set[k])
        {
          max_closeness = heuristic_param.closeness_set[k];
        }

        //DETERMINE SMOOTHNESS VALUE
        //geometry_msgs::Point tmp_best_point_wrt_world;
        //transformPoint(robot_param.robot_frame_name, robot_param.tentacle_data[k][0], map_util.getFrameName(), tmp_best_point_wrt_world);

        //if(status_param.ex_best_tentacle > 0 && status_param.ex_best_sample > 0)
        
        if(status_param.ex_best_tentacle > 0)
        {
          heuristic_param.smoothness_set[k] = find_Euclidean_distance(robot_param.tentacle_data[status_param.ex_best_tentacle][0], robot_param.tentacle_data[k][0]);

          if (robot_param.right_left_data[k] == "R" && robot_param.right_left_data[status_param.ex_best_tentacle] == "L")
          {
            heuristic_param.smoothness_set[k] *= on_tuning_param.smooth_scale;
          }
          else if (robot_param.right_left_data[k] == "L" && robot_param.right_left_data[status_param.ex_best_tentacle] == "R")
          {
            heuristic_param.smoothness_set[k] *= on_tuning_param.smooth_scale;
          }

          if(max_smoothness_dist < heuristic_param.smoothness_set[k])
          {
            max_smoothness_dist = heuristic_param.smoothness_set[k];
          }
        }
        else
        {
          max_smoothness_dist = 1;
          heuristic_param.smoothness_set[k] = 0;
        }

        // RIGHT-LEFT 
        //ROS_INFO_STREAM("tentacle " << k << ": " << robot_param.right_left_data[k]);

      }

      for(int k = 0; k < tentacle_cnt; k++)
      {
        heuristic_param.closeness_set[k] = heuristic_param.closeness_set[k] / max_closeness;
        heuristic_param.smoothness_set[k] = heuristic_param.smoothness_set[k] / max_smoothness_dist;
      }
    }

    // DESCRIPTION: SELECT THE BEST TENTACLE
    void selectBestTentacle()
    {
      int tentacle_cnt = robot_param.tentacle_data.size();
      double tentacle_value;
      double weighted_clearance_value;
      double weighted_clutterness_value;
      double weighted_closeness_value;
      double weighted_smoothness_value;
      double best_tentacle_value = INF;
      bool no_drivable_tentacle = true;

      for(int k = 1; k < tentacle_cnt; k++)
      {
        if( heuristic_param.navigability_set[k] != 0 )
        {
          weighted_clearance_value = on_tuning_param.clear_scale * heuristic_param.clearance_set[k];
          weighted_clutterness_value = on_tuning_param.clutter_scale * heuristic_param.clutterness_set[k];
          weighted_closeness_value = on_tuning_param.close_scale * heuristic_param.closeness_set[k];
          weighted_smoothness_value = on_tuning_param.smooth_scale * heuristic_param.smoothness_set[k];

          tentacle_value = weighted_clearance_value + weighted_clutterness_value + weighted_closeness_value + weighted_smoothness_value;

          /*
          ROS_INFO_STREAM("Tentacle " << k);
          ROS_INFO_STREAM("tentacle_value: " << tentacle_value);
          cout << "" << endl;
          ROS_INFO_STREAM("clear_scale: " << on_tuning_param.clear_scale);
          ROS_INFO_STREAM("clearance: " << heuristic_param.clearance_set[k]);
          ROS_INFO_STREAM("weighted_clearance_value: " << weighted_clearance_value);
          cout << "" << endl;
          ROS_INFO_STREAM("clutter_scale: " << on_tuning_param.clutter_scale);
          ROS_INFO_STREAM("clutterness: " << heuristic_param.clutterness_set[k]);
          ROS_INFO_STREAM("weighted_clutterness_value: " << weighted_clutterness_value);
          cout << "" << endl;
          ROS_INFO_STREAM("close_scale: " << on_tuning_param.close_scale);
          ROS_INFO_STREAM("closeness: " << heuristic_param.closeness_set[k]);
          ROS_INFO_STREAM("weighted_closeness_value: " << weighted_closeness_value);
          cout << "" << endl;
          ROS_INFO_STREAM("smooth_scale: " << on_tuning_param.smooth_scale);
          ROS_INFO_STREAM("smoothness: " << heuristic_param.smoothness_set[k]);
          ROS_INFO_STREAM("weighted_smoothness_value: " << weighted_smoothness_value);
          cout << "-----------------------------------------" << endl;
          cout << "" << endl;
          */

          if(no_drivable_tentacle == true)
          {
              best_tentacle_value = tentacle_value;
              status_param.best_tentacle = k;
              no_drivable_tentacle = false;
          }
          else
          {
            if(best_tentacle_value > tentacle_value)
            {
              best_tentacle_value = tentacle_value;
              status_param.best_tentacle = k;   
            }
          }
        }
      }

      if(no_drivable_tentacle == true)
      {
      	cout << "No drivable tentacle!" << endl;

        status_param.navigability_flag = false;
        
        for(int k = 1; k < tentacle_cnt; k++)
        {
          tentacle_value = on_tuning_param.clear_scale * heuristic_param.clearance_set[k] + 
                           on_tuning_param.clutter_scale * heuristic_param.clutterness_set[k] + 
                           on_tuning_param.close_scale * heuristic_param.closeness_set[k] + 
                           on_tuning_param.smooth_scale * heuristic_param.smoothness_set[k];

          if(k == 1)
          {
              best_tentacle_value = tentacle_value;
              status_param.best_tentacle = 1;
          }
          else
          {
            if(best_tentacle_value > tentacle_value)
            {
              best_tentacle_value = tentacle_value;
              status_param.best_tentacle = k;   
            }
          }
        }
      }
      else
      {
        status_param.navigability_flag = true;
      }

      if(process_param.visu_flag == true)
      {
        visu_param.tentacle_visu.markers[status_param.best_tentacle].color.r = 0.0;
        visu_param.tentacle_visu.markers[status_param.best_tentacle].color.g = 1.0;
        visu_param.tentacle_visu.markers[status_param.best_tentacle].color.b = 1.0;
      }

      /*
      ROS_INFO_STREAM("best tentacle " << status_param.best_tentacle);
      ROS_INFO_STREAM("tentacle_value: " << best_tentacle_value);
      cout << "" << endl;
      ROS_INFO_STREAM("clear_scale: " << on_tuning_param.clear_scale);
      ROS_INFO_STREAM("clearance: " << heuristic_param.clearance_set[status_param.best_tentacle]);
      ROS_INFO_STREAM("weighted_clearance_value: " << on_tuning_param.clear_scale * heuristic_param.clearance_set[status_param.best_tentacle]);
      cout << "" << endl;
      ROS_INFO_STREAM("clutter_scale: " << on_tuning_param.clutter_scale);
      ROS_INFO_STREAM("clutterness: " << heuristic_param.clutterness_set[status_param.best_tentacle]);
      ROS_INFO_STREAM("weighted_clutterness_value: " << on_tuning_param.clutter_scale * heuristic_param.clutterness_set[status_param.best_tentacle]);
      cout << "" << endl;
      ROS_INFO_STREAM("close_scale: " << on_tuning_param.close_scale);
      ROS_INFO_STREAM("closeness: " << heuristic_param.closeness_set[status_param.best_tentacle]);
      ROS_INFO_STREAM("weighted_closeness_value: " << on_tuning_param.close_scale * heuristic_param.closeness_set[status_param.best_tentacle]);
      cout << "" << endl;
      ROS_INFO_STREAM("smooth_scale: " << on_tuning_param.smooth_scale);
      ROS_INFO_STREAM("smoothness: " << heuristic_param.smoothness_set[status_param.best_tentacle]);
      ROS_INFO_STREAM("weighted_smoothness_value: " << on_tuning_param.smooth_scale * heuristic_param.smoothness_set[status_param.best_tentacle]);
      cout << "-----------------------------------------" << endl;
      cout << "" << endl;
      */
        
      status_param.ex_best_tentacle = status_param.best_tentacle;
    }

    void hoverTentabotAtZ1(double x, double y)
    {
      status_param.robot_pose_command.pose.position.x = x;
      status_param.robot_pose_command.pose.position.y = y;
      status_param.robot_pose_command.pose.position.z = 1.0;

      geometry_msgs::Point opiti;
      opiti.x = status_param.robot_pose_command.pose.position.x;
      opiti.y = status_param.robot_pose_command.pose.position.y;
      opiti.z = status_param.robot_pose_command.pose.position.z;

      visu_param.command_visu.points.push_back(opiti);

      status_param.nav_duration += process_param.nav_dt;
    }

    /*
    void hoverRotTentabotAtZ1(double x, double y, double yaw)
    {
      this -> status_param.robot_pose_command.pose.position.x = x;
      this -> status_param.robot_pose_command.pose.position.y = y;
      this -> status_param.robot_pose_command.pose.position.z = 1.0;

      geometry_msgs::Point opiti;
      opiti.x = this -> status_param.robot_pose_command.pose.position.x;
      opiti.y = this -> status_param.robot_pose_command.pose.position.y;
      opiti.z = this -> status_param.robot_pose_command.pose.position.z;

      this -> visu_param.command_visu.points.push_back(opiti);

      this -> status_param.nav_duration += this -> process_param.nav_dt;
    }
    */

    // DESCRIPTION: MOVE THE ROBOT
    void moveTentabot_extend()
    {
      geometry_msgs::Pose active_goal = goal_util.getActiveGoal();
      double dist2goal = find_Euclidean_distance(active_goal.position, status_param.robot_pose.position);
      double first_sample_dist = find_norm(status_param.robot_pose.position);
      //double first_sample_dist = find_norm(robot_param.tentacle_data[status_param.best_tentacle][0]);
      bool isSwitched;
      double dt;

      if(status_param.nav_result == -1)                         // crash
      {
        cout << "OMG! I think I've just hit something!!!" << endl;

        process_param.navexit_flag = true;
      }
      else if(dist2goal < process_param.goal_close_threshold)   // reach goal
      {
        isSwitched = goal_util.switchActiveGoal();

        if(isSwitched == false)
        {
          cout << "Cawabunga! The goal has reached!" << endl;
          process_param.navexit_flag = true;
          status_param.nav_result = 1;
        }
        else
        {
          cout << "Yey! Waypoint #" << goal_util.getActiveGoalIndex()-1 << " has reached!" << endl;

          active_goal = goal_util.getActiveGoal();
          dist2goal = find_Euclidean_distance(active_goal.position, status_param.robot_pose.position);
        }
      }
      else if(status_param.nav_duration > process_param.time_limit)     // time-out
      {
        cout << "Ugh! I am too late..." << endl;

        process_param.navexit_flag = true;
        status_param.nav_result = 0;
      }

      if(!process_param.navexit_flag)
      {
        double next_yaw;
        
        geometry_msgs::Quaternion next_quat_wrt_robot;
        geometry_msgs::Quaternion next_quat_wrt_world;
        geometry_msgs::Point next_point_wrt_robot;
        geometry_msgs::Point next_point_wrt_world;

        geometry_msgs::Point goal_wrt_robot;
        transformPoint(map_util.getFrameName(), active_goal.position, robot_param.robot_frame_name, goal_wrt_robot);

        double max_yaw_angle_dt = robot_param.dummy_max_yaw_velo * process_param.nav_dt;
        double angular_velocity_weight = 1.0;

        if (process_param.counter == 0)
        {
          status_param.robot_pose_command.pose = robot_param.init_robot_pose;
        }

        /*
        if (dist2goal < first_sample_dist)
        {
          double min_speed = 1.0;

          cout << "Goal is close..." << endl;
          next_yaw = atan2(goal_wrt_robot.y, goal_wrt_robot.x);

          status_param.dummy_current_speed = min_speed;

          if(abs(next_yaw) > max_yaw_angle_dt)
          {
            next_yaw *= max_yaw_angle_dt / abs(next_yaw);
          }

          next_yaw *= angular_velocity_weight;

          if (status_param.navigability_flag == false)
          {

            next_yaw = -0.2;

            transformPoint(robot_param.robot_frame_name, next_point_wrt_robot, map_util.getFrameName(), status_param.robot_pose_command.pose.position);

            if (status_param.robot_pose_command.pose.position.z < off_tuning_param.pdist)
            {
              status_param.robot_pose_command.pose.position.z = 1.2 * off_tuning_param.pdist;
            }
          }

          transformOrientation(robot_param.robot_frame_name, 0, 0, next_yaw, map_util.getFrameName(), status_param.robot_pose_command.pose.orientation); 
        }
        */
        //else
        {
          

          double min_speed = 2.0;
          //double ds = (robot_param.dummy_max_lat_velo - min_speed) / off_tuning_param.tyaw_cnt;
          //double speed_index = off_tuning_param.tyaw_cnt - (abs(next_yaw) / (0.5 * off_tuning_param.tyaw)) * (off_tuning_param.tyaw_cnt - 1);
          double speed_inc_dec = 1.0;
          
          //ROS_INFO_STREAM("speed_index: " << speed_index);
          //ROS_INFO_STREAM("ds: " << ds);

          /*
          ROS_INFO_STREAM("best_tentacle: " << status_param.best_tentacle);
          ROS_INFO_STREAM("x: " << next_point_wrt_robot.x);
          ROS_INFO_STREAM("y: " << next_point_wrt_robot.y);
          */

          //ROS_INFO_STREAM("curr: " << status_param.dummy_current_speed);

          // CALCULATE NEXT YAW ANGLE
          next_point_wrt_robot = robot_param.tentacle_data[status_param.best_tentacle][0];
          next_yaw = atan2(next_point_wrt_robot.y, next_point_wrt_robot.x);

          if (abs(next_yaw) > max_yaw_angle_dt)
          {
            next_yaw *= max_yaw_angle_dt / abs(next_yaw);
          }

          //ROS_INFO_STREAM("next_yaw: " << next_yaw*180/PI);
          //ROS_INFO_STREAM("max_yaw_angle_dt: " << max_yaw_angle_dt*180/PI);

          // CALCULATE NEXT POINT
          
          //ROS_INFO_STREAM("ds: " << ds);
          //ROS_INFO_STREAM("speed_index: " << speed_index);

          //double deli = 18;

          //if (min_speed + speed_index * ds >= status_param.dummy_current_speed)
          if (robot_param.dummy_max_lat_velo >= status_param.dummy_current_speed)
          {
            //if (min_speed + speed_index * ds - status_param.dummy_current_speed > speed_inc_dec)
            if (robot_param.dummy_max_lat_velo - status_param.dummy_current_speed > speed_inc_dec)
            {
              status_param.dummy_current_speed += speed_inc_dec;
              //ROS_INFO_STREAM("CINKO 1");
            }
            else
            {
              status_param.dummy_current_speed += robot_param.dummy_max_lat_velo - status_param.dummy_current_speed;
              //status_param.dummy_current_speed += min_speed + speed_index * ds - status_param.dummy_current_speed;
              //ROS_INFO_STREAM("CINKO 2");
            }
          }
          else
          {
            if (status_param.dummy_current_speed - robot_param.dummy_max_lat_velo > speed_inc_dec)
            //if (status_param.dummy_current_speed - min_speed + speed_index * ds > speed_inc_dec)
            {
              status_param.dummy_current_speed -= speed_inc_dec;
              //ROS_INFO_STREAM("CINKO 3");
            }
            else
            {
              status_param.dummy_current_speed -= status_param.dummy_current_speed - robot_param.dummy_max_lat_velo;
              //status_param.dummy_current_speed -= status_param.dummy_current_speed - min_speed + speed_index * ds;
              //ROS_INFO_STREAM("CINKO 4");
            }
          }
          
          //status_param.dummy_current_speed = robot_param.dummy_max_lat_velo;

          if(dist2goal < 0.25*off_tuning_param.tlen || status_param.navigability_flag == false)
          //if(status_param.navigability_flag == false)
          {
            //status_param.dummy_current_speed -= (status_param.dummy_current_speed - 0.25 * robot_param.dummy_max_lat_velo) * exp(0.25 * robot_param.dummy_max_lat_velo - status_param.dummy_current_speed);
            status_param.dummy_current_speed -= 2*speed_inc_dec;
            //ROS_INFO_STREAM("CINKO 5");
          }

          if(status_param.dummy_current_speed > robot_param.dummy_max_lat_velo)
          {
            status_param.dummy_current_speed = robot_param.dummy_max_lat_velo;
          }
          else if(status_param.dummy_current_speed < min_speed)
          {
            status_param.dummy_current_speed = min_speed;
          }

          //ROS_INFO_STREAM("dist2goal:" << dist2goal);
          //ROS_INFO_STREAM("dummy_current_speed: " << status_param.dummy_current_speed);
          
          double max_dist_dt = status_param.dummy_current_speed * process_param.nav_dt;

          //ROS_INFO_STREAM("max_dist_dt: " << max_dist_dt);
          //cout << "" << endl;

          int max_idx = 5;
          if(status_param.tcrash_bin[status_param.best_tentacle] > max_idx)
          {
            status_param.tcrash_bin[status_param.best_tentacle] = max_idx;
          }

          /*
          if (robot_param.tentacle_data[status_param.best_tentacle].size() > status_param.tcrash_bin[status_param.best_tentacle] + 1)
          {
            status_param.ex_best_sample = status_param.tcrash_bin[status_param.best_tentacle] + 1;
          }
          else
          {
            status_param.ex_best_sample = robot_param.tentacle_data[status_param.best_tentacle].size() - 1;
          }
          */
          
          //ROS_INFO_STREAM("tcrash_bin[status_param.best_tentacle]: " << status_param.tcrash_bin[status_param.best_tentacle]);
          next_point_wrt_robot = interpol(robot_param.tentacle_data[status_param.best_tentacle][status_param.tcrash_bin[status_param.best_tentacle]], max_dist_dt);
          //next_point_wrt_robot = interpol(robot_param.tentacle_data[status_param.best_tentacle][0], max_dist_dt);

          next_yaw *= angular_velocity_weight;
          transformOrientation(robot_param.robot_frame_name, 0, 0, next_yaw, map_util.getFrameName(), status_param.robot_pose_command.pose.orientation);
          transformPoint(robot_param.robot_frame_name, next_point_wrt_robot, map_util.getFrameName(), status_param.robot_pose_command.pose.position);

          if (status_param.robot_pose_command.pose.position.z < off_tuning_param.pdist)
          {
            status_param.robot_pose_command.pose.position.z = 1.1 * off_tuning_param.pdist;
          }
        }

        //ROS_INFO_STREAM("final next_yaw: " << next_yaw*180/PI);
        
        status_param.robot_pose_command.header.seq++;
        status_param.robot_pose_command.header.stamp = ros::Time::now();
        status_param.robot_pose_command.header.frame_id = map_util.getFrameName();
        status_param.command_pub.publish(status_param.robot_pose_command);
        
        if (process_param.counter == 0)
        {
          dt = process_param.nav_dt;
          status_param.prev_action_time = ros::Time::now();
        }
        else
        {
          dt = (ros::Time::now() - status_param.prev_action_time).toSec();
          status_param.prev_action_time = ros::Time::now();
          process_param.nav_dt = dt;
          //ROS_INFO_STREAM("dt: " << dt);
   
          double delta_dist = find_Euclidean_distance(status_param.prev_robot_pose.position, status_param.robot_pose.position);
          double speedo = delta_dist / dt;
          
          tf::Quaternion previous_q(status_param.prev_robot_pose.orientation.x, status_param.prev_robot_pose.orientation.y, status_param.prev_robot_pose.orientation.z, status_param.prev_robot_pose.orientation.w);
          tf::Quaternion current_q(status_param.robot_pose.orientation.x, status_param.robot_pose.orientation.y, status_param.robot_pose.orientation.z, status_param.robot_pose.orientation.w);
          tf::Quaternion delta_q = current_q - previous_q;

          tf::Matrix3x3 previous_m(previous_q);
          double previous_roll, previous_pitch, previous_yaw;
          previous_m.getRPY(previous_roll, previous_pitch, previous_yaw);

          tf::Matrix3x3 current_m(current_q);
          double current_roll, current_pitch, current_yaw;
          current_m.getRPY(current_roll, current_pitch, current_yaw);

          double delta_yaw_rate = (current_yaw - previous_yaw) / dt;

          /*
          if(process_param.counter >= 0)
          {
            std::cout << "counter: " << process_param.counter << endl;
            std::cout << "dt: " << dt << endl;
            std::cout << "speedo: " << speedo << std::endl;

            //std::cout << "current_roll: " << current_roll*180/PI << std::endl;
            //std::cout << "current_pitch: " << current_pitch*180/PI << std::endl;
            std::cout << "previous_yaw: " << previous_yaw*180/PI << std::endl;
            std::cout << "current_yaw: " << current_yaw*180/PI << std::endl;
            std::cout << "delta_yaw_rate: " << delta_yaw_rate*180/PI << std::endl;
            std::cout << " " << std::endl;
          }
          */

          status_param.nav_length += delta_dist;
          status_param.prev_robot_pose = status_param.robot_pose;
        }
        status_param.nav_duration += dt;

        geometry_msgs::Point opiti = status_param.robot_pose_command.pose.position;
        visu_param.command_visu.points.push_back(opiti);
        command_history.push_back(opiti);
      }
    }

    /*
    void initializeBSplineOptimization()
    {
      ROS_INFO_STREAM("START OPTIMIZATION INIT");
      const Eigen::Vector4d limits(1.2, 4, 0, 0);

      ewok::Polynomial3DOptimization<10> po(limits*0.6);

      typename ewok::Polynomial3DOptimization<10>::Vector3Array path;

      path.push_back(Eigen::Vector3d(this -> robot_param.init_robot_pose.position.x, this -> robot_param.init_robot_pose.position.y, this -> robot_param.init_robot_pose.position.z));
      
      for (int i = 0; i < goal_util.getGoal().size(); ++i)
      {
        path.push_back(Eigen::Vector3d(goal_util.getGoal()[i].position.x, goal_util.getGoal()[i].position.y, goal_util.getGoal()[i].position.z));
      }

      polynomial_trajectory = po.computeTrajectory(path);

      spline_optimization.reset(new ewok::UniformBSpline3DOptimization<6>(polynomial_trajectory, process_param.nav_dt));

      int num_opt_points = 2;

      for (int i = 0; i < num_opt_points; i++) 
      {
          spline_optimization->addControlPoint(Eigen::Vector3d(this -> robot_param.init_robot_pose.position.x, this -> robot_param.init_robot_pose.position.y, this -> robot_param.init_robot_pose.position.z));
      }

      spline_optimization -> setNumControlPointsOptimized(num_opt_points);
      //spline_optimization->setDistanceBuffer(edrb);
      //spline_optimization->setDistanceThreshold(distance_threshold);
      spline_optimization -> setLimits(limits);

      ROS_INFO_STREAM("END OPTIMIZATION INIT");
    }

    // DESCRIPTION: MOVE THE ROBOT
    void moveOptimizedTentabot()
    {
      geometry_msgs::Pose active_goal = goal_util.getActiveGoal();
      double dist2goal = find_Euclidean_distance(active_goal.position, this -> status_param.robot_pose.position);
      bool isSwitched;

      if(this -> status_param.nav_result == -1)                         // crash
      {
        cout << "OMG! I think I've just hit something!!!" << endl;

        this -> process_param.navexit_flag = true;
      }
      else if(dist2goal < this -> process_param.goal_close_threshold)   // reach goal
      {
        isSwitched = goal_util.switchActiveGoal();

        if(isSwitched == false)
        {
          cout << "Cawabunga! The goal has been reached!" << endl;
          this -> process_param.navexit_flag = true;
          this -> status_param.nav_result = 1;
        }
        else
        {
          cout << "Yey! Waypoint #" << goal_util.getActiveGoalIndex()-1 << " has been reached!" << endl;
        }
      }
      else if( this -> status_param.nav_duration > this -> process_param.time_limit )     // time-out
      {
        cout << "Ugh! I am too late..." << endl;

        this -> process_param.navexit_flag = true;
        this -> status_param.nav_result = 0;
      }
      else
      {
        if(this -> process_param.counter < 20)
        {
          tf::Point np;
          tf::Stamped<tf::Point> next_point_wrt_robot; 
          next_point_wrt_robot.frame_id_ = this -> robot_param.robot_frame_name;
          tf::Stamped<tf::Point> next_point_wrt_world;

          // Set up global trajectory
          int samp_num = 3;
          int samp_scale = (this -> off_tuning_param.tsamp_cnt-1) / samp_num;
          const Eigen::Vector4d limits(1.2, 4, 0, 0);

          ewok::Polynomial3DOptimization<10> po(limits*0.8);
          
          typename ewok::Polynomial3DOptimization<10>::Vector3Array path;

          if (this -> command_history.size() == 0)
          {
            path.push_back(Eigen::Vector3d(this -> robot_param.init_robot_pose.position.x, this -> robot_param.init_robot_pose.position.y, this -> robot_param.init_robot_pose.position.z));
          }
          else
          {
            for (int i = 0; i < this -> command_history.size(); ++i)
            {
              path.push_back(Eigen::Vector3d(this -> command_history[i].x, this -> command_history[i].y, this -> command_history[i].z));
            }
          }

          tmp_marker.points.clear();
          for (int i = 0; i < samp_num; ++i)
          {

            np.setX(this -> robot_param.tentacle_data[this -> status_param.best_tentacle][i*samp_scale].x);
            np.setY(this -> robot_param.tentacle_data[this -> status_param.best_tentacle][i*samp_scale].y);
            np.setZ(this -> robot_param.tentacle_data[this -> status_param.best_tentacle][i*samp_scale].z);
            next_point_wrt_robot.setData(np);
            next_point_wrt_robot.stamp_ = ros::Time(0);
            try
            {
              this -> tflistener -> transformPoint(this -> map_util.getFrameName(), next_point_wrt_robot, next_point_wrt_world);
            }
            catch(tf::TransformException ex)
            {
              ROS_ERROR("%s",ex.what());
              //ros::Duration(1.0).sleep();
            }

            path.push_back(Eigen::Vector3d(next_point_wrt_world.x(), next_point_wrt_world.y(),next_point_wrt_world.z()));

            geometry_msgs::Point tmpo;
            tmpo.x = next_point_wrt_world.x();
            tmpo.y = next_point_wrt_world.y();
            tmpo.z = next_point_wrt_world.z();
            tmp_marker.points.push_back(tmpo);
          }

          std_msgs::ColorRGBA c1, c2, c3;
          c1.r = 1;
          c1.a = 1;
          c2.r = 1;
          c2.g = 1;
          c2.a = 1;
          c3.b = 1;
          c3.a = 1;
          tmp_marker.colors.push_back(c1);
          tmp_marker.colors.push_back(c2);
          tmp_marker.colors.push_back(c3);
          
          polynomial_trajectory = po.computeTrajectory(path);

          polynomial_trajectory->getVisualizationMarkerArray(global_trajectory_marker, "gt", Eigen::Vector3d(1,0,1));

          spline_optimization -> setTrajectory(polynomial_trajectory);

          // B-spline
          int num_opt_points = 2;
          std::vector<ewok::UniformBSpline3DOptimization<6>::Vector3> cp_vector;
          cp_vector.resize(num_opt_points);

          for (int i = 0; i < num_opt_points; ++i)
          {
            np.setX(this -> robot_param.tentacle_data[this -> status_param.best_tentacle][i].x);
            np.setY(this -> robot_param.tentacle_data[this -> status_param.best_tentacle][i].y);
            np.setZ(this -> robot_param.tentacle_data[this -> status_param.best_tentacle][i].z);
            next_point_wrt_robot.setData(np);
            next_point_wrt_robot.stamp_ = ros::Time(0);
            try
            {
              this -> tflistener -> transformPoint(this -> map_util.getFrameName(), next_point_wrt_robot, next_point_wrt_world);
            }
            catch(tf::TransformException ex)
            {
              ROS_ERROR("%s",ex.what());
              //ros::Duration(1.0).sleep();
            }
            cp_vector[i](0) = next_point_wrt_world.x();
            cp_vector[i](1) = next_point_wrt_world.y();
            cp_vector[i](2) = next_point_wrt_world.z();
          }
          
          //spline_optimization -> addControlPoints(cp_vector);

          //ROS_INFO_STREAM("BEFORE OPT:");
          //spline_optimization -> printControlPoints();
          
          spline_optimization -> optimize();
          Eigen::Vector3d pc = spline_optimization->getFirstOptimizationPoint();
          spline_optimization->addLastControlPoint();

          //ROS_INFO_STREAM("AFTER OPT:");
          //spline_optimization -> printControlPoints();

          spline_optimization->getMarkers(current_trajectory_marker);

          //Eigen::Vector3d pc = spline_optimization -> getFirstOptimizedPoint();

          this -> status_param.robot_pose_command.pose.position.x = pc[0];
          this -> status_param.robot_pose_command.pose.position.y = pc[1];
          this -> status_param.robot_pose_command.pose.position.z = pc[2];

          this -> status_param.command_point_pub.publish(this -> status_param.robot_pose_command.pose.position);

          geometry_msgs::Point opiti;
          opiti.x = this -> status_param.robot_pose_command.pose.position.x;
          opiti.y = this -> status_param.robot_pose_command.pose.position.y;
          opiti.z = this -> status_param.robot_pose_command.pose.position.z;

          this -> visu_param.command_visu.points.push_back(opiti);
          this -> command_history.push_back(opiti);

          this -> status_param.nav_duration += this -> process_param.nav_dt;

          ROS_INFO("Command x: %f y: %f z: %f", this -> status_param.robot_pose_command.pose.position.x, this -> status_param.robot_pose_command.pose.position.y, this -> status_param.robot_pose_command.pose.position.z);
        }
        else
        {
          //ROS_INFO("Hovering at x: %f y: %f z: %f", this -> status_param.robot_pose.position.x, this -> status_param.robot_pose.position.y, this -> status_param.robot_pose.position.z);
          hoverTentabotAtZ1(this -> status_param.robot_pose_command.pose.position.x, this -> status_param.robot_pose_command.pose.position.y);
        }
        
      }
    }
    */

    void sendCommandCallback(const ros::TimerEvent& e) 
    {
      if(process_param.navexit_flag == false)
      {
        //ROS_INFO_STREAM("HELLO START");
        auto t1 = std::chrono::high_resolution_clock::now();

        // UPDATE OCCUPANCY VOXELS
        updateOccupancyVox();

        //ROS_INFO_STREAM("HELLO 1");
        auto t2 = std::chrono::high_resolution_clock::now();

        // UPDATE HEURISTIC FUNCTIONS
        updateHeuristicFunctions();

        //ROS_INFO_STREAM("HELLO 2");
        auto t3 = std::chrono::high_resolution_clock::now();

        // SELECT THE BEST TENTACLE
        selectBestTentacle();

        //ROS_INFO_STREAM("HELLO 3");
        auto t4 = std::chrono::high_resolution_clock::now();

        // MOVE THE TENTABOT
        moveTentabot_extend();
        //this -> moveOptimizedTentabot();
        //this -> hoverTentabotAtZ1(-15, 15);

        //ROS_INFO_STREAM("HELLO 4");
        auto t5 = std::chrono::high_resolution_clock::now();

        nav_process_bench <<  std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << "," << 
                                      std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << "," << 
                                      std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << "," << 
                                      std::chrono::duration_cast<std::chrono::nanoseconds>(t5-t4).count() << std::endl;
        //ROS_INFO_STREAM("HELLO END");
      }
      else
      {
        string rand_filename = createFileName();

        // WRITE RESULTS BENCHMARKS
        string bench_result_filename = "/home/akmandor/catkin_ws/src/tentabot/benchmark/tnav/" + rand_filename + "_result_" + map_util.getMapName() + ".csv";
        nav_result_bench.open(bench_result_filename);
        nav_result_bench << "nav_result[1:success/0:time_limit/-1:crash],nav_duration[s],nav_length[m]\n";
        nav_result_bench << to_string(status_param.nav_result) + ",";
        nav_result_bench << to_string(status_param.nav_duration) + ",";
        nav_result_bench << to_string(status_param.nav_length) + "\n";

        cout << "nav_result: " << status_param.nav_result << endl;
        cout << "nav_duration: " << status_param.nav_duration << endl;
        cout << "nav_length: " << status_param.nav_length << endl;

        ros::shutdown();
      }

      process_param.counter++;
    }

    void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
      //ROS_INFO("recieved depth image");

      cv_bridge::CvImageConstPtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvShare(msg);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      const float fx = 554.254691191187;
      const float fy = 554.254691191187;
      const float cx = 320.5;
      const float cy = 240.5;

      tf::StampedTransform transform;
      try
      {
        tflistener -> lookupTransform(map_util.getFrameName(), msg->header.frame_id, msg->header.stamp, transform);
      }
      catch (tf::TransformException &ex) 
      {
        ROS_INFO("Couldn't get transform");
        ROS_WARN("%s",ex.what());
        return;
      }

      Eigen::Affine3d dT_w_c;
      tf::transformTFToEigen(transform, dT_w_c);

      Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

      float* data = (float*) cv_ptr -> image.data;

      auto t1 = std::chrono::high_resolution_clock::now();

      sensor_msgs::PointCloud recent_pc;
      recent_pc.header.frame_id = map_util.getFrameName();
      recent_pc.points.clear();

      for(int u = 0; u < cv_ptr -> image.cols; u += 4) 
      {
        for(int v = 0; v < cv_ptr -> image.rows; v += 4) 
        {
          float val = data[v * cv_ptr -> image.cols + u];

          if(std::isfinite(val)) 
          {
            Eigen::Vector4f p;
            p[0] = val*(u - cx)/fx;
            p[1] = val*(v - cy)/fy;
            p[2] = val;
            p[3] = 1;

            p = T_w_c * p;

            geometry_msgs::Point32 po;
            po.x = p[0];
            po.y = p[1];
            po.z = p[2];
            
            recent_pc.points.push_back(po);
          }
        }
      }

      map_util.setRecentPCMsg(recent_pc);
      //map_util.addLocalPCMsg(recent_pc);
    }

    void odometryCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
      this -> status_param.robot_pose = *msg;

      geometry_msgs::Point opiti;
      opiti.x = this -> status_param.robot_pose.position.x;
      opiti.y = this -> status_param.robot_pose.position.y;
      opiti.z = this -> status_param.robot_pose.position.z;
      this -> visu_param.path_visu.points.push_back(opiti);

      // PUBLISH THE ROBOT, TENTACLES, SAMPLING POINTS AND SUPPORT VOXELS
      publishTentabot();
      //traj_marker_pub.publish(global_trajectory_marker);
      //current_traj_pub.publish(current_trajectory_marker);
      tmp_pub.publish(tmp_marker);

      if(status_param.robot_pose.position.z < 0.1)
      {
        cout << "Did I fall?" << endl;
        status_param.nav_result = -1;
      }
    }
}; // end of class Tentabot