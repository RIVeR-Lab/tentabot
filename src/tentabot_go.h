// LAST UPDATE: 2020.01.15
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

// CUSTOM LIBRARIES:
#include "map_utility.h"

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
  vector<bool> ovox_mask;                       // vector that keeps the availability information of the voxel (false: non-available, true: available)
  vector<int> ovox_available_index;             // vector that keeps indices of the available voxels int the ego-grid
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
  double max_lat_velo;                          						// max forward lateral velocity of the robot, unit: m/s, range: 0 < max_lat_velo, E R+
  double max_yaw_velo;                          						// max yaw angle velocity of the robot, unit: rad/s, range: 0 < max_yaw_velo, E R+
  double max_pitch_velo;                        						// max pitch angle velocity of the robot, unit: rad/s, range: 0 < max_pitch_velo, E R+
  double max_roll_velo;                         						// max roll angle velocity of the robot, unit: rad/s, range: 0 < max_roll_velo, E R+
  geometry_msgs::Pose init_robot_pose;          						// position and orientation of the robot's initial pose with respect to global coordinate system
  string robot_frame_name;                      						// name of the robot frame
  string robot_name;                            						// name of the robot
  vector< vector<geometry_msgs::Point> > tentacle_data;
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
  double flat_scale;                            // parameter to adjust the weight of the flatness value while selecting best tentacle, range: 0 <= flat_scale, E R+, NUA TODO: Change to clutterness
  double close_scale;                           // parameter to adjust the weight of the closeness value while selecting best tentacle, range: 0 <= close_scale, E R+
  double smooth_scale;                          // parameter to adjust the weight of the smoothness value while selecting best tentacle, range: 0 <= smooth_scale, E R+
  double crash_dist;                            // crash distance on the selected tentacle, range: 0 <= crash_dist, E R+
};

struct HeuristicParams
{
  vector<int> navigability_set;
  vector<double> clearance_set;
  vector<double> flatness_set;
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
  int nav_result;
  double nav_length;
  double nav_duration;                                      // navigation duration
  ros::Time prev_action_time;
  ros::Publisher command_pub;
  ros::Time tmp_time;
  int speed_counter;
};

struct VisuParams
{
  ros::Publisher robot_visu_pub;                						// publisher for robot visualization 
  ros::Publisher tentacle_visu_pub;
  ros::Publisher tsamp_visu_pub;
  ros::Publisher support_vox_visu_pub;
  ros::Publisher occupancy_pc_pub;
  ros::Publisher command_visu_pub;
  ros::Publisher path_visu_pub;
  ros::Publisher next_pub;

  visualization_msgs::Marker robot_visu;        						// marker for robot visualization
  visualization_msgs::MarkerArray tentacle_visu;
  visualization_msgs::MarkerArray tsamp_visu;
  visualization_msgs::MarkerArray support_vox_visu;
  sensor_msgs::PointCloud occupancy_pc;
  visualization_msgs::Marker path_visu;
  visualization_msgs::Marker command_visu;
};

// GENERAL FUNCTIONS:
int randint(int from, int to)
{
  std::default_random_engine rng;
  rng.seed(std::random_device()());
  std::uniform_int_distribution<std::mt19937::result_type> randi(from, to);
  return randi(rng);
}

double randdouble(double from, double to)
{
  std::default_random_engine rng;
  rng.seed(std::random_device()());
  std::uniform_real_distribution<float> randd(from, to);
  return randd(rng);
}

vector<double> sampling_func(double mini, double maxi, int snum, string stype)
{
  if(snum <= 2)
  { 
    vector<double> v;
    v.push_back(mini);
    v.push_back(maxi);
    return v;
  }
  else
  {
    vector<double> v(snum);
    int i;
    double srise;
    double srate;
    double s;

    if(stype == "inc")
    {
      v[0] = mini;
      s = mini;
      srise = (maxi - mini) / (snum-1);
      for(i = 0; i < snum-2; i++)
      {
        srate = 2 * srise * (i+1) / (snum-2);
        s = s + srate;  
        v[i+1] = s;
      }
      v[snum-1] = maxi;
    }
    else if(stype == "dec")
    {
      v.push_back(mini);
      s = maxi;
      srise = (maxi - mini) / (snum-1);
      for(i = 0; i < snum-2; i++)
      {
        srate = 2 * srise * (i+1) / (snum-2);
        s = s - srate;  
        v[snum-2-i] = s;
      }
      v[snum-1] = maxi;
    }
    else
    {
      v[0] = mini;
      s = mini;
      srise = (maxi - mini) / (snum-1);
      for(i = 0; i < snum-2; i++)
      {
        s = s + srise;  
        v[i+1] = s;
      }
      v[snum-1] = maxi;
    }
    return v;
  }
}

vector<double> find_limits(vector<geometry_msgs::Point> archy)
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

vector<double> find_closest_dist(vector<geometry_msgs::Point> archy, geometry_msgs::Point cell_center)
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

bool isInsideRectCuboid(geometry_msgs::Point po, geometry_msgs::Point center, double radx, double rady, double radz)
{
  return (po.x >= center.x - radx) && (po.x < center.x + radx) && (po.y >= center.y - rady) && (po.y < center.y + rady) && (po.z >= center.z - radz) && (po.z < center.z + radz);
}

bool isInsideRectCuboid(geometry_msgs::Point32 po, geometry_msgs::Point center, double radx, double rady, double radz)
{
  return (po.x >= center.x - radx) && (po.x < center.x + radx) && (po.y >= center.y - rady) && (po.y < center.y + rady) && (po.z >= center.z - radz) && (po.z < center.z + radz);
}

bool isInsideRectCuboid(geometry_msgs::Point po, double radx, double rady, double radz)
{
  return (po.x >= -radx) && (po.x <= radx) && (po.y >= -rady) && (po.y <= rady) && (po.z >= -radz) && (po.z <= radz);
}

bool isInsideRectCuboid(geometry_msgs::Point32 po, double radx, double rady, double radz)
{
  return (po.x >= -radx) && (po.x <= radx) && (po.y >= -rady) && (po.y <= rady) && (po.z >= -radz) && (po.z <= radz);
}

bool isSameQuat(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
  return (q1.x == q2.x) && (q1.y == q2.y) && (q1.z == q2.z) && (q1.w == q2.w);
}

bool isSimilarQuat(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2, double delta_ang)
{
  tf::Quaternion qu1;
  tf::quaternionMsgToTF(q1, qu1);
  
  tf::Quaternion qu2;
  tf::quaternionMsgToTF(q2, qu2);
  
  tf::Quaternion qu3 = qu2 * qu1.inverse();

  return (qu3.getAngle() < delta_ang);
}

bool boundary_check(int var, int minb, int maxb)
{
  return (var >= minb && var <= maxb);
}

void print_vectori(vector<int>& vec)
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

void print_vectord(vector<double>& vec)
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

void print_vecveci(vector< vector<int> >& vecvec)
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

void print_Point(geometry_msgs::Point po)
{
  cout << "(" << po.x << ", " << po.y << ", " << po.z << ")" << endl; 
}

void print_vectorPoint(vector<geometry_msgs::Point> vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    cout << i << ": (" << vec[i].x << ", " << vec[i].y << ", " << vec[i].z << ")" << endl; 
  }
}

void print_vectorPoint(vector<geometry_msgs::Point32> vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    cout << i << ": (" << vec[i].x << ", " << vec[i].y << ", " << vec[i].z << ")" << endl; 
  }
}

void vecvec2msg(vector< vector<double> >& vecvec, std_msgs::Float64MultiArray& vecvec_msg)
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

void vec2msg(vector<double> vec, std_msgs::Float64MultiArray& vec_msg)
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
    MapUtility map_util;
    VisuParams visu_param;
    ofstream nav_param_bench;
    ofstream nav_pre_bench;
    ofstream nav_process_bench;
    ofstream nav_result_bench;
    ofstream rl_bench;
    tf::TransformListener* tflistener;

    int adjustVnum(double length, double dim)
		{
		  int vnum = ceil(length / dim);
		  if(vnum % 2 == 1)
		  {
		    return (vnum + 1);
		  } 
		  return vnum;
		}

		int toIndex(double pos, int grid_vnum, double grid_vdim)
		{
		  return (0.5 * grid_vnum + floor(pos / grid_vdim));
		}

		int toLinIndex(geometry_msgs::Point32 po, int grid_vnumx, int grid_vnumy, int grid_vnumz, double grid_vdim)
		{
		  int ind_x = toIndex(po.x, grid_vnumx, grid_vdim);
		  int ind_y = toIndex(po.y, grid_vnumy, grid_vdim);
		  int ind_z = toIndex(po.z, grid_vnumz, grid_vdim);
		  return (ind_x + ind_y * grid_vnumx + ind_z * grid_vnumx * grid_vnumy);
		}

		geometry_msgs::Point toPoint(int ind, int grid_vnumx, int grid_vnumy, int grid_vnumz, double grid_vdim)
		{
		  geometry_msgs::Point po; 
		  po.x = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) % grid_vnumx  - 0.5 * grid_vnumx) + 0.5 * grid_vdim;
		  po.y = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) / grid_vnumx  - 0.5 * grid_vnumy) + 0.5 * grid_vdim;
		  po.z = grid_vdim * (ind / (grid_vnumx * grid_vnumy) - 0.5 * grid_vnumz) + 0.5 * grid_vdim;
		  return po;
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
      int min_vox_ind_x = toIndex(archy_limits[0] - this -> off_tuning_param.sdist, this -> off_tuning_param.egrid_vnumx, this -> off_tuning_param.egrid_vdim);
      int max_vox_ind_x = toIndex(archy_limits[1] + this -> off_tuning_param.sdist, this -> off_tuning_param.egrid_vnumx, this -> off_tuning_param.egrid_vdim);

      if(min_vox_ind_x < 0)
      {
        min_vox_ind_x = 0;
      }

      if(max_vox_ind_x >= this -> off_tuning_param.egrid_vnumx)
      {
        max_vox_ind_x = this -> off_tuning_param.egrid_vnumx - 1;
      }

      int min_vox_ind_y = toIndex(archy_limits[2] - this -> off_tuning_param.sdist, this -> off_tuning_param.egrid_vnumy, this -> off_tuning_param.egrid_vdim);
      int max_vox_ind_y = toIndex(archy_limits[3] + this -> off_tuning_param.sdist, this -> off_tuning_param.egrid_vnumy, this -> off_tuning_param.egrid_vdim);

      if(min_vox_ind_y < 0)
      {
        min_vox_ind_y = 0;
      }

      if(max_vox_ind_y >= this -> off_tuning_param.egrid_vnumy)
      {
        max_vox_ind_y = this -> off_tuning_param.egrid_vnumy - 1;
      }

      int min_vox_ind_z = toIndex(archy_limits[4] - this -> off_tuning_param.sdist, this -> off_tuning_param.egrid_vnumz, this -> off_tuning_param.egrid_vdim);
      int max_vox_ind_z = toIndex(archy_limits[5] + this -> off_tuning_param.sdist, this -> off_tuning_param.egrid_vnumz, this -> off_tuning_param.egrid_vdim);

      if(min_vox_ind_z < 0)
      {
        min_vox_ind_z = 0;
      }

      if(max_vox_ind_z >= this -> off_tuning_param.egrid_vnumz)
      {
        max_vox_ind_z = this -> off_tuning_param.egrid_vnumz - 1;
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
            gind = i + j * this -> off_tuning_param.egrid_vnumx + r * this -> off_tuning_param.egrid_vnumx * this -> off_tuning_param.egrid_vnumy;
            cell_center = this -> status_param.ego_grid_data.ovox_pos[gind];
            cd = find_closest_dist(archy, cell_center);

            if(cd[0] <= this -> off_tuning_param.sdist)
            {
              sv.index = gind;     
              sv.histbin = (int) cd[1];
              if(cd[0] <= this -> off_tuning_param.pdist)
              {
                sv.weight = this -> off_tuning_param.sweight_max;
                sv.flag = true;
              }
              else
              {
                sv.weight = this -> off_tuning_param.sweight_max / (this -> off_tuning_param.sweight_scale * (cd[0] - this -> off_tuning_param.pdist));
              }
              svg.push_back(sv);
            }
          }
        }
      }
      return svg;
    }

    void arc_extender_voxel_extractor(vector<geometry_msgs::Point> planar_tentacle_data, double yaw_sample, vector<double> ang_samples, string rot_type, double yaw_offset=0)
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
    }

    // NUA TODO: SEPERATE FOR PITCH AND ROLL
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
      this -> visu_param.robot_visu.color.r = 1.0;
      this -> visu_param.robot_visu.color.g = 1.0;
      this -> visu_param.robot_visu.color.b = 1.0;
      this -> visu_param.robot_visu.color.a = 1.0;
      this -> visu_param.robot_visu.header.frame_id = this -> robot_param.robot_frame_name;
    }

    void publishRobot()
    {
      if(this -> process_param.visu_flag == true)
      {
        this -> visu_param.robot_visu.header.seq++;
        this -> visu_param.robot_visu.header.stamp = ros::Time(0);
        this -> visu_param.robot_visu_pub.publish(this -> visu_param.robot_visu);
      }
    }

    void fillTentacleTsampSupportvoxVisu()
    {
      int tentacle_cnt = this -> robot_param.tentacle_data.size();

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

        for(int p = 0; p < this -> off_tuning_param.tsamp_cnt; p++)
        {
          tentacle_line_strip.points.push_back(this -> robot_param.tentacle_data[k][p]);

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
          std_msgs::ColorRGBA sv_point_color;

          support_vox_points.points.push_back( toPoint( this -> robot_param.support_vox_data[k][s].index, 
                                                        this -> off_tuning_param.egrid_vnumx, 
                                                        this -> off_tuning_param.egrid_vnumy, 
                                                        this -> off_tuning_param.egrid_vnumz, 
                                                        this -> off_tuning_param.egrid_vdim ) );

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
      
      double m = dist / p_norm;
      
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
      
      double m = dist_from_p1 / v_norm;
      
      geometry_msgs::Point pm;
      pm.x = p1.x + m * v.x;
      pm.y = p1.y + m * v.y;
      pm.z = p1.z + m * v.z;

      return pm;
    }

    vector< vector<int> > getTentacleMap(bool log_flag=false)
    {
      int column_num = off_tuning_param.tyaw_cnt;
      int row_num = off_tuning_param.tpitch_cnt;
      int column_origin = 0.5*(column_num-1);
      int row_origin = 0.5*(row_num-1);
      vector< vector<int> > tMap(row_num, vector<int>(column_num));
      tMap[row_origin][column_origin] = 0;

      for(int i = 0; i < row_origin; i++)
      {
          tMap[row_origin - (i+1)][column_origin] = tMap[row_origin][column_origin] + (i+1);
          tMap[row_origin + (i+1)][column_origin] = tMap[row_origin][column_origin] + (row_origin+i+1);
      }

      for(int i = 0; i < row_origin; i++)
      {
        for(int j = 0; j < column_origin; j++)
        {
          if(i == 0)
          {
            tMap[row_origin][column_origin - (j+1)] = tMap[row_origin][column_origin - (j+1) + 1] + row_num;

            if(j == 0)
            { 
              tMap[row_origin][column_origin + (j+1)] = tMap[row_origin][column_origin + (j+1) - 1] + row_num * (column_origin+1);
            }
            else
            {
              tMap[row_origin][column_origin + (j+1)] = tMap[row_origin][column_origin + (j+1) - 1] + row_num;
            }
          }

          tMap[row_origin - (i+1)][column_origin - (j+1)] = tMap[row_origin - (i+1)][column_origin - (j+1) + 1] + row_num;    
          tMap[row_origin + (i+1)][column_origin - (j+1)] = tMap[row_origin + (i+1)][column_origin - (j+1) + 1] + row_num;
          
          if(j == 0)
          {
            tMap[row_origin + (i+1)][column_origin + (j+1)] = tMap[row_origin + (i+1)][column_origin + (j+1) - 1] + row_num * (column_origin+1);
            tMap[row_origin - (i+1)][column_origin + (j+1)] = tMap[row_origin - (i+1)][column_origin + (j+1) - 1] + row_num * (column_origin+1);
          }
          else
          {
            tMap[row_origin + (i+1)][column_origin + (j+1)] = tMap[row_origin + (i+1)][column_origin + (j+1) - 1] + row_num;
            tMap[row_origin - (i+1)][column_origin + (j+1)] = tMap[row_origin - (i+1)][column_origin + (j+1) - 1] + row_num;
          }
        }
      }

      if(log_flag)
      {
        for(int i = 0; i < row_num; i++)
        {
          for(int j = 0; j < column_num; j++)
          {
            if(j == column_num-1)
            {
              cout << tMap[i][j] << " " << endl;
            }
            else
            {
              cout << tMap[i][j] << " ";
            }
          }
        }
      }
      
      return tMap;
    }

  public:
    Tentabot(NodeHandle& nh, tf::TransformListener* listener, ProcessParams& pp, RobotParams& rp, OffTuningParams& offtp, OnTuningParams& ontp, MapUtility& mu)                                                    // constructor
    {
      cout << "Welcome to Tentabot 3D Navigation Simulation! I hope you'll enjoy the ride..." << endl;

      this -> tflistener = new tf::TransformListener;
      this -> tflistener = listener;

      this -> setProcessParams(pp);
      this -> setRobotParams(rp);
      this -> setOffTuningParams(offtp);
      this -> setOnTuningParams(ontp);
      ros::Time t1 = ros::Time::now();
      this -> setEgoGridData();
      ros::Time t2 = ros::Time::now();
      this -> map_util = mu;

      ros::Time t3 = ros::Time::now();
      this -> construct_tentacle_pitchExt();
      ros::Time t4 = ros::Time::now();

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
      this -> status_param.nav_result = 8;
      this -> status_param.nav_length = 0;
      this -> status_param.command_pub = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 10);
      this -> status_param.nav_duration = 0;
      this -> status_param.speed_counter = 0;

      if(this -> process_param.visu_flag == true)
      {
        this -> fillRobotVisu();
        this -> fillTentacleTsampSupportvoxVisu();
        this -> visu_param.occupancy_pc.header.frame_id = this -> robot_param.robot_frame_name;
        this -> fillPathVisu();
        this -> fillCommandVisu();

        this -> visu_param.robot_visu_pub = nh.advertise<visualization_msgs::Marker>(this -> robot_param.robot_name, 100);
        this -> visu_param.tentacle_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("tentacles", 100);
        this -> visu_param.tsamp_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("sampling_points", 100);
        this -> visu_param.support_vox_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("support_voxels", 100);
        this -> visu_param.occupancy_pc_pub = nh.advertise<sensor_msgs::PointCloud>("ego_occupancy_pc", 100);
        this -> visu_param.path_visu_pub = nh.advertise<visualization_msgs::Marker>("path2glory", 100);
        this -> visu_param.command_visu_pub = nh.advertise<visualization_msgs::Marker>("commandante", 100);
        this -> visu_param.next_pub = nh.advertise<visualization_msgs::Marker>("nextPub", 100);
      }

      string rand_filename = createFileName();
      
      // WRITE PARAMETERS
      string bench_param_filename = "/home/akmandor/catkin_ws/src/tentabot/benchmark/tnav/" + rand_filename + "_param_" + this -> map_util.getMapName() + ".csv";
      this -> nav_param_bench.open(bench_param_filename);
      this -> nav_param_bench << "nav_dt," + to_string(this -> process_param.nav_dt) + "\n";
      this -> nav_param_bench << "max_lat_velo," + to_string(this -> robot_param.max_lat_velo) + "\n";
      this -> nav_param_bench << "max_yaw_velo," + to_string(this -> robot_param.max_yaw_velo) + "\n";
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
      this -> nav_param_bench << "flat_scale," + to_string(this -> on_tuning_param.flat_scale) + "\n";
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
      this -> robot_param.max_lat_velo = new_robot_param.max_lat_velo; 
      this -> robot_param.max_yaw_velo = new_robot_param.max_yaw_velo; 
      this -> robot_param.max_pitch_velo = new_robot_param.max_pitch_velo;
      this -> robot_param.max_roll_velo = new_robot_param.max_roll_velo;
      this -> robot_param.init_robot_pose = new_robot_param.init_robot_pose;
      this -> robot_param.robot_frame_name = new_robot_param.robot_frame_name;
      this -> robot_param.robot_name = new_robot_param.robot_name;
      this -> robot_param.tentacle_data = new_robot_param.tentacle_data;
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
      this -> on_tuning_param.flat_scale = new_on_tuning_param.flat_scale;
      this -> on_tuning_param.close_scale = new_on_tuning_param.close_scale;
      this -> on_tuning_param.smooth_scale = new_on_tuning_param.smooth_scale;
      this -> on_tuning_param.crash_dist = new_on_tuning_param.crash_dist;
    }

    void setHeuristicParams(HeuristicParams new_heuristic_param)
    {
      this -> heuristic_param.navigability_set = new_heuristic_param.navigability_set;
      this -> heuristic_param.clearance_set = new_heuristic_param.clearance_set;
      this -> heuristic_param.flatness_set = new_heuristic_param.flatness_set;
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
      this -> status_param.nav_result = new_status_param.nav_result;
      this -> status_param.nav_length = new_status_param.nav_length;
      this -> status_param.prev_action_time = new_status_param.prev_action_time;
      this -> status_param.command_pub = new_status_param.command_pub;
      this -> status_param.nav_duration = new_status_param.nav_duration;
      this -> status_param.speed_counter = new_status_param.speed_counter;
    }

    void setVisuParams(VisuParams new_visu_param)
    {
      this -> visu_param.robot_visu_pub = new_visu_param.robot_visu_pub;
      this -> visu_param.tentacle_visu_pub = new_visu_param.tentacle_visu_pub;
      this -> visu_param.tsamp_visu_pub = new_visu_param.tsamp_visu_pub;
      this -> visu_param.support_vox_visu_pub = new_visu_param.support_vox_visu_pub;
      this -> visu_param.path_visu_pub = new_visu_param.path_visu_pub;
      this -> visu_param.command_visu_pub = new_visu_param.command_visu_pub;
      
      this -> visu_param.robot_visu = new_visu_param.robot_visu;
      this -> visu_param.tentacle_visu = new_visu_param.tentacle_visu;
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
      int total_voxel_cnt = this -> off_tuning_param.egrid_vnumx * this -> off_tuning_param.egrid_vnumy * this -> off_tuning_param.egrid_vnumz;
      
      this -> status_param.ego_grid_data.ovox_pos.resize(total_voxel_cnt);
      
      for(int v = 0; v < total_voxel_cnt; v++)
      {
        this -> status_param.ego_grid_data.ovox_pos[v] = toPoint(v, this -> off_tuning_param.egrid_vnumx, this -> off_tuning_param.egrid_vnumy, this -> off_tuning_param.egrid_vnumz, this -> off_tuning_param.egrid_vdim);
      }

      this -> status_param.ego_grid_data.ovox_value.resize(total_voxel_cnt);
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
      if(this -> process_param.visu_flag == true)
      {
        this -> map_util.publishMap();

        //this -> publishRobot();

        this -> publishTentacleTsampSupportvox();

        this -> publishOccupancy();

        this -> publishPath();

        this -> publishCommand();
      }
    }

    // DESCRIPTION: UPDATE OCCUPANCY GRID CELL FOR THE ROBOT
    void updateOccupancyVox()
    {
      // TRANSFORM POINT CLOUD WRT WORLD TO ROBOT FRAME
      sensor_msgs::PointCloud pc_wrt_robot;

      this -> map_util.addHistoryPCData(this -> map_util.getPCData());

      try
      {
        this -> tflistener -> transformPointCloud(this -> robot_param.robot_frame_name, this -> map_util.getHistoryPCData(), pc_wrt_robot);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.01).sleep();
      }

      // UPDATE THE LINEAR OCCUPANCY GRID AROUND THE ROBOT
      int total_voxel_cnt = this -> status_param.ego_grid_data.ovox_pos.size();
      this -> status_param.ego_grid_data.ovox_value.clear(); 
      this -> status_param.ego_grid_data.ovox_value.resize(total_voxel_cnt);

      if(this -> process_param.visu_flag == true)
      {
        this -> visu_param.occupancy_pc.points.clear();
      }

      int pc_size = pc_wrt_robot.points.size();

      int tmp_index;
      bool no_need = false;

      for (int i = 0; i < pc_size; i++)
      {
        tmp_index = toLinIndex(pc_wrt_robot.points[i], this -> off_tuning_param.egrid_vnumx, this -> off_tuning_param.egrid_vnumy, this -> off_tuning_param.egrid_vnumz, this -> off_tuning_param.egrid_vdim);

        if( tmp_index >= 0 && tmp_index < (this -> off_tuning_param.egrid_vnumx * this -> off_tuning_param.egrid_vnumy * this -> off_tuning_param.egrid_vnumz) )
        {
          this -> status_param.ego_grid_data.ovox_value[tmp_index] = this -> map_util.getMaxOccupancyBeliefValue();

          /*if(no_need == false && isInsideRectCuboid(pc_wrt_robot.points[i], 0.5 * this -> robot_param.width, 0.5 * this -> robot_param.length, 0.5 * this -> robot_param.height))
          {
            this -> status_param.nav_result = -1;
            no_need = true;
          }*/

          if(this -> process_param.visu_flag == true)
          {
            geometry_msgs::Point32 po;
            po.x = this -> status_param.ego_grid_data.ovox_pos[tmp_index].x;
            po.y = this -> status_param.ego_grid_data.ovox_pos[tmp_index].y;
            po.z = this -> status_param.ego_grid_data.ovox_pos[tmp_index].z;

            this -> visu_param.occupancy_pc.points.push_back(po);
          }
        }
      }
    }

    void updateHeuristicFunctions()
    {
      geometry_msgs::Pose active_goal = this -> map_util.getGoalUtil().getActiveGoal();
      int tentacle_cnt = this -> robot_param.tentacle_data.size();

      vector<int> tentacle_bin;
      double total_weight;
      double total_weighted_occupancy;
      
      bool bin_end_flag;
      int bin_index;
      int total_obstacle;
      double temp_len;
      
      double flatness_value_avg;
      
      int tentacle_crash_index = (this -> off_tuning_param.tsamp_cnt - 1) * this -> on_tuning_param.crash_dist / this -> off_tuning_param.tlen;
      geometry_msgs::PointStamped crash_po_wrt_robot;
      crash_po_wrt_robot.header.frame_id = this -> robot_param.robot_frame_name;
      geometry_msgs::PointStamped crash_po_wrt_global;

      double max_closeness = 0;

      geometry_msgs::PointStamped prev_best_crash_po_wrt_robot;
      prev_best_crash_po_wrt_robot.header.frame_id = this -> robot_param.robot_frame_name;
      geometry_msgs::PointStamped prev_best_crash_po_wrt_global;
      double max_smoothness_dist = 0;

      // CLEAR NAVIGABILITY, CLEARANCE, FLATNESS AND CLOSENESS SETS
      this -> heuristic_param.navigability_set.clear();  
      this -> heuristic_param.navigability_set.resize(tentacle_cnt);
      
      this -> heuristic_param.clearance_set.clear();
      this -> heuristic_param.clearance_set.resize(tentacle_cnt);

      this -> heuristic_param.flatness_set.clear();                      //TODO: CHANGE TO CLUTTERNESS
      this -> heuristic_param.flatness_set.resize(tentacle_cnt);
        
      this -> heuristic_param.closeness_set.clear();
      this -> heuristic_param.closeness_set.resize(tentacle_cnt);

      this -> heuristic_param.smoothness_set.clear();
      this -> heuristic_param.smoothness_set.resize(tentacle_cnt);

      // FOR EACH TENTACLE
      for(int k = 0; k < tentacle_cnt; k++)
      {     
        // CALCULATE TOTAL WEIGHT AND TOTAL WEIGHTED OCCUPANCY FOR EACH TENTACLE
        total_weight = 0;
        total_weighted_occupancy = 0;
        tentacle_bin.clear();
        tentacle_bin.resize(this -> off_tuning_param.tsamp_cnt);

        for(int s = 0; s < this -> robot_param.support_vox_data[k].size(); s++)
        {
          total_weight = total_weight + this -> robot_param.support_vox_data[k][s].weight;
          total_weighted_occupancy = total_weighted_occupancy + this -> robot_param.support_vox_data[k][s].weight * this -> status_param.ego_grid_data.ovox_value[this -> robot_param.support_vox_data[k][s].index];
            
          if(this -> robot_param.support_vox_data[k][s].flag && this -> status_param.ego_grid_data.ovox_value[this -> robot_param.support_vox_data[k][s].index] > 0)
          {
            tentacle_bin[this -> robot_param.support_vox_data[k][s].histbin] = tentacle_bin[this -> robot_param.support_vox_data[k][s].histbin] + 1;       
          }
        }

        // DETERMINE NAVIGABILITY OF THE TENTACLE, 1: NAVIGABLE, 0: NON-NAVIGABLE, -1: TEMPORARILY NAVIGABLE
        bin_end_flag = false;
        bin_index = 0;
          
        while(bin_end_flag == false)
        {
          total_obstacle = 0;
          for(int b = 0; b < this -> on_tuning_param.tbin_window; b++)
          {
            if(bin_index < this -> off_tuning_param.tsamp_cnt)
            {
              total_obstacle = total_obstacle + tentacle_bin[bin_index];

              if(total_obstacle >= this -> on_tuning_param.tbin_obs_cnt_threshold)
              {
                temp_len = (this -> off_tuning_param.tlen * (bin_index + 1)) / this -> off_tuning_param.tsamp_cnt;

                if(temp_len >= this -> on_tuning_param.crash_dist)
                {
                  this -> heuristic_param.navigability_set[k] = -1;

                  if(this -> process_param.visu_flag == true)
                  {
                    this -> visu_param.tentacle_visu.markers[k].color.r = 0.0;
                    this -> visu_param.tentacle_visu.markers[k].color.g = 0.0;
                    this -> visu_param.tentacle_visu.markers[k].color.b = 1.0;
                  }
                }
                else
                {
                  this -> heuristic_param.navigability_set[k] = 0;

                  if(this -> process_param.visu_flag == true)
                  {
                    this -> visu_param.tentacle_visu.markers[k].color.r = 1.0;
                    this -> visu_param.tentacle_visu.markers[k].color.g = 0.0;
                    this -> visu_param.tentacle_visu.markers[k].color.b = 0.0;
                  }
                }

                bin_end_flag = true;
                break;
              }
              bin_index++;
            }
            else
            {
              bin_index = this -> off_tuning_param.tsamp_cnt - 1;
              this -> heuristic_param.navigability_set[k] = 1;

              if(this -> process_param.visu_flag == true)
              {
                this -> visu_param.tentacle_visu.markers[k].color.r = 0.0;
                this -> visu_param.tentacle_visu.markers[k].color.g = 1.0;
                this -> visu_param.tentacle_visu.markers[k].color.b = 0.0;
              }

              temp_len = this -> off_tuning_param.tlen;

              bin_end_flag = true;
              break;
            }
          }       
        }

        this -> status_param.tcrash_bin[k] = bin_index;
          
        // DETERMINE CLEARANCE VALUE
        if(this -> heuristic_param.navigability_set[k] == 1)
        {
          this -> heuristic_param.clearance_set[k] = 0;
        }
        else
        {
          this -> heuristic_param.clearance_set[k] = 1 - temp_len / this -> off_tuning_param.tlen;
        }

        // DETERMINE NEARBY CLUTTERNESS VALUE
        flatness_value_avg = total_weighted_occupancy / total_weight;

        this -> heuristic_param.flatness_set[k] = (2 / (1 + exp(-1 * flatness_value_avg))) - 1;

        // DETERMINE TARGET CLOSENESS VALUE
        crash_po_wrt_robot.point = this -> robot_param.tentacle_data[k][tentacle_crash_index];
        crash_po_wrt_robot.header.seq++;
        crash_po_wrt_robot.header.stamp = ros::Time(0);

        try
        {
          this -> tflistener -> transformPoint(this -> map_util.getFrameName(), crash_po_wrt_robot, crash_po_wrt_global);
        }
        catch(tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        this -> heuristic_param.closeness_set[k] = find_Euclidean_distance(active_goal.position, crash_po_wrt_global.point);

        if(max_closeness < this -> heuristic_param.closeness_set[k])
        {
          max_closeness = this -> heuristic_param.closeness_set[k];
        }

        //DETERMINE SMOOTHNESS VALUE
        if(this -> status_param.ex_best_tentacle >= 0)
        {
          prev_best_crash_po_wrt_robot.point = this -> robot_param.tentacle_data[this -> status_param.ex_best_tentacle][tentacle_crash_index];
          prev_best_crash_po_wrt_robot.header.seq++;
          prev_best_crash_po_wrt_robot.header.stamp = ros::Time(0);

          try
          {
            this -> tflistener -> transformPoint(this -> map_util.getFrameName(), prev_best_crash_po_wrt_robot, prev_best_crash_po_wrt_global);
          }
          catch(tf::TransformException ex)
          {
            //ROS_ERROR("%s",ex.what());
            //ros::Duration(1.0).sleep();
          }

          this -> heuristic_param.smoothness_set[k] = find_Euclidean_distance(prev_best_crash_po_wrt_global.point, crash_po_wrt_global.point);

          if(max_smoothness_dist < this -> heuristic_param.smoothness_set[k])
          {
            max_smoothness_dist = this -> heuristic_param.smoothness_set[k];
          }
        }
        else
        {
          max_smoothness_dist = 1;
          this -> heuristic_param.smoothness_set[k] = 0;
        }
      }

      for(int k = 0; k < tentacle_cnt; k++)
      {
        this -> heuristic_param.closeness_set[k] = this -> heuristic_param.closeness_set[k] / max_closeness;
        this -> heuristic_param.smoothness_set[k] = this -> heuristic_param.smoothness_set[k] / max_smoothness_dist;
      }
    }

    // DESCRIPTION: SELECT THE BEST TENTACLE
    void selectBestTentacle(bool isSecondTime)
    {
      int tentacle_cnt = this -> robot_param.tentacle_data.size();
      double tentacle_value;
      double best_tentacle_value = INF;
      bool no_drivable_tentacle = true;

      for(int k = 0; k < tentacle_cnt; k++)
      {
        if( this -> heuristic_param.navigability_set[k] != 0 )
        {
          tentacle_value = this -> on_tuning_param.clear_scale * this -> heuristic_param.clearance_set[k] + 
                           this -> on_tuning_param.flat_scale * this -> heuristic_param.flatness_set[k] + 
                           this -> on_tuning_param.close_scale * this -> heuristic_param.closeness_set[k] + 
                           this -> on_tuning_param.smooth_scale * this -> heuristic_param.smoothness_set[k];

          if(no_drivable_tentacle == true)
          {
              best_tentacle_value = tentacle_value;
              this -> status_param.best_tentacle = k;
          }
          else
          {
            if(best_tentacle_value > tentacle_value)
            {
              best_tentacle_value = tentacle_value;
              this -> status_param.best_tentacle = k;   
            }
          }

          no_drivable_tentacle = false;
        }
      }

      if(no_drivable_tentacle == true)
      {
      	//cout << "No drivable tentacle!" << endl;
        this -> status_param.navigability_flag = false;
        
        bool first = true;
        for(int k = 0; k < tentacle_cnt; k++)
        {
          if( this -> heuristic_param.navigability_set[k] == 0 )
          {
            tentacle_value = this -> on_tuning_param.clear_scale * this -> heuristic_param.clearance_set[k] + 
                             this -> on_tuning_param.flat_scale * this -> heuristic_param.flatness_set[k] + 
                             this -> on_tuning_param.close_scale * this -> heuristic_param.closeness_set[k] + 
                             this -> on_tuning_param.smooth_scale * this -> heuristic_param.smoothness_set[k];

            if(first)
            {
                best_tentacle_value = tentacle_value;
                this -> status_param.best_tentacle = k;
                first = false;
            }
            else
            {
              if(best_tentacle_value > tentacle_value)
              {
                best_tentacle_value = tentacle_value;
                this -> status_param.best_tentacle = k;   
              }
            }
          }
        }
      }
      else
      {
        this -> status_param.navigability_flag = true;
      }
      
      //cout << "SELECTED TENTACLE: " << this -> status_param.best_tentacle << endl;
      if(this -> process_param.visu_flag == true)
      {
        this -> visu_param.tentacle_visu.markers[this -> status_param.best_tentacle].color.r = 0.0;
        this -> visu_param.tentacle_visu.markers[this -> status_param.best_tentacle].color.g = 1.0;
        this -> visu_param.tentacle_visu.markers[this -> status_param.best_tentacle].color.b = 1.0;
      }
        
      this -> status_param.ex_best_tentacle = this -> status_param.best_tentacle;
    }

    // DESCRIPTION: MOVE THE ROBOT
    void moveTentabot()
    {
      geometry_msgs::Pose active_goal = this -> map_util.getGoalUtil().getActiveGoal();
      double dist2goal = find_Euclidean_distance(active_goal.position, this -> status_param.robot_pose.position);
      bool isSwitched;
      bool firstFlag = false;
      double dt;

      if(this -> status_param.nav_result == -1)                         // crash
      {
        cout << "OMG! I think I've just hit something!!!" << endl;

        this -> process_param.navexit_flag = true;
      }
      else if(dist2goal < this -> process_param.goal_close_threshold)   // reach goal
      {
        isSwitched = this -> map_util.getGoalUtil().switchActiveGoal();

        this -> status_param.speed_counter = 0;

        if(isSwitched == false)
        {
          cout << "Cawabunga! The goal has reached!" << endl;
          this -> process_param.navexit_flag = true;
        }
        else
        {
          cout << "Yey! Waypoint #" << this -> map_util.getGoalUtil().getActiveGoalIndex()-1 << " has reached!" << endl;
        }

        this -> status_param.nav_result = 1;
      }
      else if( this -> status_param.nav_duration > this -> process_param.time_limit )     // time-out
      {
        cout << "Ugh! I am too late..." << endl;

        this -> process_param.navexit_flag = true;
        this -> status_param.nav_result = 0;
      }
      else
      {
        this -> status_param.speed_counter++;

        double delta_y;
        double delta_x;
        double next_yaw;

        tf::Point np;
        tf::Point gp;
        gp.setX(active_goal.position.x);
        gp.setY(active_goal.position.y);
        gp.setZ(active_goal.position.z);
        tf::Quaternion nq;
        
        tf::Stamped<tf::Quaternion> next_quat_wrt_robot;
        next_quat_wrt_robot.frame_id_ = this -> robot_param.robot_frame_name;
        tf::Stamped<tf::Quaternion> next_quat_wrt_world;
    
        tf::Stamped<tf::Point> next_point_wrt_robot; 
        next_point_wrt_robot.frame_id_ = this -> robot_param.robot_frame_name;
        tf::Stamped<tf::Point> next_point_wrt_world;

        tf::Stamped<tf::Point> goal_wrt_world;
        goal_wrt_world.setData(gp);
        goal_wrt_world.frame_id_ = this -> map_util.getFrameName();
        tf::Stamped<tf::Point> goal_wrt_robot;

        bool non_stop = true;

        if(this -> status_param.nav_duration <= 0.0001)
        {
          dt = this -> process_param.nav_dt;
          this -> status_param.prev_action_time = ros::Time::now();
          firstFlag = true;
        }
        else
        {
          dt = (ros::Time::now() - this -> status_param.prev_action_time).toSec();
          this -> status_param.prev_action_time = ros::Time::now();
        }
        this -> status_param.nav_duration += dt;

        double current_speed = 0;
        double ds = 1;

        if(dist2goal < 2)
        {
          current_speed = 0.5 * this -> robot_param.max_lat_velo;
        }
        else if(this -> status_param.navigability_flag == false)
        {
          current_speed = 0.25 * this -> robot_param.max_lat_velo;
        }
        else if(ds *  this -> status_param.speed_counter < this -> robot_param.max_lat_velo)
        {
          current_speed = ds *  this -> status_param.speed_counter;
        }
        else
        {
          current_speed = this -> robot_param.max_lat_velo;
        }

        double max_dist_dt = current_speed * dt;
        double max_yaw_angle_dt = this -> robot_param.max_yaw_velo * dt;

        int best_tentacle = this -> status_param.best_tentacle;

        geometry_msgs::Point next_position;

        // SELECT NEXT POSITION WITH RESPECT TO ROBOT
        np.setX(this -> robot_param.tentacle_data[best_tentacle][0].x);
        np.setY(this -> robot_param.tentacle_data[best_tentacle][0].y);
        np.setZ(this -> robot_param.tentacle_data[best_tentacle][0].z);
          
        double tmp_dist = find_norm(this -> robot_param.tentacle_data[best_tentacle][1]);

        if(dist2goal < tmp_dist)
        {
          //cout << "Goal is close..." << endl;

          try
          {
            this -> tflistener -> transformPoint(this -> robot_param.robot_frame_name, goal_wrt_world, goal_wrt_robot);
          }
          catch(tf::TransformException ex)
          {
            ROS_ERROR("%s",ex.what());
            //ros::Duration(1.0).sleep();
          }
              
          delta_y = goal_wrt_robot.y();
          delta_x = goal_wrt_robot.x();

          next_yaw = atan2(delta_y, delta_x);

          if(abs(next_yaw) > 0.25*this -> off_tuning_param.tyaw)
          {
            double turn_radi = 0.5*off_tuning_param.pdist;
            double xoffset = turn_radi * sin( abs(next_yaw) );
            double yoffset = (turn_radi - turn_radi * cos( abs(next_yaw) )) * next_yaw / abs(next_yaw);

            np.setX(xoffset);
            np.setY(yoffset);
            np.setZ(0.1);

            if(abs(next_yaw) > this -> off_tuning_param.tyaw)
            {
              next_yaw = 0.5 * this -> off_tuning_param.tyaw * next_yaw / abs(next_yaw);
            }

            non_stop = false;
          }
        }
          
        if(non_stop)
        {
          next_position = interpol(this -> robot_param.tentacle_data[best_tentacle][this -> status_param.tcrash_bin[best_tentacle]], max_dist_dt);

          double distil = find_norm(next_position);

          np.setX(next_position.x);
          np.setY(next_position.y);
          np.setZ(next_position.z);

          delta_y = next_position.y;
          delta_x = next_position.x;

          next_yaw = atan2(delta_y, delta_x);
        }

        if(abs(next_yaw) > max_yaw_angle_dt)
        {
          next_yaw = next_yaw * max_yaw_angle_dt / abs(next_yaw);
        }

        // SET NEXT ORIENTATION WITH RESPECT TO ROBOT                                     // NUA TODO: Consider also pitch and roll orientations
        nq.setRPY(0, 0, 3*next_yaw);

        next_quat_wrt_robot.setData(nq);
        next_quat_wrt_robot.stamp_ = ros::Time(0);

        next_point_wrt_robot.setData(np);
        next_point_wrt_robot.stamp_ = ros::Time(0);

        try
        {
          this -> tflistener -> transformQuaternion(this -> map_util.getFrameName(), next_quat_wrt_robot, next_quat_wrt_world);
          this -> tflistener -> transformPoint(this -> map_util.getFrameName(), next_point_wrt_robot, next_point_wrt_world);
        }
        catch(tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
          //ros::Duration(1.0).sleep();
          firstFlag = true;
        }

        this -> status_param.robot_pose_command.pose.position.x = next_point_wrt_world.x();
        this -> status_param.robot_pose_command.pose.position.y = next_point_wrt_world.y();
        this -> status_param.robot_pose_command.pose.position.z = next_point_wrt_world.z();

        tf::quaternionTFToMsg(next_quat_wrt_world, this -> status_param.robot_pose_command.pose.orientation);

        this -> status_param.robot_pose_command.header.seq++;
        this -> status_param.robot_pose_command.header.stamp = ros::Time::now();
        this -> status_param.robot_pose_command.header.frame_id = this -> map_util.getFrameName();
        
        if(!firstFlag)
        {
          this -> status_param.command_pub.publish(this -> status_param.robot_pose_command);

          geometry_msgs::Point opiti;
          opiti.x = this -> status_param.robot_pose_command.pose.position.x;
          opiti.y = this -> status_param.robot_pose_command.pose.position.y;
          opiti.z = this -> status_param.robot_pose_command.pose.position.z;

          this -> visu_param.command_visu.points.push_back(opiti);
        }
      }
    }

    void sendCommandCallback(const ros::TimerEvent& e) 
    {
      if(this -> process_param.navexit_flag == false)
      {
        auto t1 = std::chrono::high_resolution_clock::now();

        // UPDATE OCCUPANCY VOXELS
        this -> updateOccupancyVox();

        auto t2 = std::chrono::high_resolution_clock::now();

        // UPDATE HEURISTIC FUNCTIONS
        this -> updateHeuristicFunctions();

        auto t3 = std::chrono::high_resolution_clock::now();

        // SELECT THE BEST TENTACLE
        this -> selectBestTentacle(false);

        auto t4 = std::chrono::high_resolution_clock::now();

        // MOVE THE TENTABOT
        this -> moveTentabot();

        auto t5 = std::chrono::high_resolution_clock::now();

        this -> nav_process_bench <<  std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << "," << 
                                      std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << "," << 
                                      std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << "," << 
                                      std::chrono::duration_cast<std::chrono::nanoseconds>(t5-t4).count() << std::endl;
      }
      else
      {
        string rand_filename = createFileName();

        // WRITE RESULTS BENCHMARKS
        string bench_result_filename = "/home/akmandor/catkin_ws/src/tentabot/benchmark/tnav/" + rand_filename + "_result_" + this -> map_util.getMapName() + ".csv";
        this -> nav_result_bench.open(bench_result_filename);
        this -> nav_result_bench << "nav_result[1:success/0:time_limit/-1:crash],nav_duration[s],nav_length[m]\n";
        this -> nav_result_bench << to_string(this -> status_param.nav_result) + ",";
        this -> nav_result_bench << to_string(this -> status_param.nav_duration) + ",";
        this -> nav_result_bench << to_string(this -> status_param.nav_length) + "\n";

        cout << "nav_result: " << this -> status_param.nav_result << endl;
        cout << "nav_duration: " << this -> status_param.nav_duration << endl;
        cout << "nav_length: " << this -> status_param.nav_length << endl;

        ros::shutdown();
      }

      this -> process_param.counter++;
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
        this -> tflistener -> lookupTransform(this -> map_util.getFrameName(), msg->header.frame_id, msg->header.stamp, transform);
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
      recent_pc.header.frame_id = this -> map_util.getFrameName();
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

      this -> map_util.setPCData(recent_pc);
    }

    void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      this -> status_param.robot_pose = msg -> pose;

      this -> status_param.command_pub.publish(this -> status_param.robot_pose_command);

      geometry_msgs::Point opiti;
      opiti.x = this -> status_param.robot_pose.position.x;
      opiti.y = this -> status_param.robot_pose.position.y;
      opiti.z = this -> status_param.robot_pose.position.z;
      this -> visu_param.path_visu.points.push_back(opiti);

      double delta_dist = find_Euclidean_distance(this -> status_param.prev_robot_pose.position, this -> status_param.robot_pose.position);

      if(this -> status_param.nav_length != 0)
      {
          double dtt = (ros::Time::now() - this -> status_param.tmp_time).toSec();
          this -> status_param.tmp_time = ros::Time::now();
          double speedo = delta_dist/dtt;
          //std::cout << "speedo: " << speedo << std::endl;
      }

      this -> status_param.nav_length += delta_dist;
      this -> status_param.prev_robot_pose.position = this -> status_param.robot_pose.position;

      //std::cout << "delta_dist: " << delta_dist << std::endl;

      // PUBLISH THE ROBOT, TENTACLES, SAMPLING POINTS AND SUPPORT VOXELS
      this -> publishTentabot();

      if(this -> status_param.robot_pose.position.z < 0.15)
      {
        cout << "Did I fall?" << endl;
        this -> status_param.nav_result = -1;
      }
    }
}; // end of class Tentabot