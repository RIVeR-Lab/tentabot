#ifndef TRAJECTORY_SAMPLING_UTILITY_H
#define TRAJECTORY_SAMPLING_UTILITY_H

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
//
// TODO:
// - Add copy constructors.

// --OUTSOURCE LIBRARIES--
#include <std_msgs/Float64MultiArray.h>
#include <tf/message_filter.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/filesystem.hpp>

// --CUSTOM LIBRARIES--
#include "common_utility.h"

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
class TrajectorySamplingUtility
{
  public:

    // DESCRIPTION: TODO...Constructor
    TrajectorySamplingUtility(NodeHandle& nh, string tframe="");

    // DESCRIPTION: TODO...Constructor
    TrajectorySamplingUtility(NodeHandle& nh, vector<vector<geometry_msgs::Point>> tdata, 
                              string tframe,
                              string tsdataset_path);

    // DESCRIPTION: TODO...Constructor
    TrajectorySamplingUtility(NodeHandle& nh,
                              string tframe,
                              double tlen,
                              double tyaw,
                              double tpitch,
                              int tsamp_cnt,
                              int lat_velo_cnt,
                              int tyaw_cnt, 
                              int tpitch_cnt,
                              string tsdataset_path,
                              string trajectory_type="",
                              string tyaw_samp_type="",
                              string tpitch_samp_type="");
    
    // DESCRIPTION: Destructor
    ~TrajectorySamplingUtility();

    // DESCRIPTION: TODO...
    vector<vector<geometry_msgs::Point>> get_trajectory_data();

    // DESCRIPTION: TODO...
    vector<vector<double>> get_velocity_control_data();

    // DESCRIPTION: TODO...
    vector<string> get_trajectory_lrm_data();

    // DESCRIPTION: TODO...
    string get_trajectory_sampling_dataset_path();

    // DESCRIPTION: TODO...
    string get_trajectory_data_path();

    // DESCRIPTION: TODO...
    string get_trajectory_frame();

    // DESCRIPTION: TODO...
    double get_trajectory_time();

    // DESCRIPTION: TODO...
    double get_trajectory_length();

    // DESCRIPTION: TODO...
    double get_trajectory_yaw();

    // DESCRIPTION: TODO...
    double get_trajectory_pitch();

    // DESCRIPTION: TODO...
    int get_trajectory_sampling_count();

    // DESCRIPTION: TODO...
    int get_lateral_velocity_sampling_count();

    // DESCRIPTION: TODO...
    int get_trajectory_yaw_sampling_count();

    // DESCRIPTION: TODO...
    int get_trajectory_pitch_sampling_count();

    // DESCRIPTION: TODO...
    string get_trajectory_type();

    // DESCRIPTION: TODO...
    string get_trajectory_yaw_sampling_type();

    // DESCRIPTION: TODO...
    string get_trajectory_pitch_sampling_type();

    // DESCRIPTION: TODO...
    ros::Publisher get_trajectory_visu_pub();

    // DESCRIPTION: TODO...
    ros::Publisher get_trajectory_sampling_visu_pub();

    // DESCRIPTION: TODO...
    visualization_msgs::MarkerArray get_trajectory_visu();

    // DESCRIPTION: TODO...
    visualization_msgs::MarkerArray get_trajectory_sampling_visu();

    // DESCRIPTION: TODO...
    void set_trajectory_data(vector<vector<geometry_msgs::Point>> new_trajectory_data);

    // DESCRIPTION: TODO...
    void set_velocity_control_data(vector<vector<double>> new_velocity_control_data);

    // DESCRIPTION: TODO...
    void set_trajectory_lrm_data(vector<string> new_trajectory_lrm_data);

    // DESCRIPTION: TODO...
    void set_trajectory_sampling_dataset_path(string new_trajectory_sampling_dataset_path);

    // DESCRIPTION: TODO...
    void set_trajectory_data_path(string new_trajectory_data_path);

    // DESCRIPTION: TODO...
    void set_trajectory_frame(string new_trajectory_frame);

    // DESCRIPTION: TODO...
    void set_trajectory_time(double new_trajectory_time);

    // DESCRIPTION: TODO...
    void set_trajectory_length(double new_trajectory_length);

    // DESCRIPTION: TODO...
    void set_trajectory_yaw(double new_trajectory_yaw);

    // DESCRIPTION: TODO...
    void set_trajectory_pitch(double new_trajectory_pitch);

    // DESCRIPTION: TODO...
    void set_trajectory_sampling_count(int new_trajectory_sampling_count);

    // DESCRIPTION: TODO...
    void set_lateral_velocity_sampling_count(int new_lateral_velocity_sampling_count);

    // DESCRIPTION: TODO...
    void set_trajectory_yaw_sampling_count(int new_trajectory_yaw_sampling_count);

    // DESCRIPTION: TODO...
    void set_trajectory_pitch_sampling_count(int new_trajectory_pitch_sampling_count);

    // DESCRIPTION: TODO...
    void set_trajectory_type(string new_trajectory_type);

    // DESCRIPTION: TODO...
    void set_trajectory_yaw_sampling_type(string new_trajectory_yaw_sampling_type);

    // DESCRIPTION: TODO...
    void set_trajectory_pitch_sampling_type(string new_trajectory_pitch_sampling_type);

    // DESCRIPTION: TODO...
    void set_trajectory_visu_pub(ros::Publisher new_trajectory_visu_pub);

    // DESCRIPTION: TODO...
    void set_trajectory_sampling_visu_pub(ros::Publisher new_trajectory_sampling_visu_pub);

    // DESCRIPTION: TODO...
    void set_trajectory_visu(visualization_msgs::MarkerArray new_trajectory_visu);

    // DESCRIPTION: TODO...
    void set_trajectory_sampling_visu(visualization_msgs::MarkerArray new_trajectory_sampling_visu);

    // DESCRIPTION: TODO...
    void clear_trajectory_data();

    // DESCRIPTION: TODO...
    void clear_velocity_control_data();

    // DESCRIPTION: TODO...
    vector<double> angle_sampling_func(double angle, int dcount);

    // DESCRIPTION: TODO...
    bool isInsideTriangle(double x, double y, double edge_length, double half_angle);

    // DESCRIPTION: TODO...
    void construct_trajectory_data_by_geometry(bool no_restriction=true);

    // DESCRIPTION: TODO...
    // x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    // u = [v(m/s), omega(rad/s)]
    vector<double> simple_car_model(vector<double>& x, vector<double>& u, double dt);

    // DESCRIPTION: TODO...
    vector<geometry_msgs::Point> construct_trajectory_by_model(string model_name, vector<double> x_init, vector<double> u, double dt=0.01);

    // DESCRIPTION: TODO...
    void construct_trajectory_data_by_simple_car_model(double robot_max_lat_velo, double robot_max_yaw_velo, double dt=0.01);

    // DESCRIPTION: TODO...
    void read_trajectory_data(string tdata_path);

    // DESCRIPTION: TODO...
    void read_velocity_control_data(string tdata_path);

    // DESCRIPTION: TODO...
    void fill_trajectory_sampling_visu();

    // DESCRIPTION: TODO...
    void publish_trajectory_sampling();

    // DESCRIPTION: TODO...
    void create_trajectory_data_path();

    // DESCRIPTION: TODO...
    void save_input_data();

    // DESCRIPTION: TODO...
    void save_trajectory_data();

  private:

    vector<vector<geometry_msgs::Point>> trajectory_data;
    vector<vector<double>> velocity_control_data;
    vector<string> trajectory_lrm_data;
    string trajectory_sampling_dataset_path;
    string trajectory_data_path;
    string trajectory_frame;
    double trajectory_time;
    double trajectory_length;
    double trajectory_yaw;
    double trajectory_pitch;
    int trajectory_sampling_count;                              // TODO: Review: number of sample points on the tentacle, range: 1 <= tsamp_cnt, E Z+
    int lateral_velocity_sampling_count;
    int trajectory_yaw_sampling_count;                          // TODO: Review: number of tentacles along yaw direction, range: 1 <= tyaw_cnt, E Z+
    int trajectory_pitch_sampling_count;                        // TODO: Review: number of tentacles along pitch direction, range: 1 <= tpitch_cnt, E Z+
    string trajectory_type;
    string trajectory_yaw_sampling_type;                        // TODO: Review: parameter to adjust yaw angle sampling type of tentacles    
    string trajectory_pitch_sampling_type;                      // TODO: Review: parameter to adjust pitch angle sampling type of tentacles
    ros::Publisher trajectory_visu_pub;
    ros::Publisher trajectory_sampling_visu_pub;
    visualization_msgs::MarkerArray trajectory_visu;
    visualization_msgs::MarkerArray trajectory_sampling_visu;

}; // END of class TrajectorySamplingUtility

#endif