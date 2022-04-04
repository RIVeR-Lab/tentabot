// LAST UPDATE: 2022.03.24
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

// --CUSTOM LIBRARIES--
#include "trajectory_sampling_utility.h"

TrajectorySamplingUtility::TrajectorySamplingUtility(NodeHandle& nh, string tframe)
{
  set_trajectory_frame(tframe);

  trajectory_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectories", 10);
  trajectory_sampling_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_points", 10);
}

TrajectorySamplingUtility::TrajectorySamplingUtility( NodeHandle& nh, vector<vector<geometry_msgs::Point>> tdata, 
                                                      string tframe, 
                                                      string tsdataset_path)
{
  set_trajectory_data(tdata);
  set_trajectory_frame(tframe);
  set_trajectory_sampling_dataset_path(tsdataset_path);
  fill_trajectory_sampling_visu();
  save_trajectory_data();
  save_input_data();

  trajectory_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectories", 10);
  trajectory_sampling_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_points", 10);
}

TrajectorySamplingUtility::TrajectorySamplingUtility( NodeHandle& nh,
                                                      string tframe,
                                                      double tlen,
                                                      int tsamp_cnt,
                                                      double tyaw,
                                                      double tpitch,
                                                      int tyaw_cnt, 
                                                      int tpitch_cnt,
                                                      string tsdataset_path,
                                                      string tyaw_samp_type,
                                                      string tpitch_samp_type)
{
  set_trajectory_frame(tframe);
  set_trajectory_generation_type("geometric");
  set_trajectory_length(tlen);
  set_trajectory_sampling_count(tsamp_cnt);
  set_trajectory_yaw(tyaw);
  set_trajectory_pitch(tpitch);
  set_trajectory_yaw_sampling_count(tyaw_cnt);
  set_trajectory_pitch_sampling_count(tpitch_cnt);
  set_trajectory_sampling_dataset_path(tsdataset_path);
  set_trajectory_yaw_sampling_type(tyaw_samp_type);
  set_trajectory_pitch_sampling_type(tpitch_samp_type);
  
  construct_trajectory_data_by_geometry();
  
  fill_trajectory_sampling_visu();
  save_trajectory_data();
  save_input_data();

  trajectory_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectories", 10);
  trajectory_sampling_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_points", 10);
}

TrajectorySamplingUtility::TrajectorySamplingUtility( NodeHandle& nh,
                                                      string tframe,
                                                      double tlen,
                                                      int tsamp_cnt,
                                                      int lat_velo_cnt,
                                                      int ang_velo_cnt,
                                                      double max_lat_velo,
                                                      double max_yaw_velo,
                                                      double dt,
                                                      string tsdataset_path,
                                                      string tyaw_samp_type,
                                                      string tpitch_samp_type)
{
  set_trajectory_frame(tframe);
  set_trajectory_generation_type("kinematic");
  set_trajectory_length(tlen);
  set_trajectory_sampling_count(tsamp_cnt);
  set_lateral_velocity_sampling_count(lat_velo_cnt);
  set_angular_velocity_sampling_count(ang_velo_cnt);
  set_trajectory_sampling_dataset_path(tsdataset_path);
  set_trajectory_yaw_sampling_type(tyaw_samp_type);
  set_trajectory_pitch_sampling_type(tpitch_samp_type);

  construct_trajectory_data_by_simple_car_model(max_lat_velo, max_yaw_velo, dt);
  
  fill_trajectory_sampling_visu();
  save_trajectory_data();
  save_input_data();

  trajectory_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectories", 10);
  trajectory_sampling_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_points", 10);
}

TrajectorySamplingUtility::~TrajectorySamplingUtility()
{
  //ROS_INFO( "Calling Destructor for Trajectory_Sampling_Utility..." );
}

vector<vector<geometry_msgs::Point>> TrajectorySamplingUtility::get_trajectory_data()
{
  return trajectory_data;
}

vector<vector<double>> TrajectorySamplingUtility::get_velocity_control_data()
{
  return velocity_control_data;
}

vector<string> TrajectorySamplingUtility::get_trajectory_lrm_data()
{
  return trajectory_lrm_data;
}

string TrajectorySamplingUtility::get_trajectory_sampling_dataset_path()
{
  return trajectory_sampling_dataset_path;
}

string TrajectorySamplingUtility::get_trajectory_data_path()
{
  return trajectory_data_path;
}

string TrajectorySamplingUtility::get_trajectory_frame()
{
  return trajectory_frame;
}

string TrajectorySamplingUtility::get_trajectory_generation_type()
{
  return trajectory_generation_type;
}

double TrajectorySamplingUtility::get_trajectory_time()
{
  return trajectory_time;
}

double TrajectorySamplingUtility::get_trajectory_length()
{
  return trajectory_length;
}

double TrajectorySamplingUtility::get_trajectory_yaw()
{
  return trajectory_yaw;
}

double TrajectorySamplingUtility::get_trajectory_pitch()
{
  return trajectory_pitch;
}

int TrajectorySamplingUtility::get_trajectory_sampling_count()
{
  return trajectory_sampling_count;
}

int TrajectorySamplingUtility::get_lateral_velocity_sampling_count()
{
  return lateral_velocity_sampling_count;
}

int TrajectorySamplingUtility::get_angular_velocity_sampling_count()
{
  return angular_velocity_sampling_count;
}

int TrajectorySamplingUtility::get_trajectory_yaw_sampling_count()
{
  return trajectory_yaw_sampling_count;
}

int TrajectorySamplingUtility::get_trajectory_pitch_sampling_count()
{
  return trajectory_pitch_sampling_count;
}

string TrajectorySamplingUtility::get_trajectory_yaw_sampling_type()
{
  return trajectory_yaw_sampling_type;
}

string TrajectorySamplingUtility::get_trajectory_pitch_sampling_type()
{
  return trajectory_pitch_sampling_type;
}

ros::Publisher TrajectorySamplingUtility::get_trajectory_visu_pub()
{
  return trajectory_visu_pub;
}

ros::Publisher TrajectorySamplingUtility::get_trajectory_sampling_visu_pub()
{
  return trajectory_sampling_visu_pub;
}

visualization_msgs::MarkerArray TrajectorySamplingUtility::get_trajectory_visu()
{
  return trajectory_visu;
}

visualization_msgs::MarkerArray TrajectorySamplingUtility::get_trajectory_sampling_visu()
{
  return trajectory_sampling_visu;
}

void TrajectorySamplingUtility::set_trajectory_data(vector<vector<geometry_msgs::Point>> new_trajectory_data)
{
  trajectory_data = new_trajectory_data;
}

void TrajectorySamplingUtility::set_velocity_control_data(vector<vector<double>> new_velocity_control_data)
{
  velocity_control_data = new_velocity_control_data;
}

void TrajectorySamplingUtility::set_trajectory_lrm_data(vector<string> new_trajectory_lrm_data)
{
  trajectory_lrm_data = new_trajectory_lrm_data;
}

void TrajectorySamplingUtility::set_trajectory_sampling_dataset_path(string new_trajectory_sampling_dataset_path)
{
  trajectory_sampling_dataset_path = new_trajectory_sampling_dataset_path;
}

void TrajectorySamplingUtility::set_trajectory_data_path(string new_trajectory_data_path)
{
  trajectory_data_path = new_trajectory_data_path;
}

void TrajectorySamplingUtility::set_trajectory_frame(string new_trajectory_frame)
{
  trajectory_frame = new_trajectory_frame;
}

void TrajectorySamplingUtility::set_trajectory_generation_type(string new_trajectory_generation_type)
{
  trajectory_generation_type = new_trajectory_generation_type;
}

void TrajectorySamplingUtility::set_trajectory_time(double new_trajectory_time)
{
  trajectory_time = new_trajectory_time;
}

void TrajectorySamplingUtility::set_trajectory_length(double new_trajectory_length)
{
  trajectory_length = new_trajectory_length;
}

void TrajectorySamplingUtility::set_trajectory_yaw(double new_trajectory_yaw)
{
  trajectory_yaw = new_trajectory_yaw;
}

void TrajectorySamplingUtility::set_trajectory_pitch(double new_trajectory_pitch)
{
  trajectory_pitch = new_trajectory_pitch;
}

void TrajectorySamplingUtility::set_trajectory_sampling_count(int new_trajectory_sampling_count)
{
  trajectory_sampling_count = new_trajectory_sampling_count;
}

void TrajectorySamplingUtility::set_lateral_velocity_sampling_count(int new_lateral_velocity_sampling_count)
{
  lateral_velocity_sampling_count = new_lateral_velocity_sampling_count;
}

void TrajectorySamplingUtility::set_angular_velocity_sampling_count(int new_angular_velocity_sampling_count)
{
  angular_velocity_sampling_count = new_angular_velocity_sampling_count;
}

void TrajectorySamplingUtility::set_trajectory_yaw_sampling_count(int new_trajectory_yaw_sampling_count)
{
  trajectory_yaw_sampling_count = new_trajectory_yaw_sampling_count;
}

void TrajectorySamplingUtility::set_trajectory_pitch_sampling_count(int new_trajectory_pitch_sampling_count)
{
  trajectory_pitch_sampling_count = new_trajectory_pitch_sampling_count;
}

void TrajectorySamplingUtility::set_trajectory_yaw_sampling_type(string new_trajectory_yaw_sampling_type)
{
  trajectory_yaw_sampling_type = new_trajectory_yaw_sampling_type;
}

void TrajectorySamplingUtility::set_trajectory_pitch_sampling_type(string new_trajectory_pitch_sampling_type)
{
  trajectory_pitch_sampling_type = new_trajectory_pitch_sampling_type;
}

void TrajectorySamplingUtility::set_trajectory_visu_pub(ros::Publisher new_trajectory_visu_pub)
{
  trajectory_visu_pub = new_trajectory_visu_pub;
}

void TrajectorySamplingUtility::set_trajectory_sampling_visu_pub(ros::Publisher new_trajectory_sampling_visu_pub)
{
  trajectory_sampling_visu_pub =  new_trajectory_sampling_visu_pub;
}

void TrajectorySamplingUtility::set_trajectory_visu(visualization_msgs::MarkerArray new_trajectory_visu)
{
  trajectory_visu = new_trajectory_visu;
}

void TrajectorySamplingUtility::set_trajectory_sampling_visu(visualization_msgs::MarkerArray new_trajectory_sampling_visu)
{
  trajectory_sampling_visu = new_trajectory_sampling_visu;
}

void TrajectorySamplingUtility::clear_trajectory_data()
{
  for(int i = 0; i < trajectory_data.size(); i++)
  {
    trajectory_data[i].clear();
  }
  trajectory_data.clear();
}

void TrajectorySamplingUtility::clear_velocity_control_data()
{
  for(int i = 0; i < velocity_control_data.size(); i++)
  {
    velocity_control_data[i].clear();
  }
  velocity_control_data.clear();
}

vector<double> TrajectorySamplingUtility::angle_sampling_func(double angle, int dcount)
{
  vector<double> av;
  av.clear();

  if(dcount <= 0)
  { 
    return av;
  }
  else if(dcount == 1)
  {
    av.push_back(0);
    return av;
  }
  else if(dcount == 2)
  {
    av.push_back(0);
    av.push_back(-0.5*angle);
    return av;
  }
  else if(dcount % 2 == 0)
  {
    av.push_back(0);
    
    int left_count = 0.5 * (dcount - 2) + 1;
    double left_delta = (0.5*angle) / left_count;
    for (int i = 1; i <= left_count; ++i)
    {
      av.push_back(i*left_delta);
    }

    int right_count = left_count - 1;
    double right_delta = (0.5*angle) / right_count;
    for (int i = 1; i <= right_count; ++i)
    {
      av.push_back(-i*right_delta);
    }
    return av;
  }
  else
  {
    av.push_back(0);
    
    int count = 0.5 * (dcount - 1);
    double delta = (0.5*angle) / count;
    for (int i = 1; i <= count; ++i)
    {
      av.push_back(i*delta);
    }
    for (int i = 1; i <= count; ++i)
    {
      av.push_back(-i*delta);
    }
    return av;
  }
}

bool TrajectorySamplingUtility::isInsideTriangle(double x, double y, double edge_length, double half_angle)
{
  double angle = atan2(y, x);
  return (abs(angle) <= abs(half_angle)) && (abs(x) <= edge_length * cos(abs(angle)));
}

void TrajectorySamplingUtility::construct_trajectory_data_by_geometry(bool no_restriction)
{
  clear_trajectory_data();
  trajectory_lrm_data.clear();

  // SAMPLE YAW
  vector<double> yaw_samples = angle_sampling_func(trajectory_yaw, trajectory_yaw_sampling_count);

  // SAMPLE PITCH
  vector<double> pitch_samples = angle_sampling_func(trajectory_pitch, trajectory_pitch_sampling_count);

  // CONSTRUCT TENTACLE (SAMPLED TRAJECTORY) DATA
  int pcount = 0;
  int tcount = 0;
  double x;
  double y;
  double z;
  double ds = 1;
  double delta_dist = trajectory_length / trajectory_sampling_count;

  // TENTACLE 0 (Contains single sampling point at the center of the robot)
  /*
  vector<geometry_msgs::Point> trajectory0;
  geometry_msgs::Point p0;
  p0.x = 0;
  p0.y = 0;
  p0.z = 0;
  trajectory0.push_back(p0);
  trajectory_data.push_back(trajectory0);
  trajectory_lrm_data.push_back("M");
  */

  // ALL OTHER TENTACLES
  for (int j = 0; j < pitch_samples.size(); ++j)
  {
    for (int i = 0; i < yaw_samples.size(); ++i)
    {
      vector<geometry_msgs::Point> trajectory;
      pcount = 0;
      x = 0;
      y = 0;
      z = 0;

      if (no_restriction)
      {
        while( (pcount+1) * delta_dist <= trajectory_length )
        {
          if (trajectory_generation_type == "circular")
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
          trajectory.push_back(p);

          pcount++;
        }
        tcount++;
      }
      else
      {
        while(isInsideTriangle(x, y, trajectory_length, 0.5*trajectory_yaw) && abs(ds*yaw_samples[i]) <= PI)
        {
          if (trajectory_generation_type == "circular")
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
          trajectory.push_back(p);

          pcount++;
        }
        tcount++;
      }

      // ADD TRAJECTORY DATA
      trajectory_data.push_back(trajectory);

      if (yaw_samples[i] > 0)
      {
        trajectory_lrm_data.push_back("L");
      }
      else if (yaw_samples[i] < 0)
      {
        trajectory_lrm_data.push_back("R");
      }
      else
      {
        trajectory_lrm_data.push_back("M");
      }
    }
  }
}

vector<double> TrajectorySamplingUtility::simple_car_model(vector<double>& x, vector<double>& u, double dt)
{
  x[2] += u[1] * dt;
  x[0] += u[0] * cos(x[2]) * dt;
  x[1] += u[0] * sin(x[2]) * dt;

  x[3] = u[0];
  x[4] = u[1];

  return x;
}

vector<geometry_msgs::Point> TrajectorySamplingUtility::construct_trajectory_by_model(string model_name, vector<double> x_init, vector<double> u, double dt)
{
  vector<geometry_msgs::Point> trajectory;
  vector<double> x = x_init;
  double dl;

  double current_ttime = 0;
  double current_tlength = 0;

  geometry_msgs::Point p_new;
  geometry_msgs::Point p_pre;
  p_pre.x = 0;
  p_pre.y = 0;
  p_pre.z = 0;

  while(current_tlength < trajectory_length && current_ttime < trajectory_time)
  {
    if (model_name == "simple_car")
    {
      x = simple_car_model(x, u, dt);
    }

    p_new.x = x[0];
    p_new.y = x[1];
    p_new.z = 0;

    current_ttime += dt;

    dl = find_Euclidean_distance(p_new, p_pre);
    p_pre = p_new;
    current_tlength += dl;
    trajectory.push_back(p_new);

    if (dl == 0)
    {
      break;
    }
  }

  //cout << "TrajectorySamplingUtility::construct_trajectory_by_model -> trajectory size: " << trajectory.size() << endl;
  //print(trajectory);

  return trajectory;
}

void TrajectorySamplingUtility::construct_trajectory_data_by_simple_car_model(double robot_max_lat_velo, double robot_max_yaw_velo, double dt)
{
  clear_trajectory_data();
  trajectory_lrm_data.clear();
  clear_velocity_control_data();

  if (trajectory_data_path == "")
  {
    create_trajectory_data_path();
  }

  string tentabot_path = ros::package::getPath("tentabot") + "/";

  ofstream velocity_control_data_stream;
  velocity_control_data_stream.open(tentabot_path + trajectory_data_path + "velocity_control_data.csv");
  //velocity_control_data_stream << "lateral_velocity_samples[m/s],angular_velocity_samples[rad/s]\n";

  vector<double> lateral_velocity_samples = sampling_func(dt, robot_max_lat_velo, lateral_velocity_sampling_count);
  vector<double> angular_velocity_samples = angle_sampling_func(2*robot_max_yaw_velo, angular_velocity_sampling_count);

  // x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
  vector<double> x_init{0, 0, 0, 0, 0};

  // u = [v(m/s), omega(rad/s)]
  vector<double> u;

  vector<geometry_msgs::Point> traj;
  for (int i = 0; i < lateral_velocity_samples.size(); ++i)
  {
    for (int j = 0; j < angular_velocity_samples.size(); ++j)
    {
      u.push_back(lateral_velocity_samples[i]);
      u.push_back(angular_velocity_samples[j]);
      velocity_control_data.push_back(u);
      velocity_control_data_stream << lateral_velocity_samples[i] << "," << angular_velocity_samples[j] << "\n";

      traj = construct_trajectory_by_model("simple_car", x_init, u, dt);
      subsample(traj, trajectory_sampling_count);
      trajectory_data.push_back(traj);
      
      u.clear();
    }
  }
  velocity_control_data_stream.close();
}

void TrajectorySamplingUtility::read_trajectory_data(string tdata_path)
{
  clear_trajectory_data();

  string::size_type sz;
  string line, spoint, sval;
  string tentabot_path = ros::package::getPath("tentabot") + "/";

  ifstream tdata_stream(tentabot_path + tdata_path + "trajectory_data.csv");

  if (tdata_stream.is_open())
  {
    while ( getline(tdata_stream, line) )
    {
      vector<geometry_msgs::Point> traj;
      stringstream s_line(line);

      while( getline(s_line, spoint, ',') ) 
      {
        vector<float> pv;
        stringstream s_val(spoint);

        while( getline(s_val, sval, ' ') ) 
        {
          pv.push_back(stod(sval, &sz));
        }

        geometry_msgs::Point p;
        p.x = pv[0];
        p.y = pv[1];
        p.z = pv[2];
        traj.push_back(p);
      }
      trajectory_data.push_back(traj);
    }
    tdata_stream.close();
  }
  else 
  {
    cout << "trajectory_sampling_utility::read_trajectory_data -> Unable to open file!" << endl;
  }

  fill_trajectory_sampling_visu();
}

void TrajectorySamplingUtility::read_velocity_control_data(string tdata_path)
{
  clear_velocity_control_data();

  string::size_type sz;
  string line, spoint, sval;
  string tentabot_path = ros::package::getPath("tentabot") + "/";

  ifstream tdata_stream(tentabot_path + tdata_path + "velocity_control_data.csv");

  if (tdata_stream.is_open())
  {
    while ( getline(tdata_stream, line) )
    {
      vector<double> velo_control;
      stringstream s_line(line);

      while( getline(s_line, sval, ',') ) 
      {
        velo_control.push_back(stod(sval, &sz));
      }
      velocity_control_data.push_back(velo_control);
    }
    tdata_stream.close();
  }
  else 
  {
    cout << "trajectory_sampling_utility::read_velocity_control_data -> Unable to open file!" << endl;
  }
}

void TrajectorySamplingUtility::fill_trajectory_sampling_visu()
{
  trajectory_visu.markers.clear();
  trajectory_sampling_visu.markers.clear();
  int tsamp_cnt;

  // VISUALIZE TRAJECTORY
  for(int k = 0; k < trajectory_data.size(); k++)
  {
    // SET TRAJECTORY VISUALIZATION SETTINGS
    visualization_msgs::Marker trajectory_line_strip;
    trajectory_line_strip.ns = "trajectory" + to_string(k);
    trajectory_line_strip.id = k;
    trajectory_line_strip.header.frame_id = trajectory_frame;
    trajectory_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_line_strip.action = visualization_msgs::Marker::ADD;
    trajectory_line_strip.pose.orientation.w = 1.0;
    trajectory_line_strip.scale.x = 0.02;
    trajectory_line_strip.color.r = 1.0;
    trajectory_line_strip.color.g = 1.0;
    trajectory_line_strip.color.b = 1.0;
    trajectory_line_strip.color.a = 1.0;

    // SET SAMPLING POINTS VISUALIZATION SETTINGS
    visualization_msgs::Marker trajectory_tsamp;
    trajectory_tsamp.ns = "sampling_points" + to_string(k);;
    trajectory_tsamp.id = k;
    trajectory_tsamp.header.frame_id = trajectory_frame;
    trajectory_tsamp.type = visualization_msgs::Marker::SPHERE_LIST;
    trajectory_tsamp.action = visualization_msgs::Marker::ADD;
    trajectory_tsamp.pose.orientation.w = 1.0;
    trajectory_tsamp.scale.x = 0.05;
    trajectory_tsamp.scale.y = 0.05;
    trajectory_tsamp.scale.z = 0.05;
    trajectory_tsamp.color.r = 0.0;
    trajectory_tsamp.color.g = 0.0;
    trajectory_tsamp.color.b = 0.0;
    trajectory_tsamp.color.a = 1;

    tsamp_cnt = trajectory_data[k].size();
    for(int p = 0; p < tsamp_cnt; p++)
    {
      if (tsamp_cnt > 1)
      {
        trajectory_line_strip.points.push_back(trajectory_data[k][p]);
      }
      else
      {
        trajectory_line_strip.points.push_back(trajectory_data[k][p]);
        trajectory_line_strip.points.push_back(trajectory_data[k][p]);
      }

      trajectory_tsamp.points.push_back(trajectory_data[k][p]);
    }
    trajectory_visu.markers.push_back(trajectory_line_strip);
    trajectory_sampling_visu.markers.push_back(trajectory_tsamp);
  }
}

void TrajectorySamplingUtility::publish_trajectory_sampling()
{
  for(int k = 0; k < trajectory_visu.markers.size(); k++)
  {
    // UPDATE SEQUENCE AND STAMP FOR TRAJECTORY
    trajectory_visu.markers[k].header.seq++;
    //trajectory_visu.markers[k].header.stamp = ros::Time(0);
    trajectory_visu.markers[k].header.stamp = ros::Time::now();

    // UPDATE SEQUENCE AND STAMP FOR SAMPLING POINTS ON TRAJECTORY 
    trajectory_sampling_visu.markers[k].header.seq++;
    //trajectory_sampling_visu.markers[k].header.stamp = ros::Time(0);
    trajectory_sampling_visu.markers[k].header.stamp = ros::Time::now();
  }

  trajectory_visu_pub.publish(trajectory_visu);
  trajectory_sampling_visu_pub.publish(trajectory_sampling_visu);
}

void TrajectorySamplingUtility::create_trajectory_data_path()
{
  if (trajectory_sampling_dataset_path == "")
  {
    cout << "TrajectorySamplingUtility::create_trajectory_data_path -> trajectory_sampling_dataset_path is not defined!" << endl;
  }
  else
  {
    string trajectory_data_tag = createFileName();
    string tentabot_path = ros::package::getPath("tentabot") + "/";
    trajectory_data_path = trajectory_sampling_dataset_path + trajectory_data_tag + "/";
    boost::filesystem::create_directory(tentabot_path + trajectory_sampling_dataset_path);
    boost::filesystem::create_directory(tentabot_path + trajectory_data_path);
  }
}

void TrajectorySamplingUtility::save_input_data()
{
  if (trajectory_data_path == "")
  {
    create_trajectory_data_path();
  }

  string tentabot_path = ros::package::getPath("tentabot") + "/";

  ofstream input_data_stream;
  input_data_stream.open(tentabot_path + trajectory_data_path + "input_data.csv");
  input_data_stream << "trajectory_frame" << "," << trajectory_frame << "\n";
  input_data_stream << "trajectory_data_path" << "," << trajectory_data_path << "\n";
  input_data_stream << "trajectory_sampling_count" << "," << trajectory_sampling_count << "\n";
  input_data_stream << "trajectory_time" << "," << trajectory_time << "\n";
  input_data_stream << "trajectory_length" << "," << trajectory_length << "\n";
  input_data_stream << "trajectory_generation_type" << "," << trajectory_generation_type << "\n";
  input_data_stream << "trajectory_yaw" << "," << trajectory_yaw << "\n";
  input_data_stream << "trajectory_pitch" << "," << trajectory_pitch << "\n";
  input_data_stream << "trajectory_yaw_sampling_count" << "," << trajectory_yaw_sampling_count << "\n";
  input_data_stream << "trajectory_pitch_sampling_count" << "," << trajectory_pitch_sampling_count << "\n";
  input_data_stream << "trajectory_yaw_sampling_type" << "," << trajectory_yaw_sampling_type << "\n";
  input_data_stream << "trajectory_pitch_sampling_type" << "," << trajectory_pitch_sampling_type << "\n";
  input_data_stream << "lateral_velocity_sampling_count" << "," << lateral_velocity_sampling_count << "\n";
  input_data_stream << "angular_velocity_sampling_count" << "," << angular_velocity_sampling_count << "\n";
  
  input_data_stream.close();
}

void TrajectorySamplingUtility::save_trajectory_data()
{
  if (trajectory_data_path == "")
  {
    create_trajectory_data_path();
  }

  string tentabot_path = ros::package::getPath("tentabot") + "/";

  ofstream trajectory_data_stream;
  trajectory_data_stream.open(tentabot_path + trajectory_data_path + "trajectory_data.csv");

  for (int i = 0; i < trajectory_data.size(); ++i)
  {
    for (int j = 0; j < trajectory_data[i].size(); ++j)
    {
      trajectory_data_stream << to_string(trajectory_data[i][j].x) + " " + to_string(trajectory_data[i][j].y) + " " + to_string(trajectory_data[i][j].z) + ",";
    }
    trajectory_data_stream << "\n";
  }
  trajectory_data_stream.close();

  save_input_data();
}