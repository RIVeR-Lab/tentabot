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

// --CUSTOM LIBRARIES--
#include "tentabot.h"

Tentabot::Tentabot(NodeHandle& nh, 
                   tf::TransformListener* listener, 
                   ProcessParams& pp, 
                   RobotParams& rp, 
                   OffTuningParams& offtp, 
                   OnTuningParams& ontp, 
                   MapUtility& mu, 
                   GoalUtility& gu)
{
  cout << "Welcome to Tentabot Navigation Simulation! I hope you'll enjoy the experience..." << endl;

  tflistener = new tf::TransformListener;
  tflistener = listener;

  setProcessParams(pp);
  setRobotParams(rp);
  setOffTuningParams(offtp);
  setOnTuningParams(ontp);
  ros::Time t1 = ros::Time::now();
  setEgoGridData();
  ros::Time t2 = ros::Time::now();
  map_util = mu;
  goal_util = gu;

  ros::Time t3 = ros::Time::now();
  construct_tentacle_extend(false);
  ros::Time t4 = ros::Time::now();

  status_param.robot_pose.position.x = rp.init_robot_pose.position.x;
  status_param.robot_pose.position.y = rp.init_robot_pose.position.y;
  status_param.robot_pose.position.z = rp.init_robot_pose.position.z;
  status_param.robot_pose.orientation.x = rp.init_robot_pose.orientation.x;
  status_param.robot_pose.orientation.y = rp.init_robot_pose.orientation.y;
  status_param.robot_pose.orientation.z = rp.init_robot_pose.orientation.z;
  status_param.robot_pose.orientation.w = rp.init_robot_pose.orientation.w;
  status_param.prev_robot_pose = status_param.robot_pose;
  status_param.tcrash_bin.resize(off_tuning_param.tyaw_cnt * off_tuning_param.tpitch_cnt);
  status_param.navigability_flag = false;
  status_param.best_tentacle = -1;
  status_param.ex_best_tentacle = -1;
  status_param.ex_best_sample = -1;
  status_param.nav_result = 8;
  status_param.nav_length = 0;
  status_param.command_pub = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 10);
  status_param.nav_duration = 0;
  status_param.speed_counter = 0;
  status_param.dummy_current_speed = 1.0;

  if(process_param.visu_flag == true)
  {
    fillRobotVisu();
    fillTentacleTsampSupportvoxVisu();
    visu_param.occupancy_pc.header.frame_id = robot_param.robot_frame_name;
    fillPathVisu();
    fillCommandVisu();

    visu_param.robot_visu_pub = nh.advertise<visualization_msgs::Marker>(robot_param.robot_name, 100);
    visu_param.tentacle_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("tentacles", 100);
    visu_param.tsamp_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("sampling_points", 100);
    visu_param.support_vox_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("support_voxels", 100);
    visu_param.occupancy_pc_pub = nh.advertise<sensor_msgs::PointCloud>("ego_occupancy_pc", 100);
    visu_param.path_visu_pub = nh.advertise<visualization_msgs::Marker>("path2glory", 100);
    visu_param.command_visu_pub = nh.advertise<visualization_msgs::Marker>("commandante", 100);
    visu_param.next_pub = nh.advertise<visualization_msgs::Marker>("nextPub", 100);
  }

    string rand_filename = createFileName();
  
  // WRITE PARAMETERS
  string bench_param_filename = "/home/akmandor/catkin_ws/src/tentabot/benchmark/tnav/" + rand_filename + "_param_" + map_util.getMapName() + ".csv";
  nav_param_bench.open(bench_param_filename);
  nav_param_bench << "nav_dt," + to_string(process_param.nav_dt) + "\n";
  nav_param_bench << "dummy_max_lat_velo," + to_string(robot_param.dummy_max_lat_velo) + "\n";
  nav_param_bench << "dummy_max_lat_acc," + to_string(robot_param.dummy_max_lat_acc) + "\n";
  nav_param_bench << "dummy_max_yaw_velo," + to_string(robot_param.dummy_max_yaw_velo) + "\n";
  nav_param_bench << "dummy_max_yaw_acc," + to_string(robot_param.dummy_max_yaw_acc) + "\n";
  nav_param_bench << "tyaw_cnt," + to_string(off_tuning_param.tyaw_cnt) + "\n";
  nav_param_bench << "tpitch_cnt," + to_string(off_tuning_param.tpitch_cnt) + "\n";
  nav_param_bench << "tsamp_cnt," + to_string(off_tuning_param.tsamp_cnt) + "\n";
  nav_param_bench << "tlen," + to_string(off_tuning_param.tlen) + "\n";
  nav_param_bench << "tyaw," + to_string(off_tuning_param.tyaw) + "\n";
  nav_param_bench << "tpitch," + to_string(off_tuning_param.tpitch) + "\n";
  nav_param_bench << "pdist," + to_string(off_tuning_param.pdist) + "\n";
  nav_param_bench << "sdist," + to_string(off_tuning_param.sdist) + "\n";
  nav_param_bench << "egrid_vdim," + to_string(off_tuning_param.egrid_vdim) + "\n";
  nav_param_bench << "egrid_vnumx," + to_string(off_tuning_param.egrid_vnumx) + "\n";
  nav_param_bench << "egrid_vnumy," + to_string(off_tuning_param.egrid_vnumy) + "\n";
  nav_param_bench << "egrid_vnumz," + to_string(off_tuning_param.egrid_vnumz) + "\n";
  nav_param_bench << "crash_dist_scale," + to_string(on_tuning_param.crash_dist_scale) + "\n";
  nav_param_bench << "clear_scale," + to_string(on_tuning_param.clear_scale) + "\n";
  nav_param_bench << "clutter_scale," + to_string(on_tuning_param.clutter_scale) + "\n";
  nav_param_bench << "close_scale," + to_string(on_tuning_param.close_scale) + "\n";
  nav_param_bench << "smooth_scale," + to_string(on_tuning_param.smooth_scale) + "\n";
  nav_param_bench.close();

  // WRITE PRE BENCHMARKS
  string pre_bench_filename = "/home/akmandor/catkin_ws/src/tentabot/benchmark/tnav/" + rand_filename + "_pre_" + map_util.getMapName() + ".csv";
  nav_pre_bench.open(pre_bench_filename);
  nav_pre_bench << "grid[s],tentacle_voxel[s]\n";
  nav_pre_bench << to_string( (t2-t1).toSec() ) + ",";
  nav_pre_bench << to_string( (t4-t3).toSec() ) + "\n";
  nav_pre_bench.close();

  // WRITE PROCESS BENCHMARKS
  string process_bench_filename = "/home/akmandor/catkin_ws/src/tentabot/benchmark/tnav/" + rand_filename + "_process_" + map_util.getMapName() + ".csv";
  nav_process_bench.open(process_bench_filename);
  nav_process_bench << "upGVox[ns],upHeur[ns],selectT[ns],moveT[ns]\n";
}

Tentabot::~Tentabot()                                // destructor
{
  //ROS_INFO( "Calling Destructor for Tentabot..." );
  delete tflistener;
  nav_process_bench.close();
  nav_result_bench.close();
  rl_bench.close();
} 

// GET FUNCTIONS
Tentabot::ProcessParams Tentabot::getProcessParams()
{
  return process_param;
}

Tentabot::RobotParams Tentabot::getRobotParams()
{
  return robot_param;
}

Tentabot::OffTuningParams Tentabot::getOffTuningParams()
{
  return off_tuning_param;
}

Tentabot::OnTuningParams Tentabot::getOnTuningParams()
{
  return on_tuning_param;
}

Tentabot::HeuristicParams Tentabot::getHeuristicParams()
{
  return heuristic_param;
}

Tentabot::StatusParams Tentabot::getStatusParams()
{
  return status_param;
}

Tentabot::VisuParams Tentabot::getVisuParams()
{
  return visu_param;
}

ofstream& Tentabot::getNavParamBench()
{
  return nav_param_bench;
}

ofstream& Tentabot::getNavPreBench()
{
  return nav_pre_bench;
}

ofstream& Tentabot::getNavProcessBench()
{
  return nav_process_bench;
}

ofstream& Tentabot::getNavResultBench()
{
  return nav_result_bench;
}

ofstream& Tentabot::getRLBench()
{
  return rl_bench;
}

// SET FUNCTIONS
void Tentabot::setProcessParams(Tentabot::ProcessParams new_process_param)
{
  process_param.visu_flag = new_process_param.visu_flag;
  process_param.online_tuning_flag = new_process_param.online_tuning_flag;
  process_param.time_limit = new_process_param.time_limit;
  process_param.nav_dt = new_process_param.nav_dt;
  process_param.navexit_flag = new_process_param.navexit_flag;
  process_param.goal_close_threshold = new_process_param.goal_close_threshold;
  process_param.counter = new_process_param.counter;
}

void Tentabot::setRobotParams(Tentabot::RobotParams new_robot_param)
{
  robot_param.width = new_robot_param.width;
  robot_param.length = new_robot_param.length;
  robot_param.height = new_robot_param.height;
  robot_param.dummy_max_lat_velo = new_robot_param.dummy_max_lat_velo; 
  robot_param.dummy_max_yaw_velo = new_robot_param.dummy_max_yaw_velo; 
  robot_param.dummy_max_pitch_velo = new_robot_param.dummy_max_pitch_velo;
  robot_param.dummy_max_roll_velo = new_robot_param.dummy_max_roll_velo;
  robot_param.init_robot_pose = new_robot_param.init_robot_pose;
  robot_param.robot_frame_name = new_robot_param.robot_frame_name;
  robot_param.robot_name = new_robot_param.robot_name;
  robot_param.tentacle_data = new_robot_param.tentacle_data;
  robot_param.right_left_data = new_robot_param.right_left_data;
  robot_param.support_vox_data = new_robot_param.support_vox_data;
  robot_param.nav_sensor.freq = new_robot_param.nav_sensor.freq;
  robot_param.nav_sensor.resolution = new_robot_param.nav_sensor.resolution;
  robot_param.nav_sensor.range_x = new_robot_param.nav_sensor.range_x;
  robot_param.nav_sensor.range_y = new_robot_param.nav_sensor.range_y;
  robot_param.nav_sensor.range_z = new_robot_param.nav_sensor.range_z;
  robot_param.nav_sensor.pose_wrt_robot = new_robot_param.nav_sensor.pose_wrt_robot;
  robot_param.nav_sensor.frame_name = new_robot_param.nav_sensor.frame_name;
  
}

void Tentabot::setOffTuningParams(Tentabot::OffTuningParams new_off_tuning_param)
{
  off_tuning_param.tyaw_cnt = new_off_tuning_param.tyaw_cnt;
  off_tuning_param.tpitch_cnt = new_off_tuning_param.tpitch_cnt;
  off_tuning_param.troll_cnt = new_off_tuning_param.troll_cnt;
  off_tuning_param.tsamp_cnt = new_off_tuning_param.tsamp_cnt;
  off_tuning_param.tlat_velo_cnt = new_off_tuning_param.tlat_velo_cnt;
  off_tuning_param.tyaw_velo_cnt = new_off_tuning_param.tyaw_velo_cnt;
  off_tuning_param.tpitch_velo_cnt = new_off_tuning_param.tpitch_velo_cnt;
  off_tuning_param.troll_velo_cnt = new_off_tuning_param.troll_velo_cnt;
  off_tuning_param.tlen = new_off_tuning_param.tlen;
  off_tuning_param.tyaw = new_off_tuning_param.tyaw;
  off_tuning_param.tpitch = new_off_tuning_param.tpitch;
  off_tuning_param.troll = new_off_tuning_param.troll;
  off_tuning_param.tentacle_type = new_off_tuning_param.tentacle_type;
  off_tuning_param.tyaw_samp_type = new_off_tuning_param.tyaw_samp_type;
  off_tuning_param.tpitch_samp_type = new_off_tuning_param.tpitch_samp_type;
  off_tuning_param.troll_samp_type = new_off_tuning_param.troll_samp_type;
  off_tuning_param.pdist = new_off_tuning_param.pdist;
  off_tuning_param.sdist = new_off_tuning_param.sdist;
  off_tuning_param.sweight_max = new_off_tuning_param.sweight_max;
  off_tuning_param.sweight_scale = new_off_tuning_param.sweight_scale;
  off_tuning_param.egrid_vdim = new_off_tuning_param.egrid_vdim;
  off_tuning_param.egrid_vnumx = new_off_tuning_param.egrid_vnumx;
  off_tuning_param.egrid_vnumy = new_off_tuning_param.egrid_vnumy;
  off_tuning_param.egrid_vnumz = new_off_tuning_param.egrid_vnumz;
}

void Tentabot::setOnTuningParams(Tentabot::OnTuningParams new_on_tuning_param)
{
  on_tuning_param.tbin_window = new_on_tuning_param.tbin_window;
  on_tuning_param.tbin_obs_cnt_threshold = new_on_tuning_param.tbin_obs_cnt_threshold;
  on_tuning_param.clear_scale = new_on_tuning_param.clear_scale;
  on_tuning_param.clutter_scale = new_on_tuning_param.clutter_scale;
  on_tuning_param.close_scale = new_on_tuning_param.close_scale;
  on_tuning_param.smooth_scale = new_on_tuning_param.smooth_scale;
  on_tuning_param.crash_dist_scale = new_on_tuning_param.crash_dist_scale;
}

void Tentabot::setHeuristicParams(Tentabot::HeuristicParams new_heuristic_param)
{
  heuristic_param.navigability_set = new_heuristic_param.navigability_set;
  heuristic_param.clearance_set = new_heuristic_param.clearance_set;
  heuristic_param.clutterness_set = new_heuristic_param.clutterness_set;
  heuristic_param.closeness_set = new_heuristic_param.closeness_set;
  heuristic_param.smoothness_set = new_heuristic_param.smoothness_set;
}

void Tentabot::setStatusParams(Tentabot::StatusParams new_status_param)
{
  status_param.prev_robot_pose = new_status_param.prev_robot_pose;
  status_param.robot_pose = new_status_param.robot_pose;
  status_param.robot_pose_command = new_status_param.robot_pose_command;
  status_param.ego_grid_data.ovox_pos = new_status_param.ego_grid_data.ovox_pos;
  status_param.ego_grid_data.ovox_value = new_status_param.ego_grid_data.ovox_value;
  status_param.best_tentacle = new_status_param.best_tentacle;
  status_param.tcrash_bin = new_status_param.tcrash_bin;
  status_param.navigability_flag = new_status_param.navigability_flag;
  status_param.ex_best_tentacle = new_status_param.ex_best_tentacle;
  status_param.ex_best_sample = new_status_param.ex_best_sample;
  status_param.nav_result = new_status_param.nav_result;
  status_param.nav_length = new_status_param.nav_length;
  status_param.prev_action_time = new_status_param.prev_action_time;
  status_param.command_pub = new_status_param.command_pub;
  status_param.command_point_pub = new_status_param.command_point_pub;
  status_param.nav_duration = new_status_param.nav_duration;
  status_param.speed_counter = new_status_param.speed_counter;
  status_param.dummy_current_speed = new_status_param.dummy_current_speed;
}

void Tentabot::setVisuParams(Tentabot::VisuParams new_visu_param)
{
  visu_param.robot_visu_pub = new_visu_param.robot_visu_pub;
  visu_param.tentacle_visu_pub = new_visu_param.tentacle_visu_pub;
  visu_param.tsamp_visu_pub = new_visu_param.tsamp_visu_pub;
  visu_param.support_vox_visu_pub = new_visu_param.support_vox_visu_pub;
  visu_param.path_visu_pub = new_visu_param.path_visu_pub;
  visu_param.command_visu_pub = new_visu_param.command_visu_pub;
  
  visu_param.robot_visu = new_visu_param.robot_visu;
  visu_param.tentacle_visu = new_visu_param.tentacle_visu;
  visu_param.opt_tentacle_visu = new_visu_param.opt_tentacle_visu;
  visu_param.tsamp_visu = new_visu_param.tsamp_visu;
  visu_param.support_vox_visu = new_visu_param.support_vox_visu;
  visu_param.occupancy_pc.header = new_visu_param.occupancy_pc.header;
  visu_param.occupancy_pc.points = new_visu_param.occupancy_pc.points;
  visu_param.occupancy_pc.channels = new_visu_param.occupancy_pc.channels;
  visu_param.path_visu = new_visu_param.path_visu;
  visu_param.command_visu = new_visu_param.command_visu;
}

void Tentabot::setEgoGridData()
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

void Tentabot::setOnTuning(bool flag)
{
  process_param.online_tuning_flag = flag;
}

void Tentabot::print_vecvecd(vector< vector<double> > vecvec)
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

void Tentabot::print_vecvecPoint(vector< vector<geometry_msgs::Point> > vecvec)
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

void Tentabot::clearTentacleData()
{
  int tentacle_cnt = robot_param.tentacle_data.size();

  for(int i = 0; i < tentacle_cnt; i++)
  {
    robot_param.tentacle_data[i].clear();
  }

  robot_param.tentacle_data.clear();

  robot_param.right_left_data.clear();
}

void Tentabot::clearSupportVoxData()
{
  int svdnum = robot_param.support_vox_data.size();

  for(int i = 0; i < svdnum; i++)
  {
    robot_param.support_vox_data[i].clear();
  }

  robot_param.support_vox_data.clear();
}

void Tentabot::publishTentabot()
{
  if(process_param.visu_flag == true)
  {
    goal_util.publishGoal();

    map_util.publishOctmapMsg();

    map_util.publishRecentPCMsg();

    publishRobot();

    publishTentacleTsampSupportvox();

    publishOccupancy();

    publishPath();

    publishCommand();
  }
}

// DESCRIPTION: UPDATE OCCUPANCY GRID CELL FOR THE ROBOT
void Tentabot::updateOccupancyVox()
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

void Tentabot::updateHeuristicFunctions()
{
  geometry_msgs::Pose active_goal = goal_util.getActiveGoal();
  int tentacle_cnt = robot_param.tentacle_data.size();
  int tsamp_cnt;
  double delta_len = off_tuning_param.pdist;
  double crash_dist;
  double tlen_k;

  double dist2goal = find_Euclidean_distance(active_goal.position, status_param.robot_pose.position);
  int sindex;
  double crash_tlen_k;

  vector<int> tentacle_bin;
  double total_weight;
  double total_weighted_occupancy;
  
  double temp_len;
  double clutterness_value_avg;

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
    crash_dist = tlen_k * (on_tuning_param.crash_dist_scale * status_param.dummy_current_speed / robot_param.dummy_max_lat_velo);
    if (crash_dist < 3*delta_len)
    {
      crash_dist = 3*delta_len;
    }
    else if (crash_dist > tlen_k)
    {
      crash_dist = tlen_k;
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

        if (temp_len > crash_dist)
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

    heuristic_param.closeness_set[k] = find_Euclidean_distance(active_goal.position, crash_po_wrt_global);

    if(max_closeness < heuristic_param.closeness_set[k])
    {
      max_closeness = heuristic_param.closeness_set[k];
    }

    //DETERMINE SMOOTHNESS VALUE  
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
  }

  for(int k = 0; k < tentacle_cnt; k++)
  {
    heuristic_param.closeness_set[k] = heuristic_param.closeness_set[k] / max_closeness;
    heuristic_param.smoothness_set[k] = heuristic_param.smoothness_set[k] / max_smoothness_dist;
  }
}

// DESCRIPTION: SELECT THE BEST TENTACLE
void Tentabot::selectBestTentacle()
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

void Tentabot::hoverTentabotAtZ1(double x, double y)
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

// DESCRIPTION: MOVE THE ROBOT
void Tentabot::moveTentabot_extend()
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
    double min_speed = 2.0;
    double speed_inc_dec = 1.0;
    
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
    else
    {
      // CALCULATE NEXT YAW ANGLE
      next_point_wrt_robot = robot_param.tentacle_data[status_param.best_tentacle][0];
      next_yaw = atan2(next_point_wrt_robot.y, next_point_wrt_robot.x);

      if (abs(next_yaw) > max_yaw_angle_dt)
      {
        next_yaw *= max_yaw_angle_dt / abs(next_yaw);
      }

      next_yaw *= angular_velocity_weight;

      // CONDITIONS TO KEEP THE LATERAL SPEED AT THE DESIRED
      if (robot_param.dummy_max_lat_velo >= status_param.dummy_current_speed)
      {
        if (robot_param.dummy_max_lat_velo - status_param.dummy_current_speed > speed_inc_dec)
        {
          status_param.dummy_current_speed += speed_inc_dec;
        }
        else
        {
          status_param.dummy_current_speed += robot_param.dummy_max_lat_velo - status_param.dummy_current_speed;
        }
      }
      else
      {
        if (status_param.dummy_current_speed - robot_param.dummy_max_lat_velo > speed_inc_dec)
        {
          status_param.dummy_current_speed -= speed_inc_dec;
        }
        else
        {
          status_param.dummy_current_speed -= status_param.dummy_current_speed - robot_param.dummy_max_lat_velo;
        }
      }

      // CONDITIONS TO SLOW DOWN THE LATERAL SPEED
      if(dist2goal < 0.25*off_tuning_param.tlen || status_param.navigability_flag == false)
      {
        status_param.dummy_current_speed -= 2*speed_inc_dec;
      }

      // CHECK THE LATERAL SPEED BOUNDARIES
      if(status_param.dummy_current_speed > robot_param.dummy_max_lat_velo)
      {
        status_param.dummy_current_speed = robot_param.dummy_max_lat_velo;
      }
      else if(status_param.dummy_current_speed < min_speed)
      {
        status_param.dummy_current_speed = min_speed;
      }
      
      // CALCULATE NEXT POSITION
      double max_dist_dt = status_param.dummy_current_speed * process_param.nav_dt;

      int max_idx = 5;
      if(status_param.tcrash_bin[status_param.best_tentacle] > max_idx)
      {
        status_param.tcrash_bin[status_param.best_tentacle] = max_idx;
      }

      next_point_wrt_robot = interpol(robot_param.tentacle_data[status_param.best_tentacle][status_param.tcrash_bin[status_param.best_tentacle]], max_dist_dt);
    }

    transformOrientation(robot_param.robot_frame_name, 0, 0, next_yaw, map_util.getFrameName(), status_param.robot_pose_command.pose.orientation);
    transformPoint(robot_param.robot_frame_name, next_point_wrt_robot, map_util.getFrameName(), status_param.robot_pose_command.pose.position);
    
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

void Tentabot::sendCommandCallback(const ros::TimerEvent& e) 
{
  if(process_param.navexit_flag == false)
  {
    // UPDATE OCCUPANCY INFO
    auto t1 = std::chrono::high_resolution_clock::now();
    updateOccupancyVox();
    auto t2 = std::chrono::high_resolution_clock::now();
    
    // UPDATE HEURISTIC FUNCTIONS
    updateHeuristicFunctions();
    auto t3 = std::chrono::high_resolution_clock::now();

    // SELECT THE BEST TENTACLE
    selectBestTentacle();
    auto t4 = std::chrono::high_resolution_clock::now();
    
    // MOVE THE TENTABOT
    moveTentabot_extend();
    auto t5 = std::chrono::high_resolution_clock::now();

    nav_process_bench <<  std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << "," << 
                                  std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << "," << 
                                  std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << "," << 
                                  std::chrono::duration_cast<std::chrono::nanoseconds>(t5-t4).count() << std::endl;
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

void Tentabot::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
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
}

void Tentabot::odometryCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  status_param.robot_pose = *msg;

  geometry_msgs::Point opiti;
  opiti.x = status_param.robot_pose.position.x;
  opiti.y = status_param.robot_pose.position.y;
  opiti.z = status_param.robot_pose.position.z;
  visu_param.path_visu.points.push_back(opiti);

  // PUBLISH THE ROBOT, TENTACLES, SAMPLING POINTS AND SUPPORT VOXELS
  publishTentabot();

  if(status_param.robot_pose.position.z < 0.1)
  {
    cout << "Did I fall?" << endl;
    status_param.nav_result = -1;
  }
}

void Tentabot::transformPoint(string frame_from, 
                              geometry_msgs::Point& p_from_msg, 
                              string frame_to, 
                              geometry_msgs::Point& p_to_msg)
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

void Tentabot::transformPoint(string frame_from, 
                              geometry_msgs::Point32& p_from_msg, 
                              string frame_to, 
                              geometry_msgs::Point32& p_to_msg)
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

void Tentabot::transformOrientation(string frame_from, 
                                    geometry_msgs::Quaternion& q_from_msg, 
                                    string frame_to, 
                                    geometry_msgs::Quaternion& q_to_msg)
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

void Tentabot::transformOrientation(string frame_from, 
                                    double roll_from, 
                                    double pitch_from, 
                                    double yaw_from, 
                                    string frame_to, 
                                    geometry_msgs::Quaternion& q_to_msg)
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

int Tentabot::toIndex(double pos, int grid_vnum)
{   
  return (0.5 * grid_vnum + floor(pos / off_tuning_param.egrid_vdim));
}

int Tentabot::toLinIndex(geometry_msgs::Point po)
{
  int grid_vnumx = off_tuning_param.egrid_vnumx;
  int grid_vnumy = off_tuning_param.egrid_vnumy;
  int grid_vnumz = off_tuning_param.egrid_vnumz;

  int ind_x = toIndex(po.x, grid_vnumx);
  int ind_y = toIndex(po.y, grid_vnumy);
  int ind_z = toIndex(po.z, grid_vnumz);

  return (ind_x + ind_y * grid_vnumx + ind_z * grid_vnumx * grid_vnumy);
}

int Tentabot::toLinIndex(geometry_msgs::Point32 po)
{
  int grid_vnumx = off_tuning_param.egrid_vnumx;
  int grid_vnumy = off_tuning_param.egrid_vnumy;
  int grid_vnumz = off_tuning_param.egrid_vnumz;

  int ind_x = toIndex(po.x, grid_vnumx);
  int ind_y = toIndex(po.y, grid_vnumy);
  int ind_z = toIndex(po.z, grid_vnumz);

  return (ind_x + ind_y * grid_vnumx + ind_z * grid_vnumx * grid_vnumy);
}

void Tentabot::toPoint(int ind, geometry_msgs::Point& po)
{
    int grid_vnumx = off_tuning_param.egrid_vnumx;
    int grid_vnumy = off_tuning_param.egrid_vnumy;
    int grid_vnumz = off_tuning_param.egrid_vnumz;
    double grid_vdim = off_tuning_param.egrid_vdim;

    po.x = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) % grid_vnumx  - 0.5 * grid_vnumx) + 0.5 * grid_vdim;
    po.y = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) / grid_vnumx  - 0.5 * grid_vnumy) + 0.5 * grid_vdim;
    po.z = grid_vdim * (ind / (grid_vnumx * grid_vnumy) - 0.5 * grid_vnumz) + 0.5 * grid_vdim;
}

void Tentabot::toPoint(int ind, geometry_msgs::Point32& po)
{
  int grid_vnumx = off_tuning_param.egrid_vnumx;
  int grid_vnumy = off_tuning_param.egrid_vnumy;
  int grid_vnumz = off_tuning_param.egrid_vnumz;
  double grid_vdim = off_tuning_param.egrid_vdim;

  po.x = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) % grid_vnumx  - 0.5 * grid_vnumx) + 0.5 * grid_vdim;
  po.y = grid_vdim * ((ind % (grid_vnumx * grid_vnumy)) / grid_vnumx  - 0.5 * grid_vnumy) + 0.5 * grid_vdim;
  po.z = grid_vdim * (ind / (grid_vnumx * grid_vnumy) - 0.5 * grid_vnumz) + 0.5 * grid_vdim;
}

bool Tentabot::isInsideRectCuboid(geometry_msgs::Point32 po)
{
  double radx = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumx;
  double rady = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumy;
  double radz = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumz;

  return (po.x >= -radx) && (po.x < radx) && (po.y >= -rady) && (po.y < rady) && (po.z >= -radz) && (po.z < radz);
}

bool Tentabot::isInsideRectCuboid(geometry_msgs::Point32 po, geometry_msgs::Point center)
{
  double radx = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumx;
  double rady = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumy;
  double radz = 0.5 * off_tuning_param.egrid_vdim * off_tuning_param.egrid_vnumz;

  return (po.x >= center.x - radx) && (po.x < center.x + radx) && (po.y >= center.y - rady) && (po.y < center.y + rady) && (po.z >= center.z - radz) && (po.z < center.z + radz);
}

bool Tentabot::isInsideTriangle(double x, double y, double edge_length, double half_angle)
{
  double angle = atan2(y, x);
  return (abs(angle) <= abs(half_angle)) && (abs(x) <= edge_length * cos(abs(angle)));
}

vector<geometry_msgs::Point> Tentabot::arc_by_radi_ang(geometry_msgs::Point from, 
                                                       double radius, 
                                                       double angle, 
                                                       int sample_cnt)
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

vector<geometry_msgs::Point> Tentabot::line_by_len_ang(geometry_msgs::Point from, 
                                                       double length, 
                                                       double angle, 
                                                       int sample_cnt)
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

geometry_msgs::Point Tentabot::rotate3d(geometry_msgs::Point po, 
                                        double ang, 
                                        string rot_type)
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

vector<Tentabot::OccupancyVoxel> Tentabot::voxel_extractor(vector<geometry_msgs::Point>& archy)
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

void Tentabot::arc_extender_voxel_extractor(vector<geometry_msgs::Point>& planar_tentacle_data, 
                                            double yaw_sample, 
                                            vector<double> ang_samples, 
                                            string rot_type, 
                                            double yaw_offset)
{
  for(int i = 0; i < ang_samples.size(); i++)
  {
    vector <geometry_msgs::Point> newt;
    for(int j = 0; j < planar_tentacle_data.size(); j++)
    {
      newt.push_back( rotate3d( rotate3d(planar_tentacle_data[j], ang_samples[i], rot_type), yaw_offset, "yaw" ) );
    }
    robot_param.tentacle_data.push_back(newt);
    robot_param.support_vox_data.push_back(voxel_extractor(newt));
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

void Tentabot::construct_tentacle_pitchExt()
{
  clearTentacleData();
  clearSupportVoxData();

  vector<geometry_msgs::Point> planar_tentacle_data;

  vector<double> yaw_samples;
  if(off_tuning_param.tyaw_cnt > 1)
  {
    int half_tyaw_cnt = 0.5 * (off_tuning_param.tyaw_cnt + 1);
    double half_yaw_angle = 0.5 * off_tuning_param.tyaw;

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

  vector<double> pitch_samples;
  if(off_tuning_param.tpitch_cnt > 1)
  {
    int half_tpitch_cnt = 0.5 * (off_tuning_param.tpitch_cnt + 1);
    double half_pitch_angle = 0.5 * off_tuning_param.tpitch;

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

  double radi;
  double delta_len = off_tuning_param.tlen / off_tuning_param.tsamp_cnt;
  double yaw_offset = -0.5 * PI;

  // TENTACLE 0 (Contains single sampling point at the center of the robot)
  vector <geometry_msgs::Point> newt;
  geometry_msgs::Point tp;
  tp.x = 0.0;
  tp.y = 0.0;
  tp.z = 0.0;
  newt.push_back(tp);
  robot_param.tentacle_data.push_back(newt);
  robot_param.support_vox_data.push_back(voxel_extractor(newt));
  robot_param.right_left_data.push_back("M");

  // ALL OTHER TENTACLES
  for(int y = 0; y < yaw_samples.size(); y++)
  {
    if(yaw_samples[y] == 0)
    {
      for(int p = 0; p < off_tuning_param.tsamp_cnt; p++)
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
      if(off_tuning_param.tentacle_type == "circular")
      {
        radi = off_tuning_param.tlen / (2 * abs(yaw_samples[y]));
        planar_tentacle_data = arc_by_radi_ang(arc_center, radi, 2 * yaw_samples[y], off_tuning_param.tsamp_cnt);
      }
      else if(off_tuning_param.tentacle_type == "linear")
      {
        planar_tentacle_data = line_by_len_ang(arc_center, off_tuning_param.tlen, yaw_samples[y], off_tuning_param.tsamp_cnt);
      }
    }

    // EXTEND TENTACLES ALONG PITCH ANGLE AND EXTRACT SUPPORT/PRIORITY CELLS
    arc_extender_voxel_extractor(planar_tentacle_data, yaw_samples[y], pitch_samples, "pitch", yaw_offset);
  }
}

void Tentabot::construct_tentacle_extend(bool no_restriction)
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

void Tentabot::fillRobotVisu()
{
  visu_param.robot_visu.ns = robot_param.robot_name;
  visu_param.robot_visu.action = visualization_msgs::Marker::ADD;
  visu_param.robot_visu.pose.position.x = 0;
  visu_param.robot_visu.pose.position.y = 0;
  visu_param.robot_visu.pose.position.z = 0;
  visu_param.robot_visu.pose.orientation.x = 0;
  visu_param.robot_visu.pose.orientation.y = 0;
  visu_param.robot_visu.pose.orientation.z = 0;
  visu_param.robot_visu.pose.orientation.w = 1;
  visu_param.robot_visu.id = 888;
  visu_param.robot_visu.type = visualization_msgs::Marker::SPHERE;
  visu_param.robot_visu.scale.x = robot_param.width;
  visu_param.robot_visu.scale.y = robot_param.length;
  visu_param.robot_visu.scale.z = robot_param.height;
  visu_param.robot_visu.color.r = 0.128;
  visu_param.robot_visu.color.g = 0.0;
  visu_param.robot_visu.color.b = 0.128;
  visu_param.robot_visu.color.a = 1.0;
  visu_param.robot_visu.header.frame_id = robot_param.robot_frame_name;
}

void Tentabot::publishRobot()
{
  if(process_param.visu_flag == true)
  {
    visu_param.robot_visu.header.seq++;
    visu_param.robot_visu.header.stamp = ros::Time(0);
    visu_param.robot_visu_pub.publish(visu_param.robot_visu);
  }
}

void Tentabot::fillTentacleTsampSupportvoxVisu()
{
  int tentacle_cnt = robot_param.tentacle_data.size();
  int tsamp_cnt;

  // VISUALIZE TENTACLES, SUPPORT/PRIORITY VOXELS
  for(int k = 0; k < tentacle_cnt; k++)
  {
    // SET TENTACLE VISUALIZATION SETTINGS
    visualization_msgs::Marker tentacle_line_strip;
    tentacle_line_strip.ns = "tentacle" + to_string(k);
    tentacle_line_strip.id = k;
    tentacle_line_strip.header.frame_id = robot_param.robot_frame_name;
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
    tentacle_tsamp.header.frame_id = robot_param.robot_frame_name;
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

    tsamp_cnt = robot_param.tentacle_data[k].size();
    for(int p = 0; p < tsamp_cnt; p++)
    {
      if (tsamp_cnt > 1)
      {
        tentacle_line_strip.points.push_back(robot_param.tentacle_data[k][p]);
      }
      else
      {
        tentacle_line_strip.points.push_back(robot_param.tentacle_data[k][p]);
        tentacle_line_strip.points.push_back(robot_param.tentacle_data[k][p]);
      }

      tentacle_tsamp.points.push_back(robot_param.tentacle_data[k][p]);
    }
    visu_param.tentacle_visu.markers.push_back(tentacle_line_strip);
    visu_param.tsamp_visu.markers.push_back(tentacle_tsamp);

    // SET SUPPORT/PRIORITY VOXEL VISUALIZATION SETTINGS
    visualization_msgs::Marker support_vox_points;
    support_vox_points.ns = "support_voxel" + to_string(k);
    support_vox_points.id = k;
    support_vox_points.header.frame_id = robot_param.robot_frame_name;
    support_vox_points.type = visualization_msgs::Marker::CUBE_LIST;
    support_vox_points.action = visualization_msgs::Marker::ADD;
    support_vox_points.pose.orientation.w = 1.0;
    support_vox_points.scale.x = off_tuning_param.egrid_vdim;
    support_vox_points.scale.y = off_tuning_param.egrid_vdim;
    support_vox_points.scale.z = off_tuning_param.egrid_vdim;

    for(int s = 0; s < robot_param.support_vox_data[k].size(); s++)
    {
      geometry_msgs::Point po;
      toPoint(robot_param.support_vox_data[k][s].index, po);

      std_msgs::ColorRGBA sv_point_color;
      support_vox_points.points.push_back(po);

      sv_point_color.g = 0;
      sv_point_color.a = 0.05;
      if(robot_param.support_vox_data[k][s].flag)
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
    visu_param.support_vox_visu.markers.push_back(support_vox_points);
  }
}

void Tentabot::publishTentacleTsampSupportvox()
{
  if(process_param.visu_flag == true)
  {
    int tentacle_cnt = robot_param.tentacle_data.size();

    for(int k = 0; k < tentacle_cnt; k++)
    {
      // UPDATE SEQUENCE AND STAMP FOR TENTACLES
      visu_param.tentacle_visu.markers[k].header.seq++;
      visu_param.tentacle_visu.markers[k].header.stamp = ros::Time::now();

      // UPDATE SEQUENCE AND STAMP FOR SUPPORT CELLS
      visu_param.support_vox_visu.markers[k].header.seq++;
      visu_param.support_vox_visu.markers[k].header.stamp = ros::Time::now();

      // UPDATE SEQUENCE AND STAMP FOR SAMPLING POINTS ON TENTACLES 
      visu_param.tsamp_visu.markers[k].header.seq++;
      visu_param.tsamp_visu.markers[k].header.stamp = ros::Time::now();
    }

    visu_param.tentacle_visu_pub.publish(visu_param.tentacle_visu);
    visu_param.tsamp_visu_pub.publish(visu_param.tsamp_visu);
    visu_param.support_vox_visu_pub.publish(visu_param.support_vox_visu);
  }
}

void Tentabot::publishOccupancy()
{
  if(process_param.visu_flag == true)
  {
    visu_param.occupancy_pc.header.seq++;
    visu_param.occupancy_pc.header.stamp = ros::Time::now();
    visu_param.occupancy_pc_pub.publish(visu_param.occupancy_pc);
  }
}

void Tentabot::fillPathVisu()
{
  if(process_param.visu_flag == true)
  {
    visu_param.path_visu.ns = "path2glory";
    visu_param.path_visu.id = 753;
    visu_param.path_visu.type = visualization_msgs::Marker::SPHERE_LIST;
    visu_param.path_visu.action = visualization_msgs::Marker::ADD;
    visu_param.path_visu.pose.orientation.w = 1.0;
    visu_param.path_visu.scale.x = 0.2;
    visu_param.path_visu.scale.y = 0.2;
    visu_param.path_visu.scale.z = 0.2;
    visu_param.path_visu.color.r = 1.0;
    visu_param.path_visu.color.g = 0.1;
    visu_param.path_visu.color.b = 1.0;
    visu_param.path_visu.color.a = 1.0;
  }
}

void Tentabot::fillCommandVisu()
{
  if(process_param.visu_flag == true)
  {
    visu_param.command_visu.ns = "commandante";
    visu_param.command_visu.id = 1928;
    visu_param.command_visu.type = visualization_msgs::Marker::SPHERE_LIST;
    visu_param.command_visu.action = visualization_msgs::Marker::ADD;
    visu_param.command_visu.pose.orientation.w = 1.0;
    visu_param.command_visu.scale.x = 0.2;
    visu_param.command_visu.scale.y = 0.2;
    visu_param.command_visu.scale.z = 0.2;
    visu_param.command_visu.color.r = 0.05;
    visu_param.command_visu.color.g = 1.0;
    visu_param.command_visu.color.b = 1.0;
    visu_param.command_visu.color.a = 1.0;
  }
}

void Tentabot::publishPath()
{
  if(process_param.visu_flag == true)
  {
    visu_param.path_visu.header.frame_id = map_util.getFrameName();
    visu_param.path_visu.header.seq++;
    //this -> visu_param.path_visu.header.stamp = ros::Time(0);
    visu_param.path_visu.header.stamp = ros::Time::now();

    visu_param.path_visu_pub.publish(visu_param.path_visu);
  }
}

void Tentabot::publishCommand()
{
  if(process_param.visu_flag == true)
  {
    visu_param.command_visu.header.frame_id = map_util.getFrameName();
    visu_param.command_visu.header.seq++;
    //this -> visu_param.command_visu.header.stamp = ros::Time(0);
    visu_param.command_visu.header.stamp = ros::Time::now();

    visu_param.command_visu_pub.publish(visu_param.command_visu);
  }
}

geometry_msgs::Point Tentabot::interpol(geometry_msgs::Point p, double dist)
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

geometry_msgs::Point Tentabot::interpol(geometry_msgs::Point p1, 
                                        geometry_msgs::Point p2, 
                                        double dist_from_p1)
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

vector<geometry_msgs::Point> Tentabot::equadistant(geometry_msgs::Point p1, 
                                                   geometry_msgs::Point p2, 
                                                   int num_btw)
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