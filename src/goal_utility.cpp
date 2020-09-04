// LAST UPDATE: 2019.08.17
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION:

// CUSTOM LIBRARIES:
#include "goal_utility.h"

GoalUtility::GoalUtility()
{
  this -> frame_name = "";
  this -> goal_name = "";
  this -> active_goal_index = -1;
}

GoalUtility::GoalUtility(NodeHandle nh, string new_frame_name, string new_goal_name)
{
  this -> frame_name = new_frame_name;
  this -> goal_name = new_goal_name;
  this -> active_goal_index = -1;
  this -> goal_visu_pub = nh.advertise<visualization_msgs::MarkerArray>(new_goal_name, 100);
}

GoalUtility::GoalUtility(NodeHandle nh, string new_frame_name, vector<geometry_msgs::Pose> new_goal, string new_goal_name)
{
  this -> frame_name = new_frame_name;
  this -> goal_name = new_goal_name;
  this -> setGoal(new_goal);
  this -> fillGoalVisu();
  this -> setActiveGoalIndex(0);
  this -> goal_visu_pub = nh.advertise<visualization_msgs::MarkerArray>(new_goal_name, 100);
}

GoalUtility::GoalUtility(const GoalUtility& gu)
{
  this -> frame_name = gu.frame_name;
  this -> goal_name = gu.goal_name;
  this -> setGoal(gu.goal);
  this -> fillGoalVisu();
  this -> setActiveGoalIndex(gu.active_goal_index);
  this -> goal_visu_pub = gu.goal_visu_pub;
}

GoalUtility::~GoalUtility()
{
  //ROS_INFO( "Calling Destructor for GoalUtility..." );
}

GoalUtility& GoalUtility::operator = (const GoalUtility& gu) 
{ 
  this -> frame_name = gu.frame_name;
  this -> goal_name = gu.goal_name;
  this -> setGoal(gu.goal);
  this -> fillGoalVisu();
  this -> setActiveGoalIndex(gu.active_goal_index);
  this -> goal_visu_pub = gu.goal_visu_pub;
} 

string GoalUtility::getFrameName()
{
  return this -> frame_name;
}

string GoalUtility::getGoalName()
{
  return this -> goal_name;
}

vector<geometry_msgs::Pose> GoalUtility::getGoal()
{
  return this -> goal;
}

int GoalUtility::getActiveGoalIndex()
{
  return this -> active_goal_index;
}

geometry_msgs::Pose GoalUtility::getActiveGoal()
{
  return this -> goal[this -> active_goal_index];
}

visualization_msgs::MarkerArray GoalUtility::getGoalVisu()
{
  return this -> goal_visu;
}

ros::Publisher GoalUtility::getGoalVisuPub()
{
  return this -> goal_visu_pub;
}

void GoalUtility::setFrameName(string new_frame_name)
{
  this -> frame_name = new_frame_name;
}

void GoalUtility::setGoalName(string new_goal_name)
{
  this -> goal_name = new_goal_name;
}

void GoalUtility::clearGoal()
{
  this -> goal.clear();
}

void GoalUtility::setGoal(vector<geometry_msgs::Pose> new_goal)
{
  this -> goal.clear();

  int goal_cnt = new_goal.size();
  for(int i = 0; i < goal_cnt; i++)
  {
    geometry_msgs::Pose goal_wp;
    goal_wp.position.x = new_goal[i].position.x;
    goal_wp.position.y = new_goal[i].position.y;
    goal_wp.position.z = new_goal[i].position.z;
    goal_wp.orientation.x = new_goal[i].orientation.x;
    goal_wp.orientation.y = new_goal[i].orientation.y;
    goal_wp.orientation.z = new_goal[i].orientation.z;
    goal_wp.orientation.w = new_goal[i].orientation.w;
    this -> goal.push_back(goal_wp);
  }
}

void GoalUtility::setGoalPoint(double gpx, double gpy, double gpz)
{
  this -> goal.clear();
      
  geometry_msgs::Pose goal_wp;
  goal_wp.position.x = gpx;
  goal_wp.position.y = gpy;
  goal_wp.position.z = gpz;
  goal_wp.orientation.x = 0;
  goal_wp.orientation.y = 0;
  goal_wp.orientation.z = 0;
  goal_wp.orientation.w = 1;

  this -> goal.push_back(goal_wp);
  this -> fillGoalVisu();
  this -> setActiveGoalIndex(0);
}

void GoalUtility::setGoalPoint(geometry_msgs::Pose gp)
{
  this -> goal.clear();

  geometry_msgs::Pose goal_wp;
  goal_wp.position.x = gp.position.x;
  goal_wp.position.y = gp.position.y;
  goal_wp.position.z = gp.position.z;
  goal_wp.orientation.x = gp.orientation.x;
  goal_wp.orientation.y = gp.orientation.y;
  goal_wp.orientation.z = gp.orientation.z;
  goal_wp.orientation.w = gp.orientation.w;

  this -> goal.push_back(goal_wp);

  this -> fillGoalVisu();
  this -> setActiveGoalIndex(0);
}

void GoalUtility::setGoalPoint(geometry_msgs::Point gp)
{
  this -> goal.clear();

  geometry_msgs::Pose goal_wp;
  goal_wp.position.x = gp.x;
  goal_wp.position.y = gp.y;
  goal_wp.position.z = gp.z;
  goal_wp.orientation.x = 0;
  goal_wp.orientation.y = 0;
  goal_wp.orientation.z = 0;
  goal_wp.orientation.w = 1;

  this -> goal.push_back(goal_wp);

  this -> fillGoalVisu();
  this -> setActiveGoalIndex(0);
}

void GoalUtility::addGoalPoint(double gpx, double gpy, double gpz)
{  
  geometry_msgs::Pose goal_wp;
  goal_wp.position.x = gpx;
  goal_wp.position.y = gpy;
  goal_wp.position.z = gpz;
  goal_wp.orientation.x = 0;
  goal_wp.orientation.y = 0;
  goal_wp.orientation.z = 0;
  goal_wp.orientation.w = 1;

  this -> goal.push_back(goal_wp);

  this -> addGoalVisu();

  if(this -> goal.size() == 1)
  {
    this -> setActiveGoalIndex(0);
  }
}

void GoalUtility::setRandomGoal(vector<double> x_range, vector<double> y_range, vector<double> z_range, int num)
{
  this -> goal.clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rgp;
    rgp.position.x = randdouble(x_range[0], x_range[1]);
    rgp.position.y = randdouble(y_range[0], y_range[1]);
    rgp.position.z = randdouble(z_range[0], z_range[1]);
    rgp.orientation.x = 0;
    rgp.orientation.y = 0;
    rgp.orientation.z = 0;
    rgp.orientation.w = 1;

    while(isVecContainPoint(this -> goal, rgp))                       // check for duplicates
    {
      rgp.position.x = randdouble(x_range[0], x_range[1]);
      rgp.position.y = randdouble(y_range[0], y_range[1]);
      rgp.position.z = randdouble(z_range[0], z_range[1]);
    }

    this -> goal.push_back(rgp);
  }

  this -> fillGoalVisu();
  this -> setActiveGoalIndex(0);
}

void GoalUtility::setActiveGoalIndex(int new_active_goal_index)
{
  if( new_active_goal_index >= 0 && new_active_goal_index < this -> goal.size() )
  {
    this -> active_goal_index = new_active_goal_index;

    this -> goal_visu.markers[new_active_goal_index].color.r = 0;
    this -> goal_visu.markers[new_active_goal_index].color.g = 0;
    this -> goal_visu.markers[new_active_goal_index].color.b = 1;
  }
}

bool GoalUtility::switchActiveGoal()
{
  if(this -> active_goal_index == this -> goal.size()-1)
  {
    this -> goal_visu.markers[this -> active_goal_index].color.r = 1;
    this -> goal_visu.markers[this -> active_goal_index].color.g = 0;
    this -> goal_visu.markers[this -> active_goal_index].color.b = 0.2;
    
    return false;
  }
  else if(this -> active_goal_index < this -> goal.size()-1)
  {
    this -> goal_visu.markers[this -> active_goal_index].color.r = 1;
    this -> goal_visu.markers[this -> active_goal_index].color.g = 0;
    this -> goal_visu.markers[this -> active_goal_index].color.b = 0.2;

    this -> setActiveGoalIndex( this -> active_goal_index + 1 );

    this -> goal_visu.markers[this -> active_goal_index].color.r = 0;
    this -> goal_visu.markers[this -> active_goal_index].color.g = 0;
    this -> goal_visu.markers[this -> active_goal_index].color.b = 1;

    return true;
  }
  return false;
}

void GoalUtility::setGoalVisu(visualization_msgs::MarkerArray new_goal_visu)
{
  this -> goal_visu.markers.clear();

  int marker_size = new_goal_visu.markers.size();

  for(int i = 0; i < marker_size; i++)
  {
    visualization_msgs::Marker new_goal_marker;
    
    new_goal_marker.header.seq = new_goal_visu.markers[i].header.seq;
    new_goal_marker.header.stamp = new_goal_visu.markers[i].header.stamp;
    new_goal_marker.header.frame_id = new_goal_visu.markers[i].header.frame_id;
    new_goal_marker.ns = new_goal_visu.markers[i].ns;
    new_goal_marker.id = new_goal_visu.markers[i].id;
    new_goal_marker.type = new_goal_visu.markers[i].type;
    new_goal_marker.action = new_goal_visu.markers[i].action;
    new_goal_marker.pose.position.x = new_goal_visu.markers[i].pose.position.x;
    new_goal_marker.pose.position.y = new_goal_visu.markers[i].pose.position.y;
    new_goal_marker.pose.position.z = new_goal_visu.markers[i].pose.position.z;
    new_goal_marker.pose.orientation.x = new_goal_visu.markers[i].pose.orientation.x;
    new_goal_marker.pose.orientation.y = new_goal_visu.markers[i].pose.orientation.y;
    new_goal_marker.pose.orientation.z = new_goal_visu.markers[i].pose.orientation.z;
    new_goal_marker.pose.orientation.w = new_goal_visu.markers[i].pose.orientation.w;
    new_goal_marker.scale = new_goal_visu.markers[i].scale;
    new_goal_marker.color = new_goal_visu.markers[i].color;
    new_goal_marker.lifetime = new_goal_visu.markers[i].lifetime;
    new_goal_marker.frame_locked = new_goal_visu.markers[i].frame_locked;
    new_goal_marker.points = new_goal_visu.markers[i].points;
    new_goal_marker.colors = new_goal_visu.markers[i].colors;
    new_goal_marker.text = new_goal_visu.markers[i].text;
    new_goal_marker.mesh_resource = new_goal_visu.markers[i].mesh_resource;
    new_goal_marker.mesh_use_embedded_materials = new_goal_visu.markers[i].mesh_use_embedded_materials;

    this -> goal_visu.markers.push_back(new_goal_marker);
  }
}

void GoalUtility::setGoalVisuPub(ros::Publisher new_goal_visu_pub)
{
  this -> goal_visu_pub = new_goal_visu_pub;
}

void GoalUtility::clearGoalVisu()
{
  this -> goal_visu.markers.clear();
}

void GoalUtility::fillGoalVisu()
{
  this -> goal_visu.markers.clear();

  int goal_size = this -> goal.size();

  for(int i = 0; i < goal_size; i++)
  {
    visualization_msgs::Marker goal_wp_visu;
        
    goal_wp_visu.ns = this -> goal_name + "_wp" + to_string(i+1);
    goal_wp_visu.id = i+1;
    goal_wp_visu.action = visualization_msgs::Marker::ADD;
    goal_wp_visu.type = visualization_msgs::Marker::CYLINDER;
    goal_wp_visu.pose.position.x = this -> goal[i].position.x;
    goal_wp_visu.pose.position.y = this -> goal[i].position.y;
    goal_wp_visu.pose.position.z = this -> goal[i].position.z;
    goal_wp_visu.pose.orientation.x = this -> goal[i].orientation.x;
    goal_wp_visu.pose.orientation.y = this -> goal[i].orientation.y;
    goal_wp_visu.pose.orientation.z = this -> goal[i].orientation.z;
    goal_wp_visu.pose.orientation.w = this -> goal[i].orientation.w;
    goal_wp_visu.scale.x = 0.5;
    goal_wp_visu.scale.y = 0.5;
    goal_wp_visu.scale.z = 0.5;
    goal_wp_visu.color.r = 0.5;
    goal_wp_visu.color.g = 0.5;
    goal_wp_visu.color.b = 0.5;
    goal_wp_visu.color.a = 0.5;
    goal_wp_visu.header.frame_id = this -> frame_name;

    this -> goal_visu.markers.push_back(goal_wp_visu);
  }
}

void GoalUtility::addGoalVisu()
{
  int i = this -> goal_visu.markers.size();

  visualization_msgs::Marker goal_wp_visu;
        
  goal_wp_visu.ns = this -> goal_name + "_wp" + to_string(i+1);
  goal_wp_visu.id = i+1;
  goal_wp_visu.action = visualization_msgs::Marker::ADD;
  goal_wp_visu.type = visualization_msgs::Marker::CYLINDER;
  goal_wp_visu.pose.position.x = this -> goal[i].position.x;
  goal_wp_visu.pose.position.y = this -> goal[i].position.y;
  goal_wp_visu.pose.position.z = this -> goal[i].position.z;

  goal_wp_visu.pose.orientation.x = this -> goal[i].orientation.x;
  goal_wp_visu.pose.orientation.y = this -> goal[i].orientation.y;
  goal_wp_visu.pose.orientation.z = this -> goal[i].orientation.z;
  goal_wp_visu.pose.orientation.w = this -> goal[i].orientation.w;
  goal_wp_visu.scale.x = 0.5;
  goal_wp_visu.scale.y = 0.5;
  goal_wp_visu.scale.z = 0.5;
  goal_wp_visu.color.r = 0.5;
  goal_wp_visu.color.g = 0.5;
  goal_wp_visu.color.b = 0.5;
  goal_wp_visu.color.a = 0.5;
  goal_wp_visu.header.frame_id = this -> frame_name;

  this -> goal_visu.markers.push_back(goal_wp_visu);
}

void GoalUtility::publishGoal()
{
  int goal_visu_size = this -> goal_visu.markers.size();

  for(int i = 0; i < goal_visu_size; i++)
  {
    this -> goal_visu.markers[i].header.seq++;
    this -> goal_visu.markers[i].header.stamp = ros::Time(0);
  }
      
  this -> goal_visu_pub.publish(this -> goal_visu);
}

void GoalUtility::printGoal()
{
  int goal_size = this -> goal.size();

  for(int i = 0; i < goal_size; i++)
  {
    cout << this -> goal_name << ": wp " << (i+1) << ": (" << this -> goal[i].position.x << ", " << this -> goal[i].position.y << ", " << this -> goal[i].position.z << ")" << endl;
  }
}

double GoalUtility::randdouble(double from, double to)
{
  double f = (double)rand() / RAND_MAX;
  return from + f * (to - from);   
}

bool GoalUtility::isVecContainPoint(vector<geometry_msgs::Pose> vec, geometry_msgs::Point p)
{
  int vec_size = vec.size();
  
  for(int i = 0; i < vec_size; i++)
  {
    if(vec[i].position.x == p.x && vec[i].position.y == p.y && vec[i].position.z == p.z)
    {
      return true;
    }
  }

  return false;
}

bool GoalUtility::isVecContainPoint(vector<geometry_msgs::Pose> vec, geometry_msgs::Pose p)
{
  int vec_size = vec.size();
  
  for(int i = 0; i < vec_size; i++)
  {
    if(vec[i].position.x == p.position.x && vec[i].position.y == p.position.y && vec[i].position.z == p.position.z)
    {
      return true;
    }
  }

  return false;
}