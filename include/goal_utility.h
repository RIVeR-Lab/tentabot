#ifndef GOAL_UTILITY_H
#define GOAL_UTILITY_H

// LAST UPDATE: 2022.03.23
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

// --EXTERNAL LIBRARIES--
#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

// --NAMESPACES--
using namespace std;
using namespace ros;

// DESCRIPTION: TODO...
class GoalUtility
{
  public:

    // DESCRIPTION: TODO...
    GoalUtility();

    // DESCRIPTION: TODO...
    GoalUtility(NodeHandle& nh, string new_frame_name, string new_goal_name, string new_goal_msg="/goal");

    // DESCRIPTION: TODO...
    GoalUtility(NodeHandle& nh, string new_frame_name, vector<geometry_msgs::Pose> new_goal, string new_goal_name, string new_goal_msg="/goal");

    // DESCRIPTION: TODO...
    GoalUtility(const GoalUtility& gu);

    // DESCRIPTION: TODO...
    ~GoalUtility();

    // DESCRIPTION: TODO...
    GoalUtility& operator = (const GoalUtility& gu);

    // DESCRIPTION: TODO...
    string getFrameName();

    // DESCRIPTION: TODO...
    string getGoalMsg();

    // DESCRIPTION: TODO...
    string getGoalName();

    // DESCRIPTION: TODO...
    vector<geometry_msgs::Pose> getGoal();

    // DESCRIPTION: TODO...
    int getActiveGoalIndex();

    // DESCRIPTION: TODO...
    geometry_msgs::Pose getActiveGoal();

    // DESCRIPTION: TODO...
    visualization_msgs::MarkerArray getGoalVisu();

    // DESCRIPTION: TODO...
    ros::Publisher getGoalVisuPub();

    // DESCRIPTION: TODO...
    void setFrameName(string new_frame_name);

    // DESCRIPTION: TODO...
    void setGoalMsg(string new_goal_msg);

    // DESCRIPTION: TODO...
    void setGoalName(string new_goal_name);

    // DESCRIPTION: TODO...
    void clearGoal();

    // DESCRIPTION: TODO...
    void setGoal(vector<geometry_msgs::Pose> new_goal);

    // DESCRIPTION: TODO...
    void setGoalPoint(double gpx, double gpy, double gpz);

    // DESCRIPTION: TODO...
    void setGoalPoint(geometry_msgs::Pose gp);

    // DESCRIPTION: TODO...
    void setGoalPoint(geometry_msgs::Point gp);

    // DESCRIPTION: TODO...
    void addGoalPoint(double gpx, double gpy, double gpz);

    // DESCRIPTION: TODO...
    void setRandomGoal(vector<double> x_range, vector<double> y_range, vector<double> z_range, int num, bool duplicate_check=false);

    // DESCRIPTION: TODO...
    void setActiveGoalIndex(int new_active_goal_index);

    // DESCRIPTION: TODO...
    bool switchActiveGoal();

    // DESCRIPTION: TODO...
    void clearGoalVisu();

    // DESCRIPTION: TODO...
    void setGoalVisu(visualization_msgs::MarkerArray new_goal_visu);

    // DESCRIPTION: TODO...
    void setGoalVisuPub(ros::Publisher new_goal_visu_pub);

    // DESCRIPTION: TODO...
    void fillGoalVisu();

    // DESCRIPTION: TODO...
    void addGoalVisu();

    // DESCRIPTION: TODO...
    void publishGoal();

    // DESCRIPTION: TODO...
    void printGoal();

    // DESCRIPTION: TODO...
    void goalCallback(const geometry_msgs::PoseStamped msg);

  private:
    
    string goal_name;
    string frame_name;
    string goal_msg;
    vector<geometry_msgs::Pose> goal;
    int active_goal_index;
    visualization_msgs::MarkerArray goal_visu;
    ros::Publisher goal_visu_pub;
    ros::Subscriber sub_goal;

    // DESCRIPTION: TODO...
    double randdouble(double from, double to);

    // DESCRIPTION: TODO...
    bool isVecContainPoint(vector<geometry_msgs::Pose> vec, geometry_msgs::Point p);

    // DESCRIPTION: TODO...
    bool isVecContainPoint(vector<geometry_msgs::Pose> vec, geometry_msgs::Pose p);

}; //END of class GoalUtility

#endif