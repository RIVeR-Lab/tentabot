// LAST UPDATE: 2019.08.17
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION:

// OUTSOURCE LIBRARIES:
#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>

// NAMESPACES:
using namespace std;
using namespace ros;

class GoalUtility
{
  private:
    string goal_name;
    vector<geometry_msgs::Pose> goal;
    int active_goal_index;
    visualization_msgs::MarkerArray goal_visu;
    ros::Publisher goal_visu_pub;
    string frame_name;

    double randdouble(double from, double to);

    bool isVecContainPoint(vector<geometry_msgs::Pose> vec, geometry_msgs::Point p);

    bool isVecContainPoint(vector<geometry_msgs::Pose> vec, geometry_msgs::Pose p);

  public:
    GoalUtility();

    GoalUtility(NodeHandle nh, string new_frame_name, string new_goal_name);

    GoalUtility(NodeHandle nh, string new_frame_name, vector<geometry_msgs::Pose> new_goal, string new_goal_name);

    GoalUtility(const GoalUtility& gu);

    ~GoalUtility();

    GoalUtility& operator = (const GoalUtility &gu);

    string getFrameName();

    string getGoalName();

    vector<geometry_msgs::Pose> getGoal();

    int getActiveGoalIndex();

    geometry_msgs::Pose getActiveGoal();

    visualization_msgs::MarkerArray getGoalVisu();

    ros::Publisher getGoalVisuPub();

    void setFrameName(string new_frame_name);

    void setGoalName(string new_goal_name);

    void clearGoal();

    void setGoal(vector<geometry_msgs::Pose> new_goal);

    void setGoalPoint(double gpx, double gpy, double gpz);

    void setGoalPoint(geometry_msgs::Pose gp);

    void setGoalPoint(geometry_msgs::Point gp);

    void addGoalPoint(double gpx, double gpy, double gpz);

    void setRandomGoal(vector<double> x_range, vector<double> y_range, vector<double> z_range, int num);

    void setActiveGoalIndex(int new_active_goal_index);

    bool switchActiveGoal();

    void clearGoalVisu();

    void setGoalVisu(visualization_msgs::MarkerArray new_goal_visu);

    void setGoalVisuPub(ros::Publisher new_goal_visu_pub);

    void fillGoalVisu();

    void addGoalVisu();

    void publishGoal();

    void printGoal();
};