// LAST UPDATE: 2022.02.23
//
// AUTHOR: Ronja Gueldenring
//         Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] This is modifed from (https://raw.githubusercontent.com/RGring/drl_local_planner_ros_stable_baselines/master/rl_local_planner/include/rl_local_planner/image_generator.h)
//
// TODO:

#include <laser_to_image.h>

int main(int argc, char** argv)
{    
  ros::init(argc, argv, "laser_to_image");
  ros::NodeHandle nh;

  string node_name = ros::this_node::getName();
  string laser_scan_msg;
  nh.getParam(node_name + "/laser_scan_msg", laser_scan_msg);
  cout << "laser_to_image::ImageGenerator -> laser_scan_msg: " << laser_scan_msg << endl;

  ImageGenerator ig(nh);

  // Subscriptions
  ros::Subscriber sub_laser_scan = nh.subscribe(laser_scan_msg, 100, &ImageGenerator::laser_scan_callback_, &ig);
  
  ros::spin();

  /*
  ros::WallRate r(100);
  while (ros::ok()) 
  {
    
    r.sleep();
  }
  */
  return 0;
};