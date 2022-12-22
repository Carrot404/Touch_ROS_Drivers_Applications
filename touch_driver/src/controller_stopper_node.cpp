/** controller_stopper_node.cpp
 * 
 * \brief controller stopper for 3D Systems Touch
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#include <ros/ros.h>

#include <touch_driver/controller_stopper.h>

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "controller_stopper_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("");

  ControllerStopper stopper(nh);

  ros::spin();
  
  return 0;
}