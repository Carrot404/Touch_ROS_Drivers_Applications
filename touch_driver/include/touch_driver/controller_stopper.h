/** controller_stopper.h
 * 
 * \brief controller stopper for 3D Systems Touch
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#ifndef CONTROLLER_STOPPER_H
#define CONTROLLER_STOPPER_H

#include <ros/ros.h>

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Point.h>
#include <touch_msgs/TouchButtonEvent.h>
#include <touch_msgs/TouchIK.h>

class ControllerStopper
{
public:
  ControllerStopper() = delete;
  ControllerStopper(const ros::NodeHandle& nh);
  virtual ~ControllerStopper() = default;

private:

  void buttonCallback(const touch_msgs::TouchButtonEventConstPtr& msg);

  void positionCallback(const geometry_msgs::PointConstPtr& msg);

  void findStoppableControllers();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Subscriber button_sub_;
  // ros::Subscriber jnt_traj_sub_;
  // ros::Publisher jnt_traj_pub_;

  ros::ServiceClient compute_ik_srv_;

  ros::ServiceClient controller_manager_srv_;
  ros::ServiceClient controller_list_srv_;

  std::vector<std::string> consistent_controllers_;
  std::vector<std::string> stopped_controllers_;

  bool button_action_prev_;
  bool controller_running_;
};





#endif  // CONTROLLER_STOPPER_H