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
#include <touch_msgs/TouchButtonEvent.h>

class ControllerStopper
{
public:
  ControllerStopper() = delete;
  ControllerStopper(const ros::NodeHandle& nh);
  virtual ~ControllerStopper() = default;

private:
  void buttonCallback(const touch_msgs::TouchButtonEventConstPtr& msg);

  /*!
   * \brief Queries running stoppable controllers.
   *
   * Queries the controller manager for running controllers and compares the result with the
   * consistent_controllers_. The remaining running controllers are stored in stopped_controllers_
   */
  void findStoppableControllers();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  ros::Subscriber controller_running_sub_;
  ros::ServiceClient controller_manager_srv_;
  ros::ServiceClient controller_list_srv_;

  std::vector<std::string> consistent_controllers_;
  std::vector<std::string> stopped_controllers_;

  bool button_action_prev_;
  bool controller_running_;
};





#endif  // CONTROLLER_STOPPER_H