/** controller_stopper.cpp
 * 
 * \brief controller stopper for 3D Systems Touch
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#include <touch_driver/controller_stopper.h>

ControllerStopper::ControllerStopper(const ros::NodeHandle& nh) : nh_(nh), priv_nh_("~"), button_action_prev_(false), controller_running_(false)
{
  // Subscribes to a robot's running state topic. Ideally this topic is latched and only publishes
  // on changes. However, this node only reacts on state changes, so a state published each cycle
  // would also be fine.
  controller_running_sub_ = nh_.subscribe("button", 1, &ControllerStopper::buttonCallback, this);

  // Controller manager service to switch controllers
  controller_manager_srv_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/"
                                                                                         "switch_controller");
  // Controller manager service to list controllers
  controller_list_srv_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/"
                                                                                     "list_controllers");
  ROS_INFO_STREAM("controller_stopper[INFO]: Waiting for controller manager service to come up on " << nh_.resolveName("controller_manager/"
                                                                                             "switch_controller"));
  controller_manager_srv_.waitForExistence();
  ROS_INFO_STREAM("controller_stopper[INFO]: Service available.");
  ROS_INFO_STREAM("controller_stopper[INFO]: Waiting for controller list service to come up on " << nh_.resolveName("controller_manager/"
                                                                                          "list_controllers"));
  controller_list_srv_.waitForExistence();
  ROS_INFO_STREAM("controller_stopper[INFO]: Service available.");

  // Consistent controllers will not be stopped when the robot stops. Defaults to
  // ["joint_state_controller"]
  if (!priv_nh_.getParam("consistent_controllers", consistent_controllers_))
  {
    consistent_controllers_.push_back("joint_state_controller");
  }

  ROS_DEBUG("controller_stopper[INFO]: Waiting for running controllers");
  // Before we can work properly, we need to know which controllers there are
//   while (stopped_controllers_.empty())
//   {
//     findStoppableControllers();
//     ros::Duration(1).sleep();
//   }
  stopped_controllers_.push_back("effort_joint_traj_controller");

  ROS_DEBUG("controller_stopper[INFO]: Initialization finished");
}

void ControllerStopper::findStoppableControllers()
{
  controller_manager_msgs::ListControllers list_srv;
  controller_list_srv_.call(list_srv);
  stopped_controllers_.clear();
  for (auto& controller : list_srv.response.controller)
  {
    // Check if in consistent_controllers
    // Else:
    //   Add to stopped_controllers
    if (controller.state == "running")
    {
      auto it = std::find(consistent_controllers_.begin(), consistent_controllers_.end(), controller.name);
      if (it == consistent_controllers_.end())
      {
        stopped_controllers_.push_back(controller.name);
      }
    }
  }
}

void ControllerStopper::buttonCallback(const touch_msgs::TouchButtonEventConstPtr& msg)
{
//   button_action_prev_ = 
  if(msg->grey_button_action && !button_action_prev_){
    ROS_INFO_STREAM("controller_stopper[INFO]: Starting controllers");
    controller_manager_msgs::SwitchController srv;
    srv.request.strictness = srv.request.STRICT;
    srv.request.start_controllers = stopped_controllers_;
    if (!controller_manager_srv_.call(srv))
    {
      ROS_ERROR_STREAM("controller_stopper[ERROR]: Could not activate requested controllers");
    }
  }
  else if (!msg->grey_button_action && button_action_prev_){
    ROS_INFO_STREAM("controller_stopper[INFO]: Stopping controllers");
    controller_manager_msgs::SwitchController srv;
    srv.request.strictness = srv.request.STRICT;
    srv.request.stop_controllers = stopped_controllers_;
    if (!controller_manager_srv_.call(srv))
    {
      ROS_ERROR_STREAM("controller_stopper[ERROR]: Could not stop requested controllers");
    }
  }
  button_action_prev_ = msg->grey_button_action;
}