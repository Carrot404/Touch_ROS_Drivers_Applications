/** controller_stopper.cpp
 * 
 * \brief controller stopper for 3D Systems Touch
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#include <touch_driver/controller_stopper.h>

ControllerStopper::ControllerStopper(const ros::NodeHandle& nh) : nh_(nh), priv_nh_("~"), button_action_prev_(false), controller_running_(false)
{
  // Subscribes to button topic. Ideally this topic is latched and only publishes on changes.
  // However, this node only reacts on state changes, so a state published each cycle would also be fine.
  button_sub_ = nh_.subscribe("button", 10, &ControllerStopper::buttonCallback, this);
  
  // jnt_traj_sub_ = nh_.subscribe("command_cart_pos", 10, &ControllerStopper::positionCallback, this);
  // jnt_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/effort_joint_traj_controller/command",10);

  compute_ik_srv_ = nh_.serviceClient<touch_msgs::TouchIK>("/touch_hardware_interface/compute_ik");
  compute_ik_srv_.waitForExistence();

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

// void ControllerStopper::positionCallback(const geometry_msgs::PointConstPtr& msg)
// {
//   touch_msgs::TouchIK srv;
//   srv.request.position.x = msg->x;
//   srv.request.position.y = msg->y;
//   srv.request.position.z = msg->z;
//   if(compute_ik_srv_.call(srv)){
//     ROS_INFO_STREAM("controller_stopper[INFO]: succeed to call compute_ik service.");

//     trajectory_msgs::JointTrajectory jnt_traj;
//     trajectory_msgs::JointTrajectoryPoint jnt_traj_pt;

//     jnt_traj.header.stamp = ros::Time::now();
//     jnt_traj.joint_names.push_back("joint1_waist");
//     jnt_traj.joint_names.push_back("joint2_shoulder");
//     jnt_traj.joint_names.push_back("joint3_elbow");
//     jnt_traj_pt.positions.push_back(srv.response.positions[0]);
//     jnt_traj_pt.positions.push_back(srv.response.positions[1]);
//     jnt_traj_pt.positions.push_back(srv.response.positions[2]);
//     jnt_traj_pt.time_from_start = ros::Duration(2.0);
//     jnt_traj.points.push_back(jnt_traj_pt);
//     jnt_traj_pub_.publish(jnt_traj);
//   }
//   else{
//     ROS_ERROR_STREAM("controller_stopper[ERROR]: failed to call compute_ik service.");
//   }
// }
