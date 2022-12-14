/** hardware_interface.cpp
 * 
 * \brief hardware_interface for 3D Systems Touch
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#include <pluginlib/class_list_macros.hpp>
#include <touch_driver/hardware_interface.h>

namespace touch_driver
{

HardwareInterface::HardwareInterface()
    // : joint_names_(6)
    // , joint_positions_(6)
    // , joint_velocities_(6)
    // , joint_efforts_(6)
    : joint_effort_command_(3)
    , tcp_pose_(7)
    , button_state_(6)
    , effort_controller_running_(false)
    , robot_program_running_(true)
    , controller_reset_necessary_(false)
    , controllers_initialized_(false)
{
  joint_state_ = std::make_shared<jointstate>();
  joint_state_->joint_names.resize(6);
  joint_state_->joint_positions.resize(6);
  joint_state_->joint_velocities.resize(6);
  joint_state_->joint_efforts.resize(6);
}

HardwareInterface::~HardwareInterface()
{
  geo_proxy_->stop();
  geo_proxy_.reset();
  fksolver_.reset();
}

bool HardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  // ROS Parameters
  // device_name is not used actually. Device name coule be modified in "GeomagicProxy.h"
  // std::string device_name = robot_hw_nh.param<std::string>("device_name", "carrot");

  // Topic namespace
  // std::string touch_namespace = robot_hw_nh.param<std::string>("touch_namespace", "Touch");

  // expected publisg rate 
  publish_rate_ = robot_hw_nh.param<int>("publish_rate", 500);

  // When the robot's URDF is being loaded with a prefix, we need to know it here, as well, in order
  // to publish correct frame names for frames reported by the robot directly.
  robot_hw_nh.param<std::string>("tf_prefix", tf_prefix_, "Touch");
    
  // Names of the joints. Usually, this is given in the controller config file.
  // if (!robot_hw_nh.getParam("joints", joint_names_))
  if (!robot_hw_nh.getParam("joints", joint_state_->joint_names))
  {
    ROS_ERROR_STREAM("Cannot find required parameter " << robot_hw_nh.resolveName("joints")
                                                      << " on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                            "'controller_joint_names' on the parameter server.");
  }

  // Create ros_control interfaces
  for (std::size_t i = 0; i < joint_state_->joint_names.size(); i++)
  {
    ROS_DEBUG_STREAM("Registering handles for joint " << joint_state_->joint_names[i]);
    // Create joint state interface for all joints
    jnt_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_state_->joint_names[i], &joint_state_->joint_positions[i],
                                                                             &joint_state_->joint_velocities[i], &joint_state_->joint_efforts[i]));

    // Create joint effort control interface
    if(i<3){
      jnt_effort_interface_.registerHandle(
          hardware_interface::JointHandle(jnt_state_interface_.getHandle(joint_state_->joint_names[i]), &joint_effort_command_[i]));
    }

  }

  // Register interfaces
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_effort_interface_);

  controllers_initialized_ = true;
  effort_controller_running_ = true;

  // Publish pose to /tf
  tcp_pose_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(root_nh, "/tf", 100));
  tcp_transform_.header.frame_id = "base_ref";
  tcp_transform_.child_frame_id = "stylus_controller";

  // Publish button state to /Touch/button
  std::string button_name = tf_prefix_ + "/button";
  button_pub_.reset(new realtime_tools::RealtimePublisher<touch_msgs::TouchButtonEvent>(root_nh, button_name.c_str(), 10));

  // Initialize Geomagic Proxy
  ROS_INFO_STREAM("Initializing Geomagic Proxy");
  geo_proxy_ = std::make_shared<GeomagicProxy>();

  fksolver_ = std::make_shared<ForwardKinematicSolver>();
  if(!fksolver_->init(robot_hw_nh, joint_state_.get())){
    ROS_ERROR("Failed to init ForwardKinematicSolver");
    return false;
  } 

  return true;
}

void HardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  std::shared_ptr<GeomagicStatus> geo_state = geo_proxy_->getDataPackage();
  if(geo_state)
  {
    // publish to Topic /joint_states
    for (std::size_t i = 0; i < joint_state_->joint_names.size(); i++)
    {
      joint_state_->joint_positions[i] = geo_state->jointPosition[i];
      joint_state_->joint_velocities[i] = geo_state->jointVelocity[i];
      if (i<3){
        joint_state_->joint_efforts[i] = geo_state->jointEffort[i];
      }
    }

    // publish to Topic /tf
    for (std::size_t i = 0; i < tcp_pose_.size(); i++)
    {
      tcp_pose_[i] = geo_state->cartPose[i];
    }
    extractToolPose(time);
    publishPose();

    // Publish to Topic /Touch/button
    button_state_[0] = geo_state->buttons[0];
    button_state_[1] = geo_state->buttons[1];
    button_state_[2] = geo_state->buttons_prev[0];
    button_state_[3] = geo_state->buttons_prev[1];
    button_state_[4] = geo_state->action[0];
    button_state_[5] = geo_state->action[1];
    publishButton();

    fksolver_->publish(time);
  }
}

void HardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if (robot_program_running_)
  {
    if (effort_controller_running_)
    {
      geo_proxy_->setJointForceMode();
      // geo_proxy_->setNoMode();
      geo_proxy_->setForceCommand(joint_effort_command_);
    }
  }
  
}

bool HardwareInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                      const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  bool ret_val = true;
  if (controllers_initialized_ && !isRobotProgramRunning() && !start_list.empty())
  {
    for (auto& controller : start_list)
    {
      if (!controller.claimed_resources.empty())
      {
        ROS_ERROR_STREAM("Robot control is currently inactive. Starting controllers that claim resources is currently "
                         "not possible. Not starting controller '"
                         << controller.name << "'");
        ret_val = false;
      }
    }
  }

  controllers_initialized_ = true;
  return ret_val;
}

void HardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                 const std::list<hardware_interface::ControllerInfo>& stop_list)
{

  for (auto& controller_it : stop_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
      if (checkControllerClaims(resource_it.resources))
      {
        if (resource_it.hardware_interface == "hardware_interface::EffortJointInterface")
        {
          effort_controller_running_ = false;
        }
      }
    }
  }
  for (auto& controller_it : start_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
      if (checkControllerClaims(resource_it.resources))
      {
        if (resource_it.hardware_interface == "hardware_interface::EffortJointInterface")
        {
          effort_controller_running_ = true;
        }
      }
    }
  }
}

bool HardwareInterface::isRobotProgramRunning() const // TODO:
{
  return robot_program_running_;
}

bool HardwareInterface::shouldResetControllers()
{
  if (controller_reset_necessary_)
  {
    controller_reset_necessary_ = false;
    return true;
  }
  else
  {
    return false;
  }
}

void HardwareInterface::extractToolPose(const ros::Time& timestamp)
{
  // double tcp_angle = std::sqrt(std::pow(tcp_pose_[3], 2) + std::pow(tcp_pose_[4], 2) + std::pow(tcp_pose_[5], 2));

  // tf2::Vector3 rotation_vec(tcp_pose_[3], tcp_pose_[4], tcp_pose_[5]);
  // tf2::Quaternion rotation;
  // if (tcp_angle > 1e-16)
  // {
  //   rotation.setRotation(rotation_vec.normalized(), tcp_angle);
  // }
  // else
  // {
  //   rotation.setValue(0.0, 0.0, 0.0, 1.0);  // default Quaternion is 0,0,0,0 which is invalid
  // }
  tcp_transform_.header.stamp = timestamp;
  tcp_transform_.transform.translation.x = - tcp_pose_[0];
  tcp_transform_.transform.translation.y = tcp_pose_[2];
  tcp_transform_.transform.translation.z = tcp_pose_[1];
  tcp_transform_.transform.rotation.x = tcp_pose_[3];
  tcp_transform_.transform.rotation.y = tcp_pose_[4];
  tcp_transform_.transform.rotation.z = tcp_pose_[5];
  tcp_transform_.transform.rotation.w = tcp_pose_[6];
}

void HardwareInterface::publishPose()
{
  if (tcp_pose_pub_)
  {
    if (tcp_pose_pub_->trylock())
    {
      tcp_pose_pub_->msg_.transforms.clear();
      tcp_pose_pub_->msg_.transforms.push_back(tcp_transform_);
      tcp_pose_pub_->unlockAndPublish();
    }
  }
}

void HardwareInterface::publishButton()
{
  if (button_pub_)
  {
    if (button_pub_->trylock())
    {
      button_pub_->msg_.grey_button = button_state_[0];
      button_pub_->msg_.white_button = button_state_[1];
      button_pub_->msg_.grey_button_action = button_state_[4];
      button_pub_->msg_.white_button_action = button_state_[5];
      button_pub_->unlockAndPublish();
    }
  }
}

bool HardwareInterface::checkControllerClaims(const std::set<std::string>& claimed_resources)
{
  for (const std::string& it : joint_state_->joint_names)
  {
    for (const std::string& jt : claimed_resources)
    {
      if (it == jt)
      {
        return true;
      }
    }
  }
  return false;
}




} // namespace touch_driver

PLUGINLIB_EXPORT_CLASS(touch_driver::HardwareInterface, hardware_interface::RobotHW)