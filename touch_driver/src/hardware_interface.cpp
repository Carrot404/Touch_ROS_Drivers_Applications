/** hardware_interface.cpp
 * 
 * \brief hardware_interface for 3D Systems Touch
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#include <pluginlib/class_list_macros.hpp>
// #include <touch_driver/hardware_interface.h>
#include "../include/hardware_interface.h"

namespace touch_driver
{

HardwareInterface::HardwareInterface()
    : joint_names_(6)
    , joint_positions_(6)
    , joint_velocities_(6)
    , joint_efforts_(6)
    , joint_effort_command_(6)
    , joint_position_command_(6)
    , effort_controller_running_(false)
    , velocity_controller_running_(false)
    , position_controller_running_(false)
    , robot_program_running_(true)
    , controller_reset_necessary_(false)
    , controllers_initialized_(false)
{
}

bool HardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  // ROS Parameters
  // device_name is not used actually. Device name coule be modified in "GeomagicProxy.h"
  std::string device_name = robot_hw_nh.param<std::string>("device_name", "carrot");
  // Topic namespace
  std::string touch_namespace = robot_hw_nh.param<std::string>("touch_namespace", "Touch");
  // expected publisg rate 
  int publish_rate = robot_hw_nh.param<int>("publish_rate", 500);
  // When the robot's URDF is being loaded with a prefix, we need to know it here, as well, in order
  // to publish correct frame names for frames reported by the robot directly.
  robot_hw_nh.param<std::string>("tf_prefix", tf_prefix_, "");
    
  // Names of the joints. Usually, this is given in the controller config file.
  if (!robot_hw_nh.getParam("joints", joint_names_))
  {
    ROS_ERROR_STREAM("Cannot find required parameter " << robot_hw_nh.resolveName("joints")
                                                      << " on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                            "'controller_joint_names' on the parameter server.");
  }

  // Create ros_control interfaces
  for (std::size_t i = 0; i < joint_names_.size(); i++)
  {
    ROS_DEBUG_STREAM("Registering handles for joint " << joint_names_[i]);
    // Create joint state interface for all joints
    jnt_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                             &joint_velocities_[i], &joint_efforts_[i]));

    // Create joint effort control interface
    jnt_effort_interface_.registerHandle(
        hardware_interface::JointHandle(jnt_state_interface_.getHandle(joint_names_[i]), &joint_effort_command_[i]));
    
    // Create joint position control interface
    jnt_position_interface_.registerHandle(
        hardware_interface::JointHandle(jnt_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));

  }

  // Register interfaces
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_effort_interface_);
  


  ROS_INFO_STREAM("Initializing Geomagic Proxy");
  geo_proxy_ = std::make_shared<GeomagicProxy>();

  // loop and update geoStatus
  // pthread_t state_thread;
  // pthread_create(&state_thread, NULL, state_update, (void*) geo_proxy_.get());

  return true;
}

// void *HardwareInterface::state_update(void *ptr)
// {
//   geo_proxy_->run();
//   return nullptr;
// }

void HardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  std::shared_ptr<GeomagicStatus> state = geo_proxy_->getDataPackage();
  for (std::size_t i = 0; i < joint_names_.size(); i++)
  {
    joint_positions_[i] = state->jointPosition[i];
    joint_velocities_[i] = state->jointVelocity[i];
    // joint_efforts_[i] = state->j
  }

}

void HardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  std::shared_ptr<GeomagicStatus> state = geo_proxy_->getDataPackage();
  for (std::size_t i = 0; i < 3; i++)
  {
    state->force[i] = joint_effort_command_[i];
  }
  // state->force[0] = joint_effort_command_[0];
  
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
        if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
        {
          velocity_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
        {
          position_controller_running_ = false;
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
        if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
        {
          velocity_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
        {
          position_controller_running_ = true;
        }
      }
    }
  }
}

double HardwareInterface::getControlFrequency() //TODO:
{
  if (geo_proxy_ != nullptr)
  {
    return geo_proxy_->getControlFrequency();
  }
  // return geo_proxy_.getControlFrequency();
  throw std::runtime_error("GeomagicProxy is not initialized");
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

bool HardwareInterface::checkControllerClaims(const std::set<std::string>& claimed_resources)
{
  for (const std::string& it : joint_names_)
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