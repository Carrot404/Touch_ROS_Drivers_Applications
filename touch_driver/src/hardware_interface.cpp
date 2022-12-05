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

    robot_hw_nh.param<std::string>("tf_prefix", tf_prefix_, "");

    

}


double HardwareInterface::getControlFrequency()
{
  if (geo_proxy_ != nullptr)
  {
    return geo_proxy_->getControlFrequency();
  }
  throw std::runtime_error("GeomagicProxy is not initialized");
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




} // namespace touch_driver

PLUGINLIB_EXPORT_CLASS(touch_driver::HardwareInterface, hardware_interface::RobotHW)