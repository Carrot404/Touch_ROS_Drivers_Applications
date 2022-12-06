/** hardware_interface.h
 * 
 * \brief hardware_interface for 3D Systems Touch
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#ifndef TOUCH_DRIVER_HARDWARE_INTERFACE_H
#define TOUCH_DRIVER_HARDWARE_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <realtime_tools/realtime_publisher.h>

// #include <touch_driver/GeomagicProxy.h>
#include "GeomagicProxy.h"

// void *state_update(void *ptr)
// {
//     GeomagicProxy *touchProxy = (GeomagicProxy *) ptr;
//     // std::shared_ptr<GeomagicProxy> touchProxy((GeomagicProxy*) ptr);
//     // touchProxy.reset(std::shared_ptr<void> ptr);
//     // touchProxy.get()
//     touchProxy->run();
//     return nullptr;
// }

namespace touch_driver
{
/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class HardwareInterface : public hardware_interface::RobotHW
{
public:
    /*!
    * \brief Creates a new HardwareInterface object.
    */
    HardwareInterface();
    virtual ~HardwareInterface() = default;
    /*!
    * \brief Handles the setup functionality for the ROS interface. This includes parsing ROS
    * parameters, creating interfaces, starting the main driver and advertising ROS services.
    *
    * \param root_nh Root level ROS node handle
    * \param robot_hw_nh ROS node handle for the robot namespace
    *
    * \returns True, if the setup was performed successfully
    */
    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
    /*!
    * \brief Read method of the control loop. Reads Touch state from HD api and
    * publishes the information as needed.
    *
    * \param time Current time
    * \param period Duration of current control loop iteration
    */
    virtual void read(const ros::Time& time, const ros::Duration& period) override;
    /*!
    * \brief Write method of the control loop. Writes target joint positions to the robot
    *
    * \param time Current time
    * \param period Duration of current control loop iteration
    */
    virtual void write(const ros::Time& time, const ros::Duration& period) override;
    /*!
    * \brief Preparation to start and stop loaded controllers.
    *
    * \param start_list List of controllers to start
    * \param stop_list List of controllers to stop
    *
    * \returns True, if the controllers can be switched
    */
    // virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
    //                             const std::list<hardware_interface::ControllerInfo>& stop_list) override;
    /*!
    * \brief Starts and stops controllers.
    *
    * \param start_list List of controllers to start
    * \param stop_list List of controllers to stop
    */
    // virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
    //                         const std::list<hardware_interface::ControllerInfo>& stop_list) override;

    /*!
    * \brief Getter for the current control frequency
    *
    * \returns The used control frequency
    */
    double getControlFrequency();

    /*!
    * \brief Checks if a reset of the ROS controllers is necessary.
    *
    * \returns Necessity of ROS controller reset
    */
    bool shouldResetControllers();

    std::shared_ptr<GeomagicProxy> geo_proxy_;

protected:

    // TODO: shared_ptr
    // std::shared_ptr<GeomagicProxy> geo_proxy_;
    // GeomagicProxy *geo_proxy_;
    // GeomagicProxy geo_proxy_;


    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::EffortJointInterface jnt_effort_interface_;
    
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<double> joint_effort_command_;




    bool controller_reset_necessary_;
    bool controllers_initialized_;

    std::string tf_prefix_;

};















} // namespace touch_driver

#endif // TOUCH_DRIVER_HARDWARE_INTERFACE_H