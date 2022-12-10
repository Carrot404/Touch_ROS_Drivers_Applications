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
#include <tf2_msgs/TFMessage.h>
#include <touch_msgs/TouchButtonEvent.h>

#include "./GeomagicProxy.h"

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
    virtual ~HardwareInterface();
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
    virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                const std::list<hardware_interface::ControllerInfo>& stop_list) override;
    /*!
    * \brief Starts and stops controllers.
    *
    * \param start_list List of controllers to start
    * \param stop_list List of controllers to stop
    */
    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                            const std::list<hardware_interface::ControllerInfo>& stop_list) override;

    /*!
    * \brief Getter for the current control frequency
    *
    * \returns The used control frequency
    */
    int getControlFrequency(){return publish_rate_;}

    /*!
    * \brief Checks if the URCaps program is running on the robot.
    *
    * \returns True, if the program is currently running, false otherwise.
    */
    bool isRobotProgramRunning() const;

    /*!
    * \brief Checks if a reset of the ROS controllers is necessary.
    *
    * \returns Necessity of ROS controller reset
    */
    bool shouldResetControllers();

    /*!
    * \brief obtain the pointer of Geomagic Proxy
    *
    * \returns pointer to the Geomagic Proxy
    */
    std::shared_ptr<GeomagicProxy> getptrGeoProxy(){return geo_proxy_;}

protected:

    /*!
    * \brief Checks whether a resource list contains joints from this hardware interface
    *
    * True is returned as soon as one joint name from claimed_resources matches a joint from this
    * hardware interface.
    */
    bool checkControllerClaims(const std::set<std::string>& claimed_resources);

    /*!
    * \brief Stores the raw tool pose data from the robot in a transformation msg
    *
    * \param timestamp Timestamp of read data
    */
    void extractToolPose(const ros::Time& timestamp);

    /*!
    * \brief Publishes the tool pose to the tf system
    *
    * Requires extractToolPose() to be run first.
    */
    void publishPose();

    /*!
    * \brief Publishes the button state
    */
    void publishButton();



    std::shared_ptr<GeomagicProxy> geo_proxy_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::EffortJointInterface jnt_effort_interface_;
    // hardware_interface::JointStateInterface pos_state_interface_;
    
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<double> joint_effort_command_;
    std::vector<double> tcp_pose_;                     //!< Pose vetor in order like this [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
    std::vector<bool> button_state_;
    
    geometry_msgs::TransformStamped tcp_transform_;
    std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> tcp_pose_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<touch_msgs::TouchButtonEvent>> button_pub_;



    bool effort_controller_running_;

    bool robot_program_running_;

    bool controller_reset_necessary_;
    bool controllers_initialized_;

    std::string tf_prefix_;
    int publish_rate_;

};

} // namespace touch_driver

#endif // TOUCH_DRIVER_HARDWARE_INTERFACE_H