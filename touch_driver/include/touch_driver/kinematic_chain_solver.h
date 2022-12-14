/** kinematic_chain_solver.h
 * 
 * \brief Forward kinematics and Inverse kinematics solvers
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#ifndef KINEMATIC_CHAIN_SOLVER_H
#define KINEMATIC_CHAIN_SOLVER_H

#include <hardware_interface/joint_state_interface.h>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <geometry_msgs/Pose.h>
#include <kdl_conversions/kdl_msg.h>
// #include "kinematic_chain_base.h"
#include <touch_driver/kinematic_chain_base.h>

namespace touch_driver
{

class ForwardKinematicSolver : public KinematicChainBase
{
public:
    ForwardKinematicSolver() {}
    ~ForwardKinematicSolver() {}

    /** \brief The init function is called to initialize the controller from a
     * non-realtime thread with a pointer to the hardware interface, itself,
     * instead of a pointer to a RobotHW.
     *
     * \param robot The specific hardware interface used by this controller.
     *
     * \param n A NodeHandle in the namespace from which the controller
     * should read its configuration, and where it should set up its ROS
     * interface.
     *
     * \returns True if initialization was successful and the controller
     * is ready to be started.
     */
    bool init(ros::NodeHandle &nh);

    void publish();

protected:

    KDL::FrameVel x_dot_;
    KDL::Frame x_;

    boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
    boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;

    double publish_rate_;
    ros::Publisher pose_pub_;

};

} // namespace touch_driver

#endif // KINEMATIC_CHAIN_SOLVER_H