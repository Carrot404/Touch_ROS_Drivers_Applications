/** kinematic_chain_solver.h
 * 
 * \brief Forward kinematics and Inverse kinematics solvers
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#ifndef KINEMATIC_CHAIN_SOLVER_H
#define KINEMATIC_CHAIN_SOLVER_H

#include <realtime_tools/realtime_publisher.h>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <touch_msgs/TouchPoseTwist.h>
#include <touch_driver/kinematic_chain_base.h>

// #include <trac_ik/trac_ik.hpp>

namespace touch_driver
{

class ForwardKinematicSolver : public KinematicChainBase
{
public:
    ForwardKinematicSolver();
    ~ForwardKinematicSolver() {}

    bool init(ros::NodeHandle &nh, jointstate* joint_state);

    void publish(const ros::Time& timestamp);

protected:

    std::shared_ptr<jointstate> joint_state_;

    KDL::FrameVel x_dot_;
    KDL::Frame x_;

    boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
    boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;

    // double publish_rate_;
    std::unique_ptr<realtime_tools::RealtimePublisher<touch_msgs::TouchPoseTwist>> pose_pub_;

    // ros::Publisher pose_pub_;

};

} // namespace touch_driver

#endif // KINEMATIC_CHAIN_SOLVER_H