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
#include <trac_ik/trac_ik.hpp>

#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <touch_msgs/TouchPoseTwist.h>
#include <touch_driver/kinematic_chain_base.h>



namespace touch_driver
{

class ForwardKinematicSolver : public KinematicChainBase
{
public:
    ForwardKinematicSolver() = default;
    ~ForwardKinematicSolver() = default;

    bool init(ros::NodeHandle &nh, jointstate* joint_state);

    void publish(const ros::Time& timestamp);

protected:

    std::shared_ptr<jointstate> joint_state_;

    KDL::FrameVel x_dot_;
    KDL::Frame x_;

    std::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;

    std::unique_ptr<realtime_tools::RealtimePublisher<touch_msgs::TouchPoseTwist>> pose_pub_;

};

class InverseKinematicSolver : public ForwardKinematicSolver
{
public:
    InverseKinematicSolver() = default;
    ~InverseKinematicSolver() = default;

    bool init(ros::NodeHandle &nh, jointstate* joint_state);

    // touch's chain has 3 joints. It works fine as long as the position term is fine. 
    void command_cart_pos(const geometry_msgs::PoseConstPtr &msg);

    // void initController();

protected:

    // boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
    // boost::shared_ptr<KDL::ChainIkSolverPos> ik_pos_solver_;  
    boost::shared_ptr<TRAC_IK::TRAC_IK> tracik_pos_solver_;

    ros::Subscriber command_sub_;
    ros::Publisher command_pub_;

    KDL::Frame x_des_;                          // Desired end-effector pose
    KDL::JntArray q_cmd_;                       // Desired joint position
    // trajectory_msgs::JointTrajectory jnt_traj_; // Desired joint trajectory

};

} // namespace touch_driver

#endif // KINEMATIC_CHAIN_SOLVER_H