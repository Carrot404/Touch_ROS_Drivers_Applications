/** kinematic_chain_solver.cpp
 * 
 * \brief Forward kinematics and Inverse kinematics solvers
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

// #include "../../include/touch_driver/kinematic_chain_solver.h"
#include <touch_driver/kinematic_chain_solver.h>

namespace touch_driver
{

bool ForwardKinematicSolver::init(
    hardware_interface::JointStateInterface *robot, ros::NodeHandle &nh)
{
    KinematicChainBase<hardware_interface::JointStateInterface>::
        init(robot, nh);
    
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    pose_pub_ = nh.advertise<geometry_msgs::Pose>("pose_test",10);

    // get publishing period
    if (!nh.getParam("publish_rate", publish_rate_)){
        ROS_ERROR("Parameter 'publish_rate' not set");
        return false;
    }

    x_.p.Zero();
    x_.M.Identity();
    x_dot_.p.Zero();
    x_dot_.M.Identity();

}

void ForwardKinematicSolver::publish()
{
    // ros::Rate loop_rate(publish_rate_);
    geometry_msgs::Pose msg;
    // while (ros::ok()){
        // Get joint positions
        for(std::size_t i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_.q(i)         = joint_handles_[i].getPosition();
            joint_msr_.qdot(i)      = joint_handles_[i].getVelocity();
        }

        // Compute forward kinematics
        fk_vel_solver_->JntToCart(joint_msr_, x_dot_);
        fk_pos_solver_->JntToCart(joint_msr_.q, x_);

        tf::poseKDLToMsg(x_, msg);
        pose_pub_.publish(msg);
        
    //     loop_rate.sleep();
    // }
}

} // namespace touch_driver
