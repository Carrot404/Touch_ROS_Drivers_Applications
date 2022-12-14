/** kinematic_chain_solver.cpp
 * 
 * \brief Forward kinematics and Inverse kinematics solvers
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#include <touch_driver/kinematic_chain_solver.h>

namespace touch_driver
{

ForwardKinematicSolver::ForwardKinematicSolver()
{
    // this->joint_state_ = std::make_shared<jointstate>();

    // this->joint_state_->joint_names.resize(6);
    // this->joint_state_->joint_positions.resize(6);
    // this->joint_state_->joint_velocities.resize(6);
    // this->joint_state_->joint_efforts.resize(6);

}

bool ForwardKinematicSolver::init(ros::NodeHandle &nh, jointstate* joint_state)
{
    if(!KinematicChainBase::init(nh)){
        ROS_ERROR("Failed to init Kinematic Base");
        return false;
    }

    joint_state_.reset(joint_state);
    
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    pose_pub_ .reset(new realtime_tools::RealtimePublisher<touch_msgs::TouchPoseTwist>(nh, "pose_fk",100));
    pose_pub_->msg_.header.frame_id = root_name_;
    pose_pub_->msg_.child_frame_id = tip_name_;

    x_.p.Zero();
    x_.M.Identity();
    x_dot_.p.Zero();
    x_dot_.M.Identity();

    return true;
}

void ForwardKinematicSolver::publish(const ros::Time& timestamp)
{
    if (pose_pub_)
    {
        // Get joint positions
        for(std::size_t i=0; i < joint_name_.size(); i++)
        {
            joint_msr_.q(i)         = joint_state_->joint_positions[i];
            joint_msr_.qdot(i)      = joint_state_->joint_velocities[i];
        }
        // Compute forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_.q, x_);
        fk_vel_solver_->JntToCart(joint_msr_, x_dot_);

        if (pose_pub_->trylock())
        {
            pose_pub_->msg_.header.stamp = timestamp;
            tf::poseKDLToMsg(x_, pose_pub_->msg_.pose);
            tf::twistKDLToMsg(x_dot_.GetTwist(), pose_pub_->msg_.twist);
            pose_pub_->unlockAndPublish();
        }
    }
}

} // namespace touch_driver
