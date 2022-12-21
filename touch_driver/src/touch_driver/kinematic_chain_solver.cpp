/** kinematic_chain_solver.cpp
 * 
 * \brief Forward kinematics and Inverse kinematics solvers
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#include <touch_driver/kinematic_chain_solver.h>

namespace touch_driver
{
bool ForwardKinematicSolver::init(ros::NodeHandle &nh, jointstate* joint_state)
{
    if(!KinematicChainBase::init(nh)){
        ROS_ERROR("KinematicChain[INFO]: Failed to init Kinematic Base");
        return false;
    }

    joint_state_.reset(joint_state);
    
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    pose_pub_ .reset(new realtime_tools::RealtimePublisher<touch_msgs::TouchPoseTwist>(nh, "ee_state",10));
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

bool InverseKinematicSolver::init(ros::NodeHandle &nh, jointstate* joint_state)
{
    if(!ForwardKinematicSolver::init(nh, joint_state)){
        ROS_ERROR("KinematicSolver[ERROR]: Failed to init ForwardKinematicSolver");
        return false;
    }

    // ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    // ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, joint_limits_.min, joint_limits_.max, *fk_pos_solver_, *ik_vel_solver_));
    tracik_pos_solver_.reset(new TRAC_IK::TRAC_IK(kdl_chain_, joint_limits_.min, joint_limits_.max));

    // Topics
    command_sub_ = nh.subscribe("command_cart_pos", 10,
                            &InverseKinematicSolver::command_cart_pos,
                            this,ros::TransportHints().reliable().tcpNoDelay());
    command_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/effort_joint_traj_controller/command",10);

    x_des_.p.Zero();
    x_des_.M.Identity();
    q_cmd_.resize(this->kdl_chain_.getNrOfJoints());

    return true;
}

void InverseKinematicSolver::command_cart_pos(const geometry_msgs::PoseConstPtr &msg)
{
    x_des_.p(0) = msg->position.x;
    x_des_.p(1) = msg->position.y;
    x_des_.p(2) = msg->position.z;
    x_des_.M = KDL::Rotation::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    // ERROR: kdl ik solver return -1;
    // if(ik_pos_solver_->CartToJnt(this->joint_msr_.q, x_des_, q_cmd_)<0){
    //     ROS_INFO("KDL: error may occur in ik solver.");
    // }
    // else{
    //     ROS_INFO("KDL ik solver success");
    // }
    if(tracik_pos_solver_->CartToJnt(this->joint_msr_.q, x_des_, q_cmd_)<0){
        ROS_ERROR("KinematicSolver[ERROR]: error may occur in trac-ik solver.");
    }
    else{
        ROS_INFO("KinematicSolver[INFO]: trac-ik solver success");

        trajectory_msgs::JointTrajectory jnt_traj;
        trajectory_msgs::JointTrajectoryPoint jnt_traj_pt;

        jnt_traj.header.stamp = ros::Time::now();
        for(std::size_t i=0; i<joint_name_.size(); i++){
            jnt_traj.joint_names.push_back(joint_name_[i]);
        }
        for (std::size_t i=0; i<joint_name_.size(); i++){
            jnt_traj_pt.positions.push_back(q_cmd_(i));
        }
        jnt_traj_pt.time_from_start = ros::Duration(2.0);
        jnt_traj.points.push_back(jnt_traj_pt);
        command_pub_.publish(jnt_traj);
    }
}

void InverseKinematicSolver::initController()
{
    trajectory_msgs::JointTrajectory jnt_traj;
    trajectory_msgs::JointTrajectoryPoint jnt_traj_pt;

    jnt_traj.header.stamp = ros::Time::now();
    for(std::size_t i=0; i<joint_name_.size(); i++){
        jnt_traj.joint_names.push_back(joint_name_[i]);
    }
    for (std::size_t i=0; i<joint_name_.size(); i++){
        jnt_traj_pt.positions.push_back(joint_state_->joint_positions[i]);
    }
    jnt_traj_pt.time_from_start = ros::Duration(1.0);
    jnt_traj.points.push_back(jnt_traj_pt);
    command_pub_.publish(jnt_traj);
}
    
} // namespace touch_driver
