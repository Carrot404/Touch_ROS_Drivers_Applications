/** kinematic_chain_solver.h
 * 
 * \brief Forward kinematics and Inverse kinematics solvers
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#ifndef KINEMATIC_CHAIN_SOLVER_H
#define KINEMATIC_CHAIN_SOLVER_H

#include <ros/ros.h>
#include <vector>
#include <urdf/model.h>

// #include <hardware_interface/joint_state_interface.h>

#include <kdl/kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <geometry_msgs/Pose.h>
#include <kdl_conversions/kdl_msg.h>
// #include "kinematic_chain_base.h"
// #include <touch_driver/kinematic_chain_base.h>

namespace touch_driver
{
struct limits
{
    KDL::JntArray min;
    KDL::JntArray max;
    KDL::JntArray center;
};

struct jointstate
{
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;
};

class ForwardKinematicSolver
{
public:
    ForwardKinematicSolver();
    ~ForwardKinematicSolver() {}

    bool initbase(ros::NodeHandle &nh);
    bool init(ros::NodeHandle &nh);

    std::shared_ptr<jointstate> getStateData(){return joint_state_;}

    void publish();

protected:

    ros::NodeHandle nh_;
    KDL::Chain kdl_chain_;
    limits joint_limits_;
    KDL::JntArrayVel joint_msr_;
    std::vector<std::string> joint_name_;

    std::shared_ptr<jointstate> joint_state_;


    KDL::FrameVel x_dot_;
    KDL::Frame x_;

    boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
    boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;

    double publish_rate_;
    ros::Publisher pose_pub_;

};

} // namespace touch_driver

#endif // KINEMATIC_CHAIN_SOLVER_H