/** kinematic_chain_base.h
 * 
 * \brief KDL base tree/chain/frames
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#ifndef KINEMATIC_CHAIN_BASE_H
#define KINEMATIC_CHAIN_BASE_H

#include <ros/ros.h>
#include <vector>
#include <urdf/model.h>

#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>

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

class KinematicChainBase
{
public:
    KinematicChainBase() {}
    ~KinematicChainBase() {};

    bool init(ros::NodeHandle &nh);

protected:
    ros::NodeHandle nh_;
    // ros::NodeHandle priv_nh_;
    KDL::Chain kdl_chain_;
    limits joint_limits_;
    KDL::JntArrayVel joint_msr_;
    std::vector<std::string> joint_name_;

    // std::vector<typename JointInterface::ResourceHandleType> joint_handles_;
};

} // namespace touch_driver

#endif // KINEMATIC_CHAIN_BASE_H