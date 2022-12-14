/** kinematic_chain_solver.cpp
 * 
 * \brief Forward kinematics and Inverse kinematics solvers
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

// #include "../../include/touch_driver/kinematic_chain_solver.h"
#include <touch_driver/kinematic_chain_solver.h>

namespace touch_driver
{

ForwardKinematicSolver::ForwardKinematicSolver()
{
    this->joint_state_ = std::make_shared<jointstate>();

    this->joint_state_->joint_names.resize(6);
    this->joint_state_->joint_positions.resize(6);
    this->joint_state_->joint_velocities.resize(6);
    this->joint_state_->joint_efforts.resize(6);

}

bool ForwardKinematicSolver::init(ros::NodeHandle &nh)
{
    initbase(nh);
    
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
        for(std::size_t i=0; i < joint_name_.size(); i++)
        {
            joint_msr_.q(i)         = joint_state_->joint_positions[i];
            joint_msr_.qdot(i)      = joint_state_->joint_positions[i];
        }

        // Compute forward kinematics
        fk_vel_solver_->JntToCart(joint_msr_, x_dot_);
        fk_pos_solver_->JntToCart(joint_msr_.q, x_);

        tf::poseKDLToMsg(x_, msg);
        pose_pub_.publish(msg);
        
    //     loop_rate.sleep();
    // }
}

bool ForwardKinematicSolver::initbase(ros::NodeHandle &nh)
{
    nh_ = nh;
    // priv_nh_ = priv_nh;
    // joint_state_ = std::make_shared<jointstate>(joint_state);
    // std::shared_ptr<jointstate> joint_state_(joint_state);

    // get URDF and name of root and tip from the parameter server
    std::string robot_description, root_name, tip_name;

    std::string name_space = nh_.getNamespace();
    std::cout<< "--------------------> name_space:  " << name_space << std::endl;

    if (!ros::param::search(name_space,"robot_description", robot_description))
    {
        ROS_ERROR_STREAM("KinematicChain: No robot description (URDF)"
                        "found on parameter server (" << nh_.getNamespace() <<
                        "/robot_description)");
        return false;
    }

    if (!nh_.getParam( name_space + "/root_name", root_name))
    {
        ROS_ERROR_STREAM("KinematicChain: No root name found on "
                        "parameter server ("<<nh_.getNamespace()<<"/root_name)");
        return false;
    }

    if (!nh_.getParam(name_space + "/tip_name", tip_name))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No tip name found on "
                        "parameter server ("<<nh_.getNamespace()<<"/tip_name)");
        return false;
    }

    // Construct an URDF model from the xml string
    std::string xml_string;

    if (nh_.hasParam(robot_description))
        nh_.getParam(robot_description.c_str(), xml_string);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...",
                robot_description.c_str());
        nh_.shutdown();
        return false;
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",
                robot_description.c_str());
        nh_.shutdown();
        return false;
    }

    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        nh_.shutdown();
        return false;
    }
    ROS_INFO("Successfully parsed urdf file");

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        nh_.shutdown();
        return false;
    }

    // Populate the KDL chain
    if(!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
        ROS_ERROR_STREAM( "    "<<(*it).first);

        return false;
    }

    ROS_INFO("tip_name:  %s",tip_name.c_str());
    ROS_INFO("root_name: %s",root_name.c_str());
    ROS_INFO("Number of segments: %d", kdl_chain_.getNrOfSegments());
    ROS_INFO("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
    for(std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++){
        ROS_INFO_STREAM("segment("<<i<<"): " << kdl_chain_.getSegment(i).getName());
    }

    // Parsing joint limits from urdf model along kdl chain
    std::shared_ptr<const urdf::Link> link = model.getLink(tip_name);
    std::shared_ptr<const urdf::Joint> joint;
    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());

    int index;
    for (std::size_t i = 0; i < kdl_chain_.getNrOfJoints() && link; i++)
    {
        joint = model.getJoint(link->parent_joint->name);
        ROS_INFO("Getting limits for joint: %s", joint->name.c_str());
        index = kdl_chain_.getNrOfJoints() - i - 1;

        if(joint->limits){
        joint_limits_.min(index) = joint->limits->lower;
        joint_limits_.max(index) = joint->limits->upper;
        joint_limits_.center(index) = (joint_limits_.min(index) +
                                        joint_limits_.max(index))/2;
        }else{
        joint_limits_.min(index) = 0;
        joint_limits_.max(index) = 0;
        joint_limits_.center(index) = 0;
        ROS_INFO("joint->limits is NULL %s",joint->name.c_str());
        }

        link = model.getLink(link->getParent()->name);
    }

    ROS_INFO("Getting joint in kdl chain");
    int count = 0;
    for(std::vector<KDL::Segment>::const_iterator it =
        kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
    {
        ROS_INFO("%s type: %s", it->getJoint().getName().c_str(),
                it->getJoint().getTypeName().c_str() );
        if(it->getJoint().getTypeName() != "None" && count < 6) {
            // joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
            joint_name_.push_back(it->getJoint().getName());
        }
        count++;
    }
    joint_msr_.resize(joint_name_.size());
    ROS_INFO("Number of joints = %lu", joint_name_.size() );
    ROS_INFO_STREAM("kdl_chain.getNrOfJoints: " << kdl_chain_.getNrOfJoints());


    // ROS_INFO("Getting joint handles");
    // // Get joint handles for all of the joints in the chain
    // int count=0;
    // for(std::vector<KDL::Segment>::const_iterator it =
    //     kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
    // {

    //     ROS_INFO("%s type: %s", it->getJoint().getName().c_str(),
    //             it->getJoint().getTypeName().c_str() );
    //     if(it->getJoint().getTypeName() != "None" && count < 7) {
    //         // joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
    //     }
    //     count++;
    // }

    // ROS_INFO("Number of joints in handle = %lu", joint_handles_.size() );
    // ROS_INFO_STREAM("kdl_chain.getNrOfJoints: " << kdl_chain_.getNrOfJoints());

    ROS_INFO("Finished Kinematic Base init");

    return true;
}

} // namespace touch_driver
