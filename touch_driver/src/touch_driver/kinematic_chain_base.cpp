/** kinematic_chain_base.cpp
 * 
 * \brief KDL base tree/chain/frames
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

// #include "../../include/touch_driver/kinematic_chain_base.h"
#include <touch_driver/kinematic_chain_base.h>

namespace touch_driver
{
bool KinematicChainBase::init(ros::NodeHandle &nh)
{
    nh_ = nh;
    // priv_nh_ = priv_nh;

    // get URDF and name of root and tip from the parameter server
    std::string robot_description;

    std::string name_space = nh_.getNamespace();
    std::cout<< "--------------------> name_space:  " << name_space << std::endl;

    if (!ros::param::search(name_space,"robot_description", robot_description))
    {
        ROS_ERROR_STREAM("KinematicChain: No robot description (URDF)"
                        "found on parameter server (" << nh_.getNamespace() <<
                        "/robot_description)");
        return false;
    }

    if (!nh_.getParam( name_space + "/root_name", root_name_))
    {
        ROS_ERROR_STREAM("KinematicChain: No root name found on "
                        "parameter server ("<<nh_.getNamespace()<<"/root_name)");
        return false;
    }

    if (!nh_.getParam(name_space + "/tip_name", tip_name_))
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
    if(!kdl_tree.getChain(root_name_, tip_name_, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name_<<" --> "<<tip_name_);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
        ROS_ERROR_STREAM( "    "<<(*it).first);

        return false;
    }

    ROS_INFO("tip_name:  %s",tip_name_.c_str());
    ROS_INFO("root_name: %s",root_name_.c_str());
    ROS_INFO("Number of segments: %d", kdl_chain_.getNrOfSegments());
    ROS_INFO("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
    for(std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++){
        ROS_INFO_STREAM("segment("<<i<<"): " << kdl_chain_.getSegment(i).getName());
    }

    // Parsing joint limits from urdf model along kdl chain
    std::shared_ptr<const urdf::Link> link = model.getLink(tip_name_);
    std::shared_ptr<const urdf::Joint> joint;
    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());

    int index;
    for (int i = 0; i < kdl_chain_.getNrOfJoints() && link; i++)
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
            i--;
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
