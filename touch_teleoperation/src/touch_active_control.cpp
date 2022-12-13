/** touch_active_control.cpp
 * 
 * \brief active control Node for Touch Applications
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "touch_active_control");
    ros::NodeHandle nh;

    std::string base_frame, target_frame, urdf_param;
    
    nh.param("base_frame", base_frame, std::string(""));
    
}

