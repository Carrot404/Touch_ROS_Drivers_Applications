/** touch_tele.cpp
 * 
 * 
 * 
 * 
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <touch_msgs/TouchButtonEvent.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/shared_ptr.hpp>

class TouchTele
{
private:
    // ROS Parameters
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    
    ros::Subscriber poseref_sub_;
    ros::Subscriber button_sub_;

    std::string touch_name_;

	boost::shared_ptr<tf::TransformListener> tf_listener_;


public:
    // Constructor and destructor
    TouchTele(ros::NodeHandle nh);
    ~TouchTele();

    // Topic Callback function
    void buttonCallback(const touch_msgs::TouchButtonEvent::ConstPtr &msg);
    void poserefCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

};

TouchTele::TouchTele(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
{
    // ROS Parameters
    priv_nh_.param<std::string>("touch_namespace", touch_name_, "Touch");
    
    // Subsribe to NameSpace/button
    std::string button_topic = touch_name_+"/button";
    button_sub_ = nh_.subscribe(button_topic, 5, &TouchTele::buttonCallback, this);

    // Subscribe to NameSpace/pose_ref
    std::string poseref_topic = touch_name_+"/pose_ref";
    poseref_sub_ = nh_.subscribe(poseref_topic, 5, &TouchTele::poserefCallback, this);

    tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
    try{
        tf_listener_->waitForTransform("base", "stylus", ros::Time(0), ros::Duration(2));
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("tf listener: transform exception : %s",ex.what());
    }
    
    ros::AsyncSpinner spinner(2);
    spinner.start();
    

}

TouchTele::~TouchTele()
{

}

void TouchTele::buttonCallback(const touch_msgs::TouchButtonEvent::ConstPtr &msg)
{

}

void TouchTele::poserefCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // transform from base_ref to base without translation.
    geometry_msgs::Point position;
    position.x = -msg->pose.position.x;
    position.y = msg->pose.position.z;
    position.z = msg->pose.position.y;

    // transform from {base_ref}Qua{sty_ref} to {base}Qua{probe}
    // {base}Qua{probe} = {base}R{base_ref}*{base_ref}Qua{sty_ref}*{sty_ref}R{probe}
    tf::Quaternion ref_R_probe(tf::Vector3(1,0,0), 3.1415927);

    /* tf::StampedTransform transform_tf;
    try{
        tf_listener_->waitForTransform("base", "stylus", ros::Time(0), ros::Duration(0.2));
        tf_listener_->lookupTransform("base", "stylus", ros::Time(0), transform_tf);
        tf_listener_->transformVector("base")
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("tf listener: transform exception : %s",ex.what());
    } */


}

int main(int argc, char** argv)
{
    ////////////////////////////////////////////////////////////////
    // Init ROS
    ////////////////////////////////////////////////////////////////
    ros::init(argc, argv, "Touch_Tele_Node");
    ros::NodeHandle nh;
    TouchTele touch_tele(nh);

    // Test for Quaternion operation
    /* tf::Quaternion ref_R_probe(tf::Vector3(1,0,0), 3.1415927);
    tf::Matrix3x3 matrix(0,0,-1,0,1,0,1,0,0);
    tf::Quaternion qua1;
    matrix.getRotation(qua1);

    tf::Quaternion qua2;
    qua2 = qua1*ref_R_probe;
    tf::Matrix3x3 matrix1(qua2);

    std::cout<<ref_R_probe.getX()<<" "<<ref_R_probe.getY()<<" "<<ref_R_probe.getZ()<<" "<<ref_R_probe.getW()<<std::endl;
    std::cout<<matrix1[0][0]<<" "<<matrix1[0][1]<<" "<<matrix1[0][2]<<std::endl;
    std::cout<<matrix1[1][0]<<" "<<matrix1[1][1]<<" "<<matrix1[1][2]<<std::endl;
    std::cout<<matrix1[2][0]<<" "<<matrix1[2][1]<<" "<<matrix1[2][2]<<std::endl; */


    





    return 0;
}
