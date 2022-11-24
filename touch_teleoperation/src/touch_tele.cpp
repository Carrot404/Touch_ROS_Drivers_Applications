/** touch_tele.cpp
 * 
 * \brief Node for 3D Systems Touch to control Universal Robots
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <touch_msgs/TouchButtonEvent.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


#include <boost/shared_ptr.hpp>

class TouchTele
{
private:
    // ROS Parameters
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    
    ros::Subscriber button_sub_;
    ros::Subscriber poseref_sub_;
    ros::Subscriber twist_sub_;


    std::string touch_name_;

    std::vector<double> position_;
    std::vector<double> position_prev_;

    std::vector<double> velocity_;
    std::vector<double> velocity_prev_;

    int count_;
    bool teleIsActive_;

    ros::AsyncSpinner *spinner;
	// boost::shared_ptr<tf::TransformListener> tf_listener_;
    boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;


public:
    std::vector<double> target_delta_;
    geometry_msgs::Quaternion target_orientation_;

    // Constructor and destructor
    TouchTele(ros::NodeHandle nh);
    ~TouchTele();

    // Topic Callback function
    void buttonCallback(const touch_msgs::TouchButtonEvent::ConstPtr &msg);
    void poserefCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // teleoperation control loop 
    void teleControl();
};

TouchTele::TouchTele(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
{

    // Init Move Group
    move_group_ = boost::shared_ptr<moveit::planning_interface::MoveGroupInterface>(new moveit::planning_interface::MoveGroupInterface("manipulator"));
    ROS_INFO("Planning frame: %s", move_group_->getPlanningFrame().c_str());
    ROS_INFO("Available Joint Names:");
    std::copy(move_group_->getJointNames().begin(), move_group_->getJointNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
    std::cout<<std::endl;
    ROS_INFO("End effector link: %s", move_group_->getEndEffectorLink().c_str());

    // ROS Parameters
    priv_nh_.param<std::string>("touch_namespace", touch_name_, "Touch");
    
    // Subsribe to NameSpace/button
    std::string button_topic = touch_name_+"/button";
    button_sub_ = nh_.subscribe(button_topic, 5, &TouchTele::buttonCallback, this);

    // Subscribe to NameSpace/pose_ref
    std::string poseref_topic = touch_name_+"/pose_ref";
    poseref_sub_ = nh_.subscribe(poseref_topic, 5, &TouchTele::poserefCallback, this);

    // Subscribe to NameSpace/twist
    std::string twist_topic = touch_name_+"/twist";
    twist_sub_ = nh_.subscribe(twist_topic, 5, &TouchTele::twistCallback, this);

    // init tf listener
    // tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
    // try{
    //     tf_listener_->waitForTransform("base", "stylus", ros::Time(0), ros::Duration(2));
    // }
    // catch(tf::TransformException &ex){
    //     ROS_ERROR("tf listener: transform exception : %s",ex.what());
    // }

    // init ROS Spinner
    spinner = new ros::AsyncSpinner(3);
    spinner->start();

    position_.resize(3);
    position_prev_.resize(3);
    target_delta_.resize(3);
    velocity_.resize(3);
    for (int i=0; i<3; i++){
        position_[i] = 0.0;
        position_prev_[i] = 0.0;
        target_delta_[i] = 0.0;
        velocity_[i] = 0.0;
    }
    count_ = 0;
    teleIsActive_ = false;

}

TouchTele::~TouchTele(){}

void TouchTele::buttonCallback(const touch_msgs::TouchButtonEvent::ConstPtr &msg)
{
    if (msg->white_button_action == 1){
        teleIsActive_ = true;
    }
    if (msg->white_button_action == 0){
        teleIsActive_ = false;
    }
}

void TouchTele::poserefCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // transform from base_ref to base without translation.
    position_[0] += -msg->pose.position.x;
    position_[1] += msg->pose.position.z;
    position_[2] += msg->pose.position.y;

    // transform from {base_ref}Qua{sty_ref} to {base}Qua{probe}
    // {base}Qua{probe} = {base}R{base_ref}*{base_ref}Qua{sty_ref}*{sty_ref}R{probe}
    tf::Quaternion sref_Q_probe(tf::Vector3(1,0,0), 3.1415927);
    tf::Matrix3x3 base_M_bref(-1,0,0,0,0,1,0,1,0);
    tf::Quaternion base_Q_bref;
    base_M_bref.getRotation(base_Q_bref);
    tf::Quaternion bref_Q_sref;
    tf::quaternionMsgToTF(msg->pose.orientation, bref_Q_sref);
    tf::Quaternion base_Q_probe = base_Q_bref * bref_Q_sref * sref_Q_probe;
    tf::quaternionTFToMsg(base_Q_probe, target_orientation_);

    // ++count_;

    // filter by average
    // if (count_ == 4){
    //     for (int i=0; i<3; i++){
    //         position_[i] /= 4;
    //         target_delta_[i] = position_[i]-position_prev_[i];
    //         position_prev_[i] = position_[i];
    //         position_[i] = 0;
    //     }
    //     count_ = 0;
    // }
}

void TouchTele::twistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    velocity_[0] = msg->twist.linear.x;
    velocity_[1] = msg->twist.linear.y;
    velocity_[2] = msg->twist.linear.z;

    // ++count_;
    // // filter by average
    // if (count_ == 4){
    //     for (int i=0; i<3; i++){
    //         velocity_[i] /= 4;
    //     }
    //     count_ = 0;
    // }
}

void TouchTele::teleControl()
{
    // Init robot state to HOME
    move_group_->setNamedTarget("home");
    move_group_->move();
    sleep(2);

    // fix orientation first 
    geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
    geometry_msgs::PoseStamped target_pose = current_pose;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;
    ros::Rate loop_rate(200);
    while (ros::ok()){
    // while (!teleIsActive_){}
        if (teleIsActive_){

            target_pose.header.stamp = ros::Time::now();
            target_pose.pose.position.x += velocity_[0] * 1;
            target_pose.pose.position.y += velocity_[1] * 1;
            target_pose.pose.position.z += velocity_[2] * 1;

            move_group_->setPoseTarget(target_pose);
            success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("Start point %s", success ? "" : "FAILED");
            move_group_->execute(my_plan);






        }
        loop_rate.sleep();
    }


    

}

int main(int argc, char** argv)
{
    ////////////////////////////////////////////////////////////////
    // Init ROS
    ////////////////////////////////////////////////////////////////
    ros::init(argc, argv, "Touch_Tele_Node");
    ros::NodeHandle nh;
    TouchTele touch_tele(nh);

    // run tele loop
    touch_tele.teleControl();

    // geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
    // ROS_INFO_STREAM(current_pose);
    // tf::Quaternion qua;
    // tf::quaternionMsgToTF(current_pose.pose.orientation, qua);
    // tf::Matrix3x3 matrix1(qua);
    // ROS_INFO_STREAM(matrix1[0][0]<<" "<<matrix1[0][1]<<" "<<matrix1[0][2]);
    // ROS_INFO_STREAM(matrix1[1][0]<<" "<<matrix1[1][1]<<" "<<matrix1[1][2]);
    // ROS_INFO_STREAM(matrix1[2][0]<<" "<<matrix1[2][1]<<" "<<matrix1[2][2]);

    // ros::shutdown();

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success;
    // // tf::StampedTransform transform_tf;
    // geometry_msgs::Pose target_pose;
    // target_pose.position.x = 0.324388;
    // target_pose.position.y = -0.0266423;
    // target_pose.position.z = 0.388319;
    // target_pose.orientation.x = -0.820758;
    // target_pose.orientation.y = 0.437589;
    // target_pose.orientation.z = 0.0736749;
    // target_pose.orientation.w = 0.359784;
    // move_group->setPoseTarget(target_pose);

    // // move_group->setPositionTarget(0.324388, -0.0266423, 0.388319);
    // // move_group->setOrientationTarget(-0.820758, 0.437589, 0.0736749, 0.359784);
    // success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO("Start point %s", success ? "" : "FAILED");
    // move_group->execute(my_plan);
    

    /*
    ros::Rate loop_rate(1);
    while(ros::ok()){
        
        // try{
        //     tf_listener->waitForTransform("world", "probe_end", ros::Time(0), ros::Duration(0.2));
        //     tf_listener->lookupTransform("world", "probe_end", ros::Time(0), transform_tf);
        // }
        // catch(tf::TransformException &ex){
        //     ROS_ERROR("tf listener: transform exception : %s",ex.what());
        // }
        // tf::pointTFToMsg(transform_tf.getOrigin(), target_pose.position);

        // Test 
        // ROS_INFO_STREAM("position x:"<< target_pose.position.x);
        // ROS_INFO_STREAM("position y:"<< target_pose.position.y);
        // ROS_INFO_STREAM("position z:"<< target_pose.position.z);


        // target_pose.position.x += touch_tele.target_delta_[0];
        // target_pose.position.y += touch_tele.target_delta_[1];
        // target_pose.position.z += touch_tele.target_delta_[2];
        // target_pose.orientation = touch_tele.target_orientation_;

        geometry_msgs::PoseStamped target_pose = move_group->getCurrentPose();
        target_pose.pose.orientation = touch_tele.target_orientation_;
        move_group->setPoseTarget(target_pose);
        // move_group->setOrientationTarget(touch_tele.target_orientation_.x, 
        //                                 touch_tele.target_orientation_.y,
        //                                 touch_tele.target_orientation_.z,
        //                                 touch_tele.target_orientation_.w);


		success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO("Start point %s", success ? "" : "FAILED");
		move_group->execute(my_plan);

        loop_rate.sleep();
    }
    */

    return 0;
}
