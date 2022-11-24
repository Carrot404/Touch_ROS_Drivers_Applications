/** touch_state.cpp
 * 
 * 
 * 
 * 
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <touch_msgs/TouchButtonEvent.h>


#include <boost/shared_ptr.hpp>
#include <pthread.h>

#include "GeomagicProxy.hpp"

using namespace std;

class PhantomROS
{
private:
    // ROS Parameters
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    ros::Publisher button_pub_;
    ros::Publisher joint_pub_;
    ros::Publisher poseref_pub_;
    // ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;


    // ros::Publisher state_pub_;
    // ros::Subscriber haptic_sub;
    string touch_name_;
    string device_name_;
    int publish_rate_;

    GeomagicStatus *geoStatus_;
	boost::shared_ptr<tf::TransformListener> tf_listener_;

public:
    // Constructor and destructor
    PhantomROS(ros::NodeHandle nh, GeomagicStatus *state);
    ~PhantomROS();

    // Get param
    string getDeviceName(){return device_name_;}
    int getPubRate(){return publish_rate_;}


    void publish_touch_state();



};

PhantomROS::PhantomROS(ros::NodeHandle nh, GeomagicStatus *state) : nh_(nh), priv_nh_("~")
{
    // ROS Parameters
    priv_nh_.param<std::string>("device_name", device_name_, "carrot");
    priv_nh_.param<std::string>("touch_namespace", touch_name_, "Touch");
    priv_nh_.param<int>("publish_rate", publish_rate_, 500);

    // Publish button state on NAMESPACE/button
    std::string button_topic = touch_name_+"/button";
    button_pub_ = nh_.advertise<touch_msgs::TouchButtonEvent>(button_topic.c_str(), 10);

    // Publish joint state on NAMESPACE/joint_states
    std::string joint_topic = touch_name_+"/joint_states";
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_topic.c_str(), 1);

    // Publish pose on NAMESPACE/pose_ref based on real frame
    // base_frame: base_ref
    // target_frame: stylus_ref
    std::string poseref_topic = touch_name_+"/pose_ref";
    poseref_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(poseref_topic.c_str(), 1);

    // Publish pose on NAMESPACE/pose based on urdf defined frame
    // base_frame: base
    // target_frame: stylus
    // std::string pose_topic = touch_name_+"/pose";
    // pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic.c_str(), 1);

    // Publish pose on NAMESPACE/twist based on desired frame
    // base_frame: base
    // target_frame: probe
    std::string twist_topic = touch_name_+"/twist";
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_topic.c_str(), 1);  

    tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
    try{
        tf_listener_->waitForTransform("base", "stylus", ros::Time(0), ros::Duration(2));
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("tf listener: transform exception : %s",ex.what());
    }

    // TouchState Init
    geoStatus_ = state;

}

PhantomROS::~PhantomROS(){}

void PhantomROS::publish_touch_state()
{
    // Button publisher
    touch_msgs::TouchButtonEvent button_msg;
    button_msg.grey_button = geoStatus_->buttons[0];
    button_msg.white_button = geoStatus_->buttons[1];
    button_msg.grey_button_action = geoStatus_->action[0];
    button_msg.white_button_action = geoStatus_->action[1];
    button_pub_.publish(button_msg);

    // Joint States publisher
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = ros::Time::now();
    joint_msg.name.resize(6);
    joint_msg.name[0] = "waist";
    joint_msg.name[1] = "shoulder";
    joint_msg.name[2] = "elbow";
    joint_msg.name[3] = "yaw";
    joint_msg.name[4] = "pitch";
    joint_msg.name[5] = "roll";
    joint_msg.position.resize(6);
    joint_msg.position[0] = geoStatus_->PosAngles[0];
    joint_msg.position[1] = geoStatus_->PosAngles[1];
    joint_msg.position[2] = geoStatus_->PosAngles[2];
    joint_msg.position[3] = geoStatus_->GimbalAngles[0];
    joint_msg.position[4] = geoStatus_->GimbalAngles[1];
    joint_msg.position[5] = geoStatus_->GimbalAngles[2];
    joint_msg.velocity.resize(6);
    joint_msg.velocity[0] = geoStatus_->PosAnglesVel[0];
    joint_msg.velocity[1] = geoStatus_->PosAnglesVel[1];
    joint_msg.velocity[2] = geoStatus_->PosAnglesVel[2];
    joint_msg.velocity[3] = geoStatus_->GimbalAnglesVel[0];
    joint_msg.velocity[4] = geoStatus_->GimbalAnglesVel[1];
    joint_msg.velocity[5] = geoStatus_->GimbalAnglesVel[2];
    joint_msg.effort.resize(6);
    joint_msg.effort[0] = geoStatus_->effort[0];
    joint_msg.effort[1] = geoStatus_->effort[1];
    joint_msg.effort[2] = geoStatus_->effort[2];
    joint_msg.effort[3] = 0;
    joint_msg.effort[4] = 0;
    joint_msg.effort[5] = 0;
    joint_pub_.publish(joint_msg); 

    // Pose_ref publisher
    geometry_msgs::PoseStamped poseref_msg;
    poseref_msg.header.frame_id = "base_ref";
    poseref_msg.header.stamp = joint_msg.header.stamp;
    poseref_msg.pose.position.x = geoStatus_->stylusPosition[0];
    poseref_msg.pose.position.y = geoStatus_->stylusPosition[1];
    poseref_msg.pose.position.z = geoStatus_->stylusPosition[2];
    poseref_msg.pose.orientation.x = geoStatus_->stylusOrientation.v()[0];
    poseref_msg.pose.orientation.y = geoStatus_->stylusOrientation.v()[1];
    poseref_msg.pose.orientation.z = geoStatus_->stylusOrientation.v()[2];
    poseref_msg.pose.orientation.w = geoStatus_->stylusOrientation.s(); 
    poseref_pub_.publish(poseref_msg);

    // Pose_based urdf publisher
    // tf::StampedTransform transform_tf;
    // try{
    //     tf_listener_->waitForTransform("base", "stylus", ros::Time(0), ros::Duration(0.5));
    //     tf_listener_->lookupTransform("base", "stylus", ros::Time(0), transform_tf);
    // }
    // catch(tf::TransformException &ex){
    //     ROS_ERROR("tf listener: transform exception : %s",ex.what());
    //     return;
    // }
    // tf::Quaternion quat_tf =  transform_tf.getRotation();
    // tf::Point vec_tf = transform_tf.getOrigin();
    // geometry_msgs::PoseStamped pose_msg;
    // pose_msg.header.frame_id = "base";
    // pose_msg.header.stamp = joint_msg.header.stamp;
    // tf::pointTFToMsg(vec_tf, pose_msg.pose.position);
    // tf::quaternionTFToMsg(quat_tf, pose_msg.pose.orientation);
    // pose_pub_.publish(pose_msg);

    // Twist publisher
    // tf::StampedTransform transform_tf;
    // try{
    //     tf_listener_->waitForTransform("base", "probe", ros::Time(0), ros::Duration(0.5));
    //     tf_listener_->lookupTransform("base", "probe", ros::Time(0), transform_tf);
    // }
    // catch(tf::TransformException &ex){
    //     ROS_ERROR("tf listener: transform exception : %s",ex.what());
    //     return;
    // }
    // tf::Quaternion quat_tf =  transform_tf.getRotation();
    // tf::Point vec_tf = transform_tf.getOrigin();

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = "base";
    twist_msg.header.stamp = joint_msg.header.stamp;
    twist_msg.twist.linear.x = -geoStatus_->stylusLinearVelocity[0];
    twist_msg.twist.linear.y = geoStatus_->stylusLinearVelocity[2];
    twist_msg.twist.linear.z = geoStatus_->stylusLinearVelocity[1];
    twist_msg.twist.angular.x = geoStatus_->stylusAngularVelocity[0];
    twist_msg.twist.angular.y = geoStatus_->stylusAngularVelocity[1];
    twist_msg.twist.angular.z = geoStatus_->stylusAngularVelocity[2];
    twist_pub_.publish(twist_msg);

}

void *ros_publish(void *ptr) 
{
    PhantomROS *touch = (PhantomROS *) ptr;
    ROS_INFO("Publishing Phantom state at [%d] Hz", touch->getPubRate());
    ros::Rate loop_rate(touch->getPubRate());
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok()) {
        touch->publish_touch_state();
        loop_rate.sleep();
    }
    return nullptr;
}

void *state_update(void *ptr)
{
    GeomagicProxy *touchProxy = (GeomagicProxy *) ptr;
    touchProxy->run();
    return nullptr;
}

int main(int argc, char** argv)
{
    // Init ROS
    ros::init(argc, argv, "Touch_State_Node");
    ros::NodeHandle nh;

    GeomagicStatus geoStatus;
    GeomagicProxy touchProxy(&geoStatus);
    PhantomROS touch(nh, &geoStatus);

    // loop and update geoStatus
    pthread_t state_thread;
    pthread_create(&state_thread, NULL, state_update, (void*) &touchProxy);

    // loop and publish state
    pthread_t publish_thread;
    pthread_create(&publish_thread, NULL, ros_publish, (void*) &touch);
    pthread_join(publish_thread, NULL);

    ROS_INFO("Ending Session....");
    touchProxy.stop();

    // loop and publish
    // pthread_t publish_thread;
    // pthread_create(&publish_thread, NULL, ros_publish, (void*) &touch);
    // pthread_detach(publish_thread);
    // pthread_join(publish_thread, NULL);


    ////////////////////////////////////////////////////////////////
    // Init Phantom
    ////////////////////////////////////////////////////////////////
    /**************************************************************

    HDErrorInfo error;
    HHD hHD;
    hHD = hdInitDevice(touch.getDeviceName().c_str());
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        // ROS_ERROR("Failed to initialize haptic device: %s", &error);
        return false;
    }
    ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        ROS_ERROR("Failed to start the scheduler"); //, &error);
        return false;
    }
    HHD_Auto_Calibration();
    // open hd thread
    // HDSchedulerHandle schHandle = hdScheduleAsynchronous(touch_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);
    hdScheduleSynchronous(touch_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);
    **************************************************************/


    ////////////////////////////////////////////////////////////////
    // Loop and publish
    ////////////////////////////////////////////////////////////////

    // pthread_t publish_thread;
    // pthread_create(&publish_thread, NULL, ros_publish, (void*) &touch);
    // pthread_join(publish_thread, NULL);
    


    // ROS_INFO("Ending Session....");
    // hdStopScheduler();
    // hdDisableDevice(hHD);

    return 0;
    


}