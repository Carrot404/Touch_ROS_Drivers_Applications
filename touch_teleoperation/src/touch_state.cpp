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

int calibrationStyle;

struct TouchState {
    /* Raw values */
    int buttons[2]; // HD_CURRENT_BUTTONS
    int buttons_prev[2]; // HD_LAST_BUTTONS

    /* Joint space values */      
    hduVector3Dd joint_angles; // HD_CURRENT_JOINT_ANGLES
    hduVector3Dd gimbal_angles; // HD_CURRENT_GIMBAL_ANGLES
    // hduVector3Dd joint_force; // HD_CURRENT_JOINT_TORQUE

    /* Cartesian space values */      
    hduVector3Dd position; // HD_CURRENT_POSITION
    hduQuaternion orientation; // HD_CURRENT_TRANSFORM
    hduVector3Dd linear_velocity; // HD_CURRENT_VELOCITY
    hduVector3Dd angular_velocity; // HD_CURRENT_ANGULAR_VELOCITY
    hduMatrix jacobian; // HD_CURRENT_JACOBIAN
    // hduVector3Dd cartesian_force; // HD_CURRENT_FORCE
    
    // bool lock;
    // bool close_gripper;
    // hduVector3Dd lock_pos;
    // double units_ratio;
};

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

    TouchState *state_;
    GeomagicStatus *geoStatus_;
	boost::shared_ptr<tf::TransformListener> tf_listener_;

public:
    // Constructor and destructor
    // PhantomROS(ros::NodeHandle nh, TouchState *state);
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
    /* std::string pose_topic = touch_name_+"/pose";
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic.c_str(), 1); */

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
    // state_ = state;
    geoStatus_ = state;
    // state_->buttons[0] = 0;
    // state_->buttons[1] = 0;
    // state_->buttons_prev[0] = 0;
    // state_->buttons_prev[1] = 0;
    // hduVector3Dd hduzeros(0, 0, 0);
    // state_->joint_angles = hduzeros;
    // state_->gimbal_angles = hduzeros;
    // // state_->joint_force = hduzeros;
    // state_->position = hduzeros;
    // hduQuaternion hduqua(1, hduzeros);
    // state_->orientation = hduqua;
    // state_->linear_velocity = hduzeros;
    // state_->angular_velocity = hduzeros;

    // state_->cartesian_force = hduzeros;
}

PhantomROS::~PhantomROS(){}

void PhantomROS::publish_touch_state()
{
    // Button publisher
    if ((state_->buttons[0] != state_->buttons_prev[0])
        or (state_->buttons[1] != state_->buttons_prev[1]))
    {
      touch_msgs::TouchButtonEvent button_msg;
      button_msg.grey_button = state_->buttons[0];
      button_msg.white_button = state_->buttons[1];
      state_->buttons_prev[0] = state_->buttons[0];
      state_->buttons_prev[1] = state_->buttons[1];
      button_pub_.publish(button_msg);
    }

    // Joint States publisher
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = ros::Time::now();
    joint_msg.name.resize(6);
    joint_msg.position.resize(6);
    joint_msg.name[0] = "waist";
    joint_msg.position[0] = -state_->joint_angles[0];
    joint_msg.name[1] = "shoulder";
    joint_msg.position[1] = state_->joint_angles[1];
    joint_msg.name[2] = "elbow";
    joint_msg.position[2] = state_->joint_angles[2]-state_->joint_angles[1];
    joint_msg.name[3] = "yaw";
    joint_msg.position[3] = -state_->gimbal_angles[0];
    joint_msg.name[4] = "pitch";
    joint_msg.position[4] = state_->gimbal_angles[1]+1.49;
    joint_msg.name[5] = "roll";
    joint_msg.position[5] = state_->gimbal_angles[2];
    joint_pub_.publish(joint_msg);

    // Pose_ref publisher
    geometry_msgs::PoseStamped poseref_msg;
    poseref_msg.header.frame_id = "base_ref";
    poseref_msg.header.stamp = joint_msg.header.stamp;
    poseref_msg.pose.position.x = state_->position[0]/1000.0;
    poseref_msg.pose.position.y = state_->position[1]/1000.0;
    poseref_msg.pose.position.z = state_->position[2]/1000.0;
    poseref_msg.pose.orientation.x = state_->orientation.v()[0];
    poseref_msg.pose.orientation.y = state_->orientation.v()[1];
    poseref_msg.pose.orientation.z = state_->orientation.v()[2];
    poseref_msg.pose.orientation.w = state_->orientation.s(); 

    // poseref_msg.pose.position.x = geoStatus_->stylusPosition[0];
    // poseref_msg.pose.position.y = geoStatus_->stylusPosition[1];
    // poseref_msg.pose.position.z = geoStatus_->stylusPosition[2];

    poseref_pub_.publish(poseref_msg);

    // Pose_based urdf publisher
    /* tf::StampedTransform transform_tf;
    try{
        tf_listener_->waitForTransform("base", "stylus", ros::Time(0), ros::Duration(0.5));
        tf_listener_->lookupTransform("base", "stylus", ros::Time(0), transform_tf);
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("tf listener: transform exception : %s",ex.what());
        return;
    }
    tf::Quaternion quat_tf =  transform_tf.getRotation();
    tf::Point vec_tf = transform_tf.getOrigin();
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "base";
    pose_msg.header.stamp = joint_msg.header.stamp;
    tf::pointTFToMsg(vec_tf, pose_msg.pose.position);
    tf::quaternionTFToMsg(quat_tf, pose_msg.pose.orientation);
    pose_pub_.publish(pose_msg); */

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
    twist_msg.twist.linear.x = -state_->linear_velocity[0]/1000.0;
    twist_msg.twist.linear.y = state_->linear_velocity[2]/1000.0;
    twist_msg.twist.linear.z = state_->linear_velocity[1]/1000.0;
    twist_msg.twist.angular.x = state_->angular_velocity[0];
    twist_msg.twist.angular.y = state_->angular_velocity[1];
    twist_msg.twist.angular.z = state_->angular_velocity[2];
    twist_pub_.publish(twist_msg);


}

// HDCallbackCode HDCALLBACK touch_state_callback(void *pUserData)
HDCallbackCode touch_state_callback(void *pUserData)
{
    TouchState *touch_state = static_cast<TouchState *>(pUserData);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
        ROS_DEBUG("Updating calibration...");
        hdUpdateCalibration(calibrationStyle);
    }
    //------------------------------------------------------------------
    hdBeginFrame(hdGetCurrentDevice());

    int nButtons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    touch_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    touch_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, touch_state->joint_angles);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, touch_state->gimbal_angles);

    // hdGetDoublev(HD_CURRENT_POSITION, touch_state->position);
    hduMatrix transform_ref;
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform_ref);
    transform_ref.getRotation(touch_state->orientation);
    touch_state->position = hduVector3Dd(transform_ref[3][0], transform_ref[3][1], transform_ref[3][2]);

    hdGetDoublev(HD_CURRENT_VELOCITY, touch_state->linear_velocity);
    // TODO: can not read HD_CURRENT_ANGULAR_VELOCITY
    // solution: difference from history
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, touch_state->angular_velocity);

    hdEndFrame(hdGetCurrentDevice());
    //-------------------------------------------------------------------
    
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Error during main scheduler callback");
        // ROS_ERROR("Error during main scheduler callback: %s", &error);
        if (hduIsSchedulerError(&error))
            return HD_CALLBACK_DONE;
    }

    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
/* void HHD_Auto_Calibration() {
    int supportedCalibrationStyles;
    HDErrorInfo error;

    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
        ROS_INFO("HD_CALIBRATION_ENCODER_RESET..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
        calibrationStyle = HD_CALIBRATION_INKWELL;
        ROS_INFO("HD_CALIBRATION_INKWELL..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
        calibrationStyle = HD_CALIBRATION_AUTO;
        ROS_INFO("HD_CALIBRATION_AUTO..");
    }
    if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
        do {
            hdUpdateCalibration(calibrationStyle);
            ROS_INFO("Calibrating.. (put stylus in well)");
            if (HD_DEVICE_ERROR(error = hdGetError())) {
                hduPrintError(stderr, &error, "Reset encoders reset failed.");
                // ROS_ERROR("Reset encoders reset failed: %s", &error);
                break;
            }
        } while (hdCheckCalibration() != HD_CALIBRATION_OK);
        ROS_INFO("Calibration complete.");
    }
    while(hdCheckCalibration() != HD_CALIBRATION_OK) {
        usleep(1e6);
        if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
            ROS_INFO("Please place the device into the inkwell for calibration");
        else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
            ROS_INFO("Calibration updated successfully");
            hdUpdateCalibration(calibrationStyle);
        }
        else
        ROS_FATAL("Unknown calibration status");
    }
} */

/* void *ros_publish(void *ptr) {
    PhantomROS *touch = (PhantomROS *) ptr;
    ROS_INFO("Publishing Phantom state at [%d] Hz", touch->getPubRate());
    ros::Rate loop_rate(touch->getPubRate());
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok()) {
        touch->publish_touch_state();
        loop_rate.sleep();
    }
    return NULL;
} */

int main(int argc, char** argv)
{
    ////////////////////////////////////////////////////////////////
    // Init ROS
    ////////////////////////////////////////////////////////////////
    ros::init(argc, argv, "Touch_State_Node");
    ros::NodeHandle nh;

    // init GeomagicProxy
    GeomagicStatus geoStatus;
    GeomagicProxy touchProxy(&geoStatus);

    touchProxy.run();

    // while(ros::ok()){
        // ROS_INFO_STREAM(geoStatus.stylusGimbalAngles[0]<< " " 
        //              << geoStatus.stylusGimbalAngles[1]<< " "
        //              << geoStatus.stylusGimbalAngles[2]<< " ");
    // }
    // touchProxy.stop();


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