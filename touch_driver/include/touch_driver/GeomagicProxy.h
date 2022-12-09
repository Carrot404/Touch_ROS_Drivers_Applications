/** GeomagicProxy.h
 * 
 * This code is cloned from https://github.com/marcofer/portable-dvrk-simulator.git
 * and modified by Songjie Xiao (songjiexiao@zju.edu.cn) mainly for academic use.
 * 
 */

#ifndef GEOMAGICPROXY_HPP_
#define GEOMAGICPROXY_HPP_

#include <unistd.h>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduVector.h>
#include <HDU/hduQuaternion.h>

#define HD_MY_DEVICE "carrot"
#define SPACE_DIM 3
#define GEOMAGIC_HAPTIC_DOF 3
#define GEOMAGIC_HAPTIC_JOINTS 6

namespace Eigen {
	typedef Eigen::Matrix<float, 7, 1> Vector7f;
	typedef Eigen::Matrix<double, 7, 1> Vector7d;
	typedef Eigen::Matrix<float, 6, 1> Vector6f;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

// struct HapticState {

// 	int asyncTriggerState;				//!< State corresponding to asyncronous triggered events (e.g., stylus buttons)
// 	bool* buttonState;					//!< Array with the state of the buttons placed on the device
// 	Eigen::VectorXf hapticForce;		//!< Vector of the haptic force of the device
// 	Eigen::Vector3f hipPosition;		//!< Vector of the 3D position of the HIP
// 	Eigen::Vector3f hipGimbalAngles;	//!< Vector of the 3D orientation of the HIP
// 	Eigen::Vector6f hipVelocity;		//!< Vector of the 6D velocity of the HIP
// };

enum BUTTONS { GEOMAGIC_LOW_BUTTON, GEOMAGIC_HIGH_BUTTON, GEOMAGIC_BUTTONS_NUM };
enum PRESSED_BUTTONS { NO_PRESSED, PRESSED_LOW, PRESSED_HIGH, PRESSED_BOTH };
enum WORKSPACE {JOINT_SPACE, CARTESIAN_SPACE};

struct HDUtilityData {

	// linear history position/velocity
	hduVector3Dd prvPos;               //!< previous position
	hduVector3Dd lstPos;               //!< pre-previous position

	hduVector3Dd prvInputVel;          //!< previous input linear velocity
	hduVector3Dd lstInputVel;          //!< pre-previous input linear velocity
	hduVector3Dd vrLstInputVel;        //!< pre-pre-previous input linearvelocity
	hduVector3Dd prvOutVel;            //!< previous output linear velocity
	hduVector3Dd lstOutVel;            //!< pre-previous output linear velocity
	hduVector3Dd vrLstOutVel;          //!< pre-pre-previous output linear velocity

	hduVector3Dd lvelocity;            //!< linear velocity
	hduVector3Dd lvelocityTemp;        //!< temp linear velocity

	// history pos joint value/velocity
	hduVector3Dd prvAng0;              //!< previous position angles
	hduVector3Dd lstAng0;              //!< pre-previous position angles

	hduVector3Dd prvInputAngVel0;       //!< previous position input velocity
	hduVector3Dd lstInputAngVel0;       //!< pre-previous position input velocity
	hduVector3Dd vrLstInputAngVel0;     //!< pre-pre-previous position input velocity
	hduVector3Dd prvOutAngVel0;         //!< previous position output velocity
	hduVector3Dd lstOutAngVel0;         //!< pre-previous position output velocity
	hduVector3Dd vrLstOutAngVel0;       //!< pre-pre-previous position output velocity

	hduVector3Dd avelocity0;            //!< position velocity
	hduVector3Dd avelocityTemp0;        //!< temp position velocity

	// history gimbal joint value/velocity
	hduVector3Dd prvAng;               //!< previous gimbal angles
	hduVector3Dd lstAng;               //!< pre-previous gimbal angles

	hduVector3Dd prvInputAngVel;       //!< previous gimbal input velocity
	hduVector3Dd lstInputAngVel;       //!< pre-previous gimbal input velocity
	hduVector3Dd vrLstInputAngVel;     //!< pre-pre-previous gimbal input velocity
	hduVector3Dd prvOutAngVel;         //!< previous gimbal output velocity
	hduVector3Dd lstOutAngVel;         //!< pre-previous gimbal output velocity
	hduVector3Dd vrLstOutAngVel;       //!< pre-pre-previous gimbal output velocity

	hduVector3Dd avelocity;            //!< gimbal velocity
	hduVector3Dd avelocityTemp;        //!< temp gimbal velocity

};

struct GeomagicStatus {

	/* Device state */
	// TODO: get Device status such as inkwell status
	// HDdouble inwell_switch;
	// HDdouble hdrate;

	/* Cartesian space values */ 
	hduVector3Dd stylusPosition;									//!< Raw Position of HIP (Haptic Interface Point) 
	hduQuaternion stylusOrientation;                                //!< Raw Orientation of HIP (Haptic Interface Point)
	hduVector3Dd stylusLinearVelocity;								//!< Raw Linear velocity of HIP (Haptic Interface Point)
	hduVector3Dd stylusAngularVelocity;								//!< Raw Angular velocity of HIP (Haptic Interface Point) (Not readable)
	hduVector3Dd stylusForce;                                       //!< Raw Cartesian force
	std::vector<double> cartPose;                                   //!< Pose vetor in order like this [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w] (URDF) TODO: transform orientation
	std::vector<double> cartTwist;                                  //!< Twist vetor (URDF) TODO: compute velocity
	std::vector<double> cartWrench;                                 //!< Wrench vetor (URDF) (DIM=3)
	
	double jacobian[SPACE_DIM * 2 * GEOMAGIC_HAPTIC_JOINTS];        //!< Raw Vectorized Jacobian matrix (TODO: Not verified yet)

	/* Joint space values */
	hduVector3Dd PosAngles;                                         //!< Raw joint angles
	hduVector3Dd GimbalAngles;							        	//!< Raw gimbal angles
	// hduVector3Dd PosAnglesVel;                                      //!< Raw joint angles velocity (not exist)
	// hduVector3Dd GimbalAnglesVel;                                   //!< Raw gimbal angles velocity (not exist)
	hduVector3Dd PosAnglesEff;							    		//!< Raw joint angles effort
	// hduVector3Dd GimbalAnglesEff;                                   //!< Raw gimbal angles effort (not exist)
	std::vector<double> jointPosition;                              //!< Array of Geomagic joint positions (according to URDF)
	std::vector<double> jointVelocity;                              //!< Array of Geomagic joint velocity (computed by difference URDF)
	std::vector<double> jointEffort;                                //!< Array of Geomagic joint effort (DIM=3 URDF)

	/* Button state */
	bool buttons[GEOMAGIC_BUTTONS_NUM];                             //!< Status of the buttons on the Geomagic stylus
	bool buttons_prev[GEOMAGIC_BUTTONS_NUM];                        //!< Previous Status of the buttons on the Geomagic stylus
	bool action[GEOMAGIC_BUTTONS_NUM];								//!< Raise + Trail action 
	bool evRaiseEdge[GEOMAGIC_BUTTONS_NUM];							//!< Raise action from unpressed to pressed
	bool evTrailEdge[GEOMAGIC_BUTTONS_NUM];							//!< Trail action from pressed to unpressed
};

/**
* @brief Callback function
* Update the state of the Geomagic device
* @param data the data containing the updated status
*/
HDCallbackCode updateGeoStateCallback(void* data);

/**
* @brief Callback function
* Force feedback callback of the Geomagic device
* @param data the data containing the force feedback data
*/
HDCallbackCode forceFeedbackCallback(void* data);

class GeomagicProxy {

	/**
	* @brief Callback function
	* Update the state of the Geomagic device
	* @param data the data containing the updated status
	*/
	friend HDCallbackCode updateGeoStateCallback(void* data);

	/**
	* @brief Callback function
	* Force feedback callback of the Geomagic device
	* @param data the data containing the force feedback data
	*/
	friend HDCallbackCode forceFeedbackCallback(void* data);

public:

	/**
	* @brief Default contructor of GeomagicProxy class
	*/
	GeomagicProxy();

	/**
	* @brief Default destroyer of GeomagicProxy class
	*/
	virtual ~GeomagicProxy() = default;

	/**
	* @brief Default run function
	*/
	void run();

	/**
	* @brief Default stop function
	*/
	void stop();

	/**
	* @brief Set function
	* Set to Joint space haptic force on the Geomagic device, which is default mode.
	*/
	void setJointForce(){this->mode_ = JOINT_SPACE;}

	/**
	* @brief Set function
	* Set to Cartesian space haptic force on the Geomagic device
	*/
	void setCartForce(){this->mode_ = CARTESIAN_SPACE;}

	/**
	* @brief Set function
	* Set the command force to the Geomagic device according to the mode.
	* @param Fx 
	* @param Fy
	* @param Fz
	* TODO: raw force or urdf force 
	*/
	void setForceCommand(double Fx, double Fy, double Fz);

	/**
	* @brief Set function
	* Set the command force to the Geomagic device according to the mode.
	* @param vec force vector to be set
	* TODO: raw force or urdf force 
	*/
	void setForceCommand(std::vector<double> vec);

	/**
	* @brief Check function
	* Check if the external system is available
	* @return true if the external system is available
	*/
	inline bool isAvailable() { return this->available_; }

	/**
	* @brief Check function
	* Check if the main loop of the external system is running
	* @return true if the main loop of the external system is running
	*/
	inline bool isRunning() { return this->running_; }

	/**
	* @brief Set function
	* Set the available flag
	* @param aval value of the available flag to be set
	*/
	inline void setAvailable(const bool& aval) { this->available_ = aval; }

	/**
	* @brief Set function
	* Set the running flag
	* @param running value of the running flag to be set
	*/
	inline void setRunning(const bool& running) { this->running_ = running; }

	/**
	* @brief Get function
	* Get HD_UPDATE_RATE
	* @return HD_UPDATE_RATE
	*/
	// double getControlFrequency() { return this->geoStatus_->hdrate; }

	std::shared_ptr<GeomagicStatus> getDataPackage() {return geoStatus_;}


protected:

	/**
	* @brief Calibration function 
	* Calibrate the Geomagic device
	* @return true if the device has been successfully calibrated
	*/
	void HHD_Auto_Calibration();

	/**
	* @brief Event catch function
	* Catch the event when the input button has been pressed, computed on the consecutive raising and trailing edges of the button pressing state
	* @param button: the current state of the pressed button
	* @param button_prev: the previous state of the pressed button
	* @param raise: the raising edge of the event
	* @param trail: the trailing edge of the event
	* @param trigger: the boolean value to be returned
	*/
	void catchButtonPressEvent(const bool& button, bool& button_prev, bool& raise, bool& trail, bool& trigger);

	/**
	* @brief Update function
	* Update the Cartesian velocities of the Geomagic stylus
	* Set internally cartVelocity
	* TODO: update angular velocity
	*/
	void updateCartesianVelocities();

	/**
	* @brief Update function
	* Update the Joint Velocity of the Geomagic stylus
	* Set internally jointVelocity
	*/
	void updateJointVelocities();

	HHD dvcHandle_;								//!< OpenHaptics device handler
	HDSchedulerHandle schHandle_;				//!< OpenHaptics scheduler handler

	std::shared_ptr<GeomagicStatus> geoStatus_; //!< Structure containing the main quantities defining the state of the haptic device (see above)
	std::shared_ptr<HDUtilityData> hdUtils_;    //!< Structure containing some utility variables necessary to process linear and angular velocities

	bool available_;					     	//!< Flag stating if the external system is available
	bool running_;					    	    //!< Flag stating if the main loop of the system is running
	int mode_;                             //!< Flag stating control force in JOINT_SPACE or CARTESIAN_SPACE
	
	hduVector3Dd command_;                      //!< force command
};

#endif // GEOMAGICPROXY_HPP_
