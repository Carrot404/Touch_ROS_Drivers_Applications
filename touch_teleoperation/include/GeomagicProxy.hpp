/** GeomagicProxy.hpp
 * 
 * This code is cloned from https://github.com/marcofer/portable-dvrk-simulator.git
 * and modified by Songjie Xiao (songjiexiao@zju.edu.cn) mainly for academic use.
 * 
 */

#ifndef GEOMAGICPROXY_HPP_
#define GEOMAGICPROXY_HPP_

#include <vector>

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduVector.h>
#include <HDU/hduQuaternion.h>

//Project Header files
#include "Timer.hpp"
#include "utils.hpp"

#define HD_MY_DEVICE "carrot"
#define GEOMAGIC_HAPTIC_DOF 3
#define GEOMAGIC_HAPTIC_JOINTS 6

struct HapticState {

	int asyncTriggerState;				//!< State corresponding to asyncronous triggered events (e.g., stylus buttons)
	bool* buttonState;					//!< Array with the state of the buttons placed on the device
	Eigen::VectorXf hapticForce;		//!< Vector of the haptic force of the device
	Eigen::Vector3f hipPosition;		//!< Vector of the 3D position of the HIP
	Eigen::Vector3f hipGimbalAngles;	//!< Vector of the 3D orientation of the HIP
	Eigen::Vector6f hipVelocity;		//!< Vector of the 6D velocity of the HIP

};

enum BUTTONS { GEOMAGIC_LOW_BUTTON, GEOMAGIC_HIGH_BUTTON, GEOMAGIC_BUTTONS_NUM };
enum PRESSED_BUTTONS { NO_PRESSED, PRESSED_LOW, PRESSED_HIGH, PRESSED_BOTH };



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

	/* Cartesian space values */      
	hduVector3Dd stylusPosition;									//!< Position vector of the styus HIP (Haptic Interface Point) of the Geomagic device
	hduQuaternion stylusOrientation;                                //!< Orientation vector of the styus HIP (Haptic Interface Point) of the Geomagic device
	hduVector3Dd stylusLinearVelocity;								//!< Linear velocity vector of the styus HIP (Haptic Interface Point) of the Geomagic device
	hduVector3Dd stylusAngularVelocity;								//!< Angular velocity vector of the styus HIP (Haptic Interface Point) of the Geomagic device
	double jacobian[SPACE_DIM * 2 * GEOMAGIC_HAPTIC_JOINTS];		//!< Vectorized Jacobian matrix
	hduVector3Dd force;												//!< Force vector of the Geomagic device
	

	/* Joint space values */
	hduVector3Dd PosAngles;                                         //!< first three joint angles of the stylus of the Geomagic device
	hduVector3Dd GimbalAngles;							        	//!< last three gimbal angles of the stylus of the Geomagic device
	hduVector3Dd PosAnglesVel;                                      //!< velocity of first three joint angles
	hduVector3Dd GimbalAnglesVel;                                   //!< velocity of last three gimbal joint angles
	hduVector3Dd effort;											//!< first three joint effort
	double jointPosition[GEOMAGIC_HAPTIC_JOINTS];				    //!< Array of Geomagic joint positions
	double jointVelocity[GEOMAGIC_HAPTIC_JOINTS];					//!< Array of Geomagic joint velocity

	/* Button state */
	bool buttons[GEOMAGIC_BUTTONS_NUM];                             //!< Status of the buttons on the Geomagic stylus
	bool buttons_prev[GEOMAGIC_BUTTONS_NUM];                        //!< Previous Status of the buttons on the Geomagic stylus
	bool action[GEOMAGIC_BUTTONS_NUM];								//!< Raise + Trail action 
	bool evHoldButton[GEOMAGIC_BUTTONS_NUM];						//!< button state
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
	*
	*/
	GeomagicProxy(GeomagicStatus* state);

	/**
	* @brief Default destroyer of GeomagicProxy class
	*
	*/
	~GeomagicProxy();

	/**
	* @brief Default run function
	*/
	void run();

	/**
	* @brief Default stop function
	*/
	void stop();

	/**
	* @brief Update function
	* Update the linear and angular velocities of the Geomagic stylus
	* Set internally stylusLinearVelocity and stylusAngularVelocity
	*/
	void updateVelocities();

	/**
	* @brief Update function
	* Update the Joint Velocity of the Geomagic stylus
	* Set internally jointVelocity
	*/
	void updateJointVelocities();

	/**
	* @brief New Calibration function 
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
	* @brief Set function
	* Set the feedback haptic force on the Geomagic device
	* @param f: the feedback force to be set
	*/
	void setHapticForce(const Eigen::VectorXf& f);

	/**
	* @brief Set function
	* Set the 3D vector of the hip position of the device
	* @param p: the 3D vector of the hip position of the device
	*/
	void setHIPPosition(const Eigen::Vector3f& p);

	/**
	* @brief Set function
	* Set the 3D vector of the hip orientation of the device
	* @param p: the 3D vector of the hip orientation of the device
	*/
	void setHIPGimbalAngles(const Eigen::Vector3f& ga);

	/**
	* @brief Set function
	* Set the 6D vector of the hip velocity of the device
	* @param v: the 6D vector of the hip velocity of the device
	*/
	void setHIPVelocity(const Eigen::Vector6f& v);

	/**
	* @brief Set function
	* Set the dynamic array with the state (on/off) of the buttons placed on the device
	* @param state: the dynamic array with the state (on/off) of the buttons placed on the device
	*/
	void setButtonState(const bool* state);

	/**
	* @brief Get function
	* Get the feedback haptic force on the Geomagic device
	* @return the feedback force
	*/
	Eigen::VectorXf getHapticForce();

	/**
	* @brief Get function
	* Get the 3D vector of the hip position of the device
	* @return the 3D vector of the hip position of the device
	*/
	Eigen::Vector3f getHIPPosition();

	/**
	* @brief Get function
	* Get the 3D vector of the hip orientation of the device
	* @return the 3D vector of the hip orientation of the device
	*/
	Eigen::Vector3f getHIPGimbalAngles();

	/**
	* @brief Get function
	* Get the 6D velocity of the HIP
	* @return the 6D velocity of the HIP
	*/
	Eigen::Vector6f getHIPVelocity();

	/**
	* @brief Get function
	* Get the dynamic array with the state (on/off) of the buttons placed on the device
	* @return the dynamic array with the state (on/off) of the buttons placed on the device
	*/
	bool* getButtonState();

	/**
	* @brief Get function
	* Get the HapticState structure from the current class
	* @return the HapticState structure
	*/
	inline HapticState getHapticState() {

		HapticState hs;

		hs.hipPosition = this->getHIPPosition();
		hs.hipGimbalAngles = this->getHIPGimbalAngles();
		hs.hipVelocity = this->getHIPVelocity();
		hs.hapticForce = this->getHapticForce();
		hs.buttonState = this->getButtonState();

		return hs;

	}

	/**
	* @brief Check function
	* Check if the external system is available
	* @return true if the external system is available
	*/
	inline bool isAvailable() { return this->available; }

	/**
	* @brief Check function
	* Check if the main loop of the external system is running
	* @return true if the main loop of the external system is running
	*/
	inline bool isRunning() { return this->running; }

	/**
	* @brief Set function
	* Set the available flag
	* @param the value of the available flag to be set
	*/
	inline void availability(const bool& aval) { this->available = aval; }

	/**
	* @brief Set function
	* Set the running flag
	* @param the value of the running flag to be set
	*/
	inline void setRunning(const bool& running_) { this->running = running_; }

	// inline void setLogPath(const std::string& path) { this->logPath = path; }

	// void saveLog();

private:

	HHD dvcHandle;								//!< OpenHaptics device handler
	HDSchedulerHandle schHandle;				//!< OpenHaptics scheduler handler

	GeomagicStatus* geoStatus;					//!< Structure containing the main quantities defining the state of the haptic device (see above)
	HDUtilityData hdUtils;						//!< Structure containing some utility variables necessary to process linear and angular velocities

	bool available;						//!< Flag stating if the external system is available
	bool running;						//!< Flag stating if the main loop of the system is running
	
	// boost::thread proxy_thread;

	// std::string logPath;
	// Geomagic stringstream
	// std::stringstream leftHipBaseVelSS;		//!< 6D Velocity of the left hip with respect to the base frame of the left geomagic
	// std::stringstream rightHipBaseVelSS;	//!< 6D Velocity of the right hip with respect to the base frame of the right geomagic
	// std::stringstream leftHipPSMVelSS;		//!< 6D Velocity of the left hip with respect to the base frame of the Left PSM
	// std::stringstream rightHipPSMVelSS;		//!< 6D Velocity of the right hip with respect to the base frame of the Left PSM
	// std::stringstream leftHipRotSS;			//!< Rotation matrix expressing the orientation of the left hip wrt the base frame of the left geomagic
	// std::stringstream rightHipRotSS;		//!< Rotation matrix expressing the orientation of the left hip wrt the base frame of the right geomagic
	// std::stringstream leftHipPosSS;			//!< Position of the left hip wrt the base frame of the left geomagic
	// std::stringstream rightHipPosSS;		//!< Position of the right hip wrt the base frame of the right geomagic

	// PSM stringstream
	// std::stringstream qdotPSM1SS;			//!< 6x1 Vector of PSM1 joint velocity
	// std::stringstream qdotPSM2SS;			//!< 6x1 Vector of PSM2 joint velocity
	// std::stringstream qPSM1SS;				//!< 6x1 Vector of PSM1 joint position
	// std::stringstream qPSM2SS;				//!< 6x1 Vector of PSM2 joint position
	// std::stringstream Tb1gSS;				//!< Homogeneous transformation matrix expressing the pose of the left gripper (PSM1) wrt the base frame of the PSM1
	// std::stringstream Tb2gSS;				//!< Homogeneous transformation matrix expressing the pose of the right gripper (PSM2) wrt the base frame of the PSM2
	// std::stringstream vb1gSS;				//!< Velocity vector of the left gripper (PSM1) expressed wrt base frame of the PSM1
	// std::stringstream vb2gSS;				//!< Velocity vector of the right gripper (PSM2) expressed wrt base frame of the PSM2

	// Signals
	// std::stringstream clutchButtonSS;
	// std::stringstream holdhButtonSS;
	// std::stringstream leftRaisingClutchEdgeSS;
	// std::stringstream rightRaisingClutchEdgeSS;
	// std::stringstream leftTrailingClutchEdgeSS;
	// std::stringstream rightTrailingClutchEdgeSS;
	// std::stringstream leftTriggerClutchButtonSS;
	// std::stringstream rightTriggerClutchButtonSS;


};


#endif // GEOMAGICPROXY_HPP_
