/** GeomagicProxy.cpp
 * 
 * This code is cloned from https://github.com/marcofer/portable-dvrk-simulator.git
 * and modified by Songjie Xiao (songjiexiao@zju.edu.cn) mainly for academic use.
 * 
 */

// Project Header files
#include "GeomagicProxy.hpp"

/**
* @brief Default contructor of GeomagicProxy class
*
*/
GeomagicProxy::GeomagicProxy(GeomagicStatus* state) {

	// Geomagic stringstream
	// leftHipBaseVelSS.str("");		//!< 6D Velocity of the left hip with respect to the base frame of the left geomagic
	// rightHipBaseVelSS.str("");	//!< 6D Velocity of the right hip with respect to the base frame of the right geomagic
	// leftHipPSMVelSS.str("");		//!< 6D Velocity of the left hip with respect to the base frame of the Left PSM
	// rightHipPSMVelSS.str("");	//!< 6D Velocity of the right hip with respect to the base frame of the Left PSM
	// leftHipRotSS.str("");			//!< Rotation matrix expressing the orientation of the left hip wrt the base frame of the left geomagic
	// rightHipRotSS.str("");		//!< Rotation matrix expressing the orientation of the left hip wrt the base frame of the right geomagic
	// leftHipPosSS.str("");			//!< Position of the left hip wrt the base frame of the left geomagic
	// rightHipPosSS.str("");		//!< Position of the right hip wrt the base frame of the right geomagic

	// PSM stringstream
	// qdotPSM1SS.str("");			//!< 6x1 Vector of PSM1 joint velocity
	// qdotPSM2SS.str("");			//!< 6x1 Vector of PSM1 joint position
	// qPSM1SS.str("");				//!< 6x1 Vector of PSM2 joint velocity
	// qPSM2SS.str("");				//!< 6x1 Vector of PSM2 joint position
	// Tb1gSS.str("");				//!< Homogeneous transformation matrix expressing the pose of the left gripper (PSM1) wrt the base frame of the PSM1
	// Tb2gSS.str("");				//!< Homogeneous transformation matrix expressing the pose of the right gripper (PSM2) wrt the base frame of the PSM2
	// vb1gSS.str("");				//!< Velocity vector of the left gripper (PSM1) expressed wrt base frame of the PSM1
	// vb2gSS.str("");				//!< Velocity vector of the right gripper (PSM2) expressed wrt base frame of the PSM2

	// Signals
	// clutchButtonSS.str("");
	// holdhButtonSS.str("");
	// leftRaisingClutchEdgeSS.str("");
	// rightRaisingClutchEdgeSS.str("");
	// leftTrailingClutchEdgeSS.str("");
	// rightTrailingClutchEdgeSS.str("");
	// leftTriggerClutchButtonSS.str("");
	// rightTriggerClutchButtonSS.str("");
	// Initialize all the structures to zero

	geoStatus = state;
	/* Cartesian space values */      
	this->geoStatus->stylusPosition.set(0.0, 0.0, 0.0);
	this->geoStatus->stylusOrientation = hduQuaternion(1.0, hduVector3Dd(0.0, 0.0, 0.0));
	this->geoStatus->stylusLinearVelocity.set(0.0, 0.0, 0.0);
	this->geoStatus->stylusAngularVelocity.set(0.0, 0.0, 0.0);
	std::memset(this->geoStatus->jacobian, 0.0, sizeof(double) * (SPACE_DIM * 2) * GEOMAGIC_HAPTIC_JOINTS);

	// Test
	this->geoStatus->stylusLinearVelocity2.set(0.0, 0.0, 0.0);
	this->geoStatus->stylusLinearVelocityjac.set(0.0, 0.0, 0.0);
	this->geoStatus->stylusAngularVelocityjac.set(0.0, 0.0, 0.0);


	/* Joint space values */
	this->geoStatus->PosAngles.set(0.0, 0.0, 0.0);
	this->geoStatus->GimbalAngles.set(0.0, 0.0, 0.0);
	this->geoStatus->force.set(0.0, 0.0, 0.0);
	std::memset(this->geoStatus->jointPosition, 0.0, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);
	std::memset(this->geoStatus->jointVelocity, 0.0, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);

	// Initialize variables related to the buttons state
	this->geoStatus->stylusButtons = 0;
	this->geoStatus->action[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus->action[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus->evHoldButton[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus->evHoldButton[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus->evRaiseEdge[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus->evRaiseEdge[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus->evTrailEdge[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus->evTrailEdge[GEOMAGIC_HIGH_BUTTON] = false;


	// Utility data
	// linear history position/velocity
	this->hdUtils.prvPos.set(0.0, 0.0, 0.0);
	this->hdUtils.lstPos.set(0.0, 0.0, 0.0);

	this->hdUtils.prvInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils.prvOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstOutVel.set(0.0, 0.0, 0.0);

	this->hdUtils.lvelocity.set(0.0, 0.0, 0.0);
	this->hdUtils.lvelocityTemp.set(0.0, 0.0, 0.0);

	// history pos joint value/velocity
	this->hdUtils.prvAng.set(0.0, 0.0, 0.0);
	this->hdUtils.lstAng.set(0.0, 0.0, 0.0);

	this->hdUtils.prvInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.prvOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstOutAngVel.set(0.0, 0.0, 0.0);

	this->hdUtils.avelocity.set(0.0, 0.0, 0.0);
	this->hdUtils.avelocityTemp.set(0.0, 0.0, 0.0);

	// history gimbal joint value/velocity
	this->hdUtils.prvAng0.set(0.0, 0.0, 0.0);
	this->hdUtils.lstAng0.set(0.0, 0.0, 0.0);

	this->hdUtils.prvInputAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils.lstInputAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstInputAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils.prvOutAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils.lstOutAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstOutAngVel0.set(0.0, 0.0, 0.0);

	this->hdUtils.avelocity0.set(0.0, 0.0, 0.0);
	this->hdUtils.avelocityTemp0.set(0.0, 0.0, 0.0);

	// std::memset(this->hdUtils.jointPosPrev, 0.0, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);

	this->availability(true);

}

/**
* @brief Default destroyer of GeomagicProxy class
*/
GeomagicProxy::~GeomagicProxy() {}

/**
* @brief New Calibration function 
* Calibrate the Geomagic device
* @return true if the device has been successfully calibrated
*/
void GeomagicProxy::HHD_Auto_Calibration(){
	int supportedCalibrationStyles;
	int calibrationStyle;
    HDErrorInfo error;

    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		std::cout << "HD_CALIBRATION_ENCODER_RESET.." << std::endl;
    }
	if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
        calibrationStyle = HD_CALIBRATION_INKWELL;
		std::cout << "HD_CALIBRATION_INKWELL.." << std::endl;
    }
	if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
        calibrationStyle = HD_CALIBRATION_AUTO;
		std::cout << "HD_CALIBRATION_AUTO.." << std::endl;
    }
    if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
        do {
            hdUpdateCalibration(calibrationStyle);
			std::cout << "Calibrating.. (put stylus in well)" << std::endl;
            if (HD_DEVICE_ERROR(error = hdGetError())) {
                hduPrintError(stderr, &error, "Reset encoders reset failed.");
                break;
            }
        } while (hdCheckCalibration() != HD_CALIBRATION_OK);
		std::cout << "Calibration complete." << std::endl;
    }

    while(hdCheckCalibration() != HD_CALIBRATION_OK) {
        usleep(1e6);
        if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
			std::cout << "Please place the device into the inkwell for calibration" << std::endl;
        else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
			std::cout << "Calibration updated successfully" << std::endl;
            hdUpdateCalibration(calibrationStyle);
        }
        else{
			std::cout << "Unknown calibration status" << std::endl;
		}
    }
}

/**
* @brief Event catch function
* Catch the event when the input button has been pressed, computed on the consecutive raising and trailing edges of the button pressing state
* @param button: the current state of the pressed button
* @param button_prev: the previous state of the pressed button
* @param raise: the raising edge of the event
* @param trail: the trailing edge of the event
* @param trigger: the boolean value to be returned
*/
void GeomagicProxy::catchButtonPressEvent(const bool& button, bool& button_prev, bool& raise, bool& trail, bool& trigger) {


	if (button && !button_prev) {
		raise = true;
	}
	if (!button && button_prev) {
		trail = true;
	}
	if (raise && trail) {

		trigger = !trigger;
		raise = false;
		trail = false;
	}

	button_prev = button;

}


/**
* @brief Default run function
*/
void GeomagicProxy::run() {
    ////////////////////////////////////////////////////////////////
    // Init Phantom
    ////////////////////////////////////////////////////////////////
	HDErrorInfo error;
	this->dvcHandle = hdInitDevice(HD_MY_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		std::cout << "Failed to initialize haptic device. Code: 0x0" << std::endl;
		this->availability(false);
		return;
	}
	std::cout << "Found Device: " << hdGetString(HD_DEVICE_MODEL_TYPE) << std::endl;

	// Calibrate the device
	HHD_Auto_Calibration();

	//hdDisable(HD_SOFTWARE_VELOCITY_LIMIT);

	// Set running on true
	this->setRunning(true);

	std::cout << "Touch control loop started!\n";

	// Set the force feedback callback in the scheduler
	this->schHandle = hdScheduleAsynchronous(forceFeedbackCallback, this, HD_MAX_SCHEDULER_PRIORITY);

	// Enable force feedback
	hdEnable(HD_FORCE_OUTPUT);

	// Start the scheduler
	//hdSetSchedulerRate(800);
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		std::cout << "Failed to start scheduler" << std::endl;
	}

	// Run the main loop
	while (this->isRunning())
	{
		//----------------------------------------------------------------//
		// Do stuff here... 

		while (!hdWaitForCompletion(schHandle, HD_WAIT_CHECK_STATUS));
		hdScheduleSynchronous(updateGeoStateCallback, this, HD_DEFAULT_SCHEDULER_PRIORITY);

		//----------------------------------------------------------------//

	}

	// Stop the scheduler
	hdStopScheduler();

	// Unschedule the tasks
	hdUnschedule(schHandle);

	// Disable the device
	hdDisableDevice(dvcHandle);

}

/**
* @brief Default stop function
*/
void GeomagicProxy::stop() {

	// Set running on false
	this->setRunning(false);

}

/**
* @brief Callback function
* Update the state of the Geomagic device
* @param data the data containing the updated status
*/
HDCallbackCode updateGeoStateCallback(void* data) {

	GeomagicProxy* device = (GeomagicProxy*)data;
	Eigen::Matrix<double, (SPACE_DIM * 2), GEOMAGIC_HAPTIC_JOINTS> J;
	Eigen::Matrix<double, (SPACE_DIM * 2), 1> vel;
	Eigen::Matrix<double, GEOMAGIC_HAPTIC_JOINTS, 1> qdot;
	bool curButton[GEOMAGIC_BUTTONS_NUM];
	double hdrate;

    hduMatrix transform_ref;

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Failed");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else {
		// hdrate varies with time 700-900
		hdGetDoublev(HD_UPDATE_RATE, &hdrate);

		// Joint space values
		hdGetDoublev(HD_CURRENT_JOINT_ANGLES, device->geoStatus->PosAngles);
		hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, device->geoStatus->GimbalAngles);
		for (int i=0; i<3; i++){
			device->geoStatus->jointPosition[i] = device->geoStatus->PosAngles[i];
			device->geoStatus->jointPosition[i+3] = device->geoStatus->GimbalAngles[i];
		}
		//update joint velocity vector
		device->updateJointVelocities();
		for (int i=0; i<3; i++){
			device->geoStatus->jointVelocity[i] = device->geoStatus->PosAnglesVel[i];
			device->geoStatus->jointVelocity[i+3] = device->geoStatus->GimbalAnglesVel[i];
		}

		// Cartesian space values
		// hdGetDoublev(HD_CURRENT_POSITION, device->geoStatus->stylusPosition);
		hdGetDoublev(HD_CURRENT_TRANSFORM, transform_ref);
		device->geoStatus->stylusPosition = hduVector3Dd(transform_ref[3][0], transform_ref[3][1], transform_ref[3][2]);
		transform_ref.getRotation(device->geoStatus->stylusOrientation);
		// Convert from [mm] to [m]
		device->geoStatus->stylusPosition *= 1e-3;
		
		// TODO:Update the velocity vector
		device->updateVelocities();

		hdGetDoublev(HD_CURRENT_VELOCITY, device->geoStatus->stylusLinearVelocity2);
		device->geoStatus->stylusLinearVelocity2 *= 1e-3;
		// hd api coule not get the angular velocity
		// hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, device->geoStatus->stylusAngularVelocity);

		hdGetDoublev(HD_CURRENT_JACOBIAN, device->geoStatus->jacobian);
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				J(i, j) = device->geoStatus->jacobian[i * 6 + j];
			}
		}

		for (int i=0; i<6; i++){
			qdot[i] = device->geoStatus->jointVelocity[i];
		}
		vel = J * qdot;
		// if (vel.norm() > 1e3) vel.setZero();
		device->geoStatus->stylusLinearVelocityjac = hduVector3Dd(vel(0), vel(1), vel(2));
		device->geoStatus->stylusAngularVelocityjac = hduVector3Dd(vel(3), vel(4), vel(5));
		// std::memcpy(device->hdUtils.jointPosPrev,device->geoStatus->jointPosition,sizeof(double)*HAPTIC_JOINTS);


		// std::memcpy(q.data(), device->geoStatus->jointPosition, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);
		// std::memcpy(qprev.data(), device->hdUtils.jointPosPrev, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);



		// Button state
		hdGetIntegerv(HD_CURRENT_BUTTONS, &(device->geoStatus->stylusButtons));

		for (int i = 0; i < GEOMAGIC_BUTTONS_NUM; i++) {
			curButton[i] = (device->geoStatus->stylusButtons == i + 1 || device->geoStatus->stylusButtons == PRESSED_BOTH) ? true : false;

			device->catchButtonPressEvent(curButton[i], device->geoStatus->evHoldButton[i],
				device->geoStatus->evRaiseEdge[i],
				device->geoStatus->evTrailEdge[i],
				device->geoStatus->action[i]);
		}
	}

	return HD_CALLBACK_DONE;
}


/**
* @brief Callback function
* Force feedback callback of the Geomagic device
* @param data the data containing the force feedback data
*/
HDCallbackCode forceFeedbackCallback(void* data) {

	GeomagicProxy* device = (GeomagicProxy*)data;
	// GeomagicProxy* device = static_cast<GeomagicProxy *>(data);

	device->dvcHandle = hdGetCurrentDevice();
	HDErrorInfo error;
	hdBeginFrame(device->dvcHandle);

	hdSetDoublev(HD_CURRENT_FORCE, device->geoStatus->force);
	hdEndFrame(device->dvcHandle);

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Error during force scheduler callback");
		device->availability(false);
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else { device->availability(true); }
	return HD_CALLBACK_CONTINUE;
}


/**
* @brief Update function
* Update the linear and angular velocities of the Geomagic stylus
* Set internally stylusLinearVelocity and stylusAngularVelocity
*/
void GeomagicProxy::updateVelocities() 
{
	hduVector3Dd vel_buff(0.0, 0.0, 0.0);

	// Compute linear velocities
	vel_buff = (this->geoStatus->stylusPosition * 3 - 4 * hdUtils.prvPos + hdUtils.lstPos) / 0.002; //mm/s, 2nd order backward dif
	hdUtils.lvelocity = (.2196 * (vel_buff + hdUtils.vrLstInputVel) + .6588 * (hdUtils.prvInputVel + hdUtils.lstInputVel)) / 1000.0 - (-2.7488 * hdUtils.prvOutVel + 2.5282 * hdUtils.lstOutVel - 0.7776 * hdUtils.vrLstOutVel); //cutoff freq of 20 Hz
	hdUtils.lstPos = hdUtils.prvPos;
	hdUtils.prvPos = this->geoStatus->stylusPosition;
	hdUtils.vrLstInputVel = hdUtils.lstInputVel;
	hdUtils.lstInputVel = hdUtils.prvInputVel;
	hdUtils.prvInputVel = vel_buff;
	hdUtils.vrLstOutVel = hdUtils.lstOutVel;
	hdUtils.lstOutVel = hdUtils.prvOutVel;
	hdUtils.prvOutVel = hdUtils.lvelocity;
	hdUtils.lvelocityTemp = hdUtils.lvelocityTemp + ((hdUtils.lvelocity - hdUtils.lvelocityTemp) * (0.001 / (0.001 + 0.07)));

	this->geoStatus->stylusLinearVelocity = hdUtils.lvelocityTemp;


	// Compute angular velocities
	// TODO: update angular velocity
	vel_buff.set(0.0, 0.0, 0.0);
	vel_buff = (this->geoStatus->GimbalAngles * 3 - 4 * hdUtils.prvAng + hdUtils.lstAng) / 0.002; //mm/s, 2nd order backward dif
	hdUtils.avelocity = (.2196 * (vel_buff + hdUtils.vrLstInputAngVel) + .6588 * (hdUtils.prvInputAngVel + hdUtils.lstInputAngVel)) / 1000.0 - (-2.7488 * hdUtils.prvOutAngVel + 2.5282 * hdUtils.lstOutAngVel - 0.7776 * hdUtils.vrLstOutAngVel); //cutoff freq of 20 Hz
	hdUtils.lstAng = hdUtils.prvAng;
	hdUtils.prvAng = this->geoStatus->GimbalAngles;
	hdUtils.vrLstInputAngVel = hdUtils.lstInputAngVel;
	hdUtils.lstInputAngVel = hdUtils.prvInputAngVel;
	hdUtils.prvInputAngVel = vel_buff;
	hdUtils.vrLstOutAngVel = hdUtils.lstOutAngVel;
	hdUtils.lstOutAngVel = hdUtils.prvOutAngVel;
	hdUtils.prvOutAngVel = hdUtils.avelocity;
	hdUtils.avelocityTemp = hdUtils.avelocityTemp + ((hdUtils.avelocity - hdUtils.avelocityTemp) * (0.001 / (0.001 + 0.07)));

	this->geoStatus->stylusAngularVelocity = hdUtils.avelocityTemp;

}

/**
* @brief Update function
* Update the Joint Velocity of the Geomagic stylus
* Set internally jointVelocity
*/
void GeomagicProxy::updateJointVelocities()
{
	hduVector3Dd vel_buff(0.0, 0.0, 0.0);

	// Compute pos joint velocities
	vel_buff.set(0.0, 0.0, 0.0);
	vel_buff = (this->geoStatus->PosAngles * 3 - 4 * hdUtils.prvAng0 + hdUtils.lstAng0) / 0.002; //mm/s, 2nd order backward dif
	hdUtils.avelocity0 = (.2196 * (vel_buff + hdUtils.vrLstInputAngVel0) + .6588 * (hdUtils.prvInputAngVel0 + hdUtils.lstInputAngVel0)) / 1000.0 
	                  - (-2.7488 * hdUtils.prvOutAngVel0 + 2.5282 * hdUtils.lstOutAngVel0 - 0.7776 * hdUtils.vrLstOutAngVel0); //cutoff freq of 20 Hz
	hdUtils.lstAng0 = hdUtils.prvAng0;
	hdUtils.prvAng0 = this->geoStatus->PosAngles;
	hdUtils.vrLstInputAngVel0 = hdUtils.lstInputAngVel0;
	hdUtils.lstInputAngVel0 = hdUtils.prvInputAngVel0;
	hdUtils.prvInputAngVel0 = vel_buff;
	hdUtils.vrLstOutAngVel0 = hdUtils.lstOutAngVel0;
	hdUtils.lstOutAngVel0 = hdUtils.prvOutAngVel0;
	hdUtils.prvOutAngVel0 = hdUtils.avelocity0;
	hdUtils.avelocityTemp0 = hdUtils.avelocityTemp0 + ((hdUtils.avelocity0 - hdUtils.avelocityTemp0) * (0.001 / (0.001 + 0.07)));

	this->geoStatus->PosAnglesVel = hdUtils.avelocityTemp0;	

	// Compute gimbal joint velocities
	vel_buff.set(0.0, 0.0, 0.0);
	vel_buff = (this->geoStatus->GimbalAngles * 3 - 4 * hdUtils.prvAng + hdUtils.lstAng) / 0.002; //mm/s, 2nd order backward dif
	hdUtils.avelocity = (.2196 * (vel_buff + hdUtils.vrLstInputAngVel) + .6588 * (hdUtils.prvInputAngVel + hdUtils.lstInputAngVel)) / 1000.0 
	                  - (-2.7488 * hdUtils.prvOutAngVel + 2.5282 * hdUtils.lstOutAngVel - 0.7776 * hdUtils.vrLstOutAngVel); //cutoff freq of 20 Hz
	hdUtils.lstAng = hdUtils.prvAng;
	hdUtils.prvAng = this->geoStatus->GimbalAngles;
	hdUtils.vrLstInputAngVel = hdUtils.lstInputAngVel;
	hdUtils.lstInputAngVel = hdUtils.prvInputAngVel;
	hdUtils.prvInputAngVel = vel_buff;
	hdUtils.vrLstOutAngVel = hdUtils.lstOutAngVel;
	hdUtils.lstOutAngVel = hdUtils.prvOutAngVel;
	hdUtils.prvOutAngVel = hdUtils.avelocity;
	hdUtils.avelocityTemp = hdUtils.avelocityTemp + ((hdUtils.avelocity - hdUtils.avelocityTemp) * (0.001 / (0.001 + 0.07)));

	this->geoStatus->GimbalAnglesVel = hdUtils.avelocityTemp;

}


/**
* @brief Set function
* Set the feedback haptic force on the Geomagic device
* @param f: the feedback force to be set
*/
void GeomagicProxy::setHapticForce(const Eigen::VectorXf& f) {

	for (int i = 0; i < GEOMAGIC_HAPTIC_DOF; i++) {
		this->geoStatus->force[i] = f(i);
	}
}

/**
* @brief Set function
* Set the 3D vector of the hip position of the device
* @param p: the 3D vector of the hip position of the device
*/
void GeomagicProxy::setHIPPosition(const Eigen::Vector3f& p) {

	for (int i = 0; i < SPACE_DIM; i++) {
		this->geoStatus->stylusPosition[i] = p(i);
	}

}


/**
* @brief Set function
* Set the 3D vector of the hip orientation of the device
* @param p: the 3D vector of the hip orientation of the device
*/
void GeomagicProxy::setHIPGimbalAngles(const Eigen::Vector3f& ga) {

	for (int i = 0; i < SPACE_DIM; i++) {
		this->geoStatus->GimbalAngles[i] = ga(i);
	}

}

/**
* @brief Set function
* Set the 6D vector of the hip velocity of the device
* @param v: the 6D vector of the hip velocity of the device
*/
void GeomagicProxy::setHIPVelocity(const Eigen::Vector6f& v) {

	for (int i = 0; i < SPACE_DIM; i++) {
		this->geoStatus->stylusLinearVelocity[i] = v(i);
		this->geoStatus->stylusAngularVelocity[i] = v(i + SPACE_DIM);
	}

}


/**
* @brief Set function
* Set the dynamic array with the state (on/off) of the buttons placed on the device
* @param state: the dynamic array with the state (on/off) of the buttons placed on the device
*/
void GeomagicProxy::setButtonState(const bool* state) {

	this->geoStatus->action[GEOMAGIC_LOW_BUTTON] = state[GEOMAGIC_LOW_BUTTON];
	this->geoStatus->action[GEOMAGIC_HIGH_BUTTON] = state[GEOMAGIC_HIGH_BUTTON];

}

/**
* @brief Get function
* Get the feedback haptic force on the Geomagic device
* @return the feedback force
*/
Eigen::VectorXf GeomagicProxy::getHapticForce() {

	Eigen::VectorXf ret;
	ret.setZero(GEOMAGIC_HAPTIC_DOF);

	for (int i = 0; i < GEOMAGIC_HAPTIC_DOF; i++) {
		ret(i) = this->geoStatus->force[i];
	}

	return ret;

}

/**
* @brief Get function
* Get the 3D vector of the hip position of the device
* @return the 3D vector of the hip position of the device
*/
Eigen::Vector3f GeomagicProxy::getHIPPosition() {

	Eigen::Vector3f ret;

	for (int i = 0; i < SPACE_DIM; i++) {
		ret(i) = this->geoStatus->stylusPosition[i];
	}

	return ret;

}

/**
* @brief Get function
* Get the 3D vector of the hip orientation of the device
* @return the 3D vector of the hip orientation of the device
*/
Eigen::Vector3f GeomagicProxy::getHIPGimbalAngles() {

	Eigen::Vector3f ret;

	for (int i = 0; i < SPACE_DIM; i++) {
		ret(i) = this->geoStatus->GimbalAngles[i];
	}

	return ret;

}

/**
* @brief Get function
* Get the 6D velocity of the HIP
* @return the 6D velocity of the HIP
*/
Eigen::Vector6f GeomagicProxy::getHIPVelocity() {

	Eigen::Vector6f ret;

	for (int i = 0; i < SPACE_DIM; i++) {
		ret(i) = this->geoStatus->stylusLinearVelocity[i];
		ret(i + SPACE_DIM) = this->geoStatus->stylusAngularVelocity[i];
	}

	return ret;
}


/**
* @brief Get function
* Get the dynamic array with the state (on/off) of the buttons placed on the device
* @return the dynamic array with the state (on/off) of the buttons placed on the device
*/
bool* GeomagicProxy::getButtonState() {

	return (this->geoStatus->action);

}

/*
void GeomagicProxy::saveLog() {

	// Geomagic stringstream
	std::string leftHipBaseVelFile = logPath + "leftHipBaseVel.txt";
	std::string rightHipBaseVelFile = logPath + "rightHipBaseVel.txt";
	std::string leftHipPSMVelFile = logPath + "leftHipPSMVel.txt";
	std::string rightHipPSMVelFile = logPath + "righttHipPSMVel.txt";
	std::string leftHipRotFile = logPath + "leftHipRot.txt";
	std::string rightHipRotFile = logPath + "rightHipRot.txt";
	std::string leftHipPosFile = logPath + "leftHipPos.txt";
	std::string rightHipPosFile = logPath + "rightHipPos.txt";

	// PSM stringstream
	std::string qdotPSM1File = logPath + "qdotPSM1.txt";
	std::string qdotPSM2File = logPath + "qdotPSM2.txt";
	std::string qPSM1File = logPath + "qPSM1.txt";
	std::string qPSM2File = logPath + "qPSM2.txt";
	std::string Tb1gFile = logPath + "Tb1g.txT";
	std::string Tb2gFile = logPath + "Tb2g.TXT";
	std::string vb1gFile = logPath + "vb1g.txt";
	std::string vb2gFile = logPath + "vb2g.txt";

	// Signals
	std::string clutchButtonFile = logPath + "clutchButton.txt";
	std::string holdhButtonFile = logPath + "holdhButton.txt";
	std::string leftRaisingClutchEdgeFile = logPath + "leftRaisingClutchEdge.txt";
	std::string rightRaisingClutchEdgeFile = logPath + "rightRaisingClutchEdge.txt";
	std::string leftTrailingClutchEdgeFile = logPath + "leftTrailingClutchEdge.txt";
	std::string rightTrailingClutchEdgeFile = logPath + "rightTrailingClutchEdge.txt";
	std::string leftTriggerClutchButtonFile = logPath + "leftTriggerClutchButton.txt";
	std::string rightTriggerClutchButtonFile = logPath + "rightTriggerClutchButton.txt";

	saveToFile(leftHipBaseVelSS, leftHipBaseVelFile);
	saveToFile(rightHipBaseVelSS, rightHipBaseVelFile);
	saveToFile(leftHipPSMVelSS, leftHipPSMVelFile);
	saveToFile(rightHipPSMVelSS, rightHipPSMVelFile);
	saveToFile(leftHipPosSS, leftHipPosFile);
	saveToFile(rightHipPosSS, rightHipPosFile);

	saveToFile(qPSM1SS, qPSM1File);
	saveToFile(qPSM2SS, qPSM2File);
	saveToFile(qdotPSM1SS, qdotPSM1File);
	saveToFile(qdotPSM2SS, qdotPSM2File);
	saveToFile(vb1gSS, vb1gFile);
	saveToFile(vb2gSS, vb2gFile);

}
*/