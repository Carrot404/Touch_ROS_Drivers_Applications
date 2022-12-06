/** GeomagicProxy.cpp
 * 
 * This code is cloned from https://github.com/marcofer/portable-dvrk-simulator.git
 * and modified by Songjie Xiao (songjiexiao@zju.edu.cn) mainly for academic use.
 * 
 */

// Project Header files
#include "GeomagicProxy.h"

/**
* @brief Default contructor of GeomagicProxy class
*
*/
GeomagicProxy::GeomagicProxy() 
{
	/* Cartesian space values */
	this->geoStatus_ = std::make_shared<GeomagicStatus>();
	this->geoStatus_->stylusPosition.set(0.0, 0.0, 0.0);
	this->geoStatus_->stylusOrientation = hduQuaternion(1.0, hduVector3Dd(0.0, 0.0, 0.0));
	this->geoStatus_->stylusLinearVelocity.set(0.0, 0.0, 0.0);
	this->geoStatus_->stylusAngularVelocity.set(0.0, 0.0, 0.0);
	std::memset(this->geoStatus_->jacobian, 0.0, sizeof(double) * (SPACE_DIM * 2) * GEOMAGIC_HAPTIC_JOINTS);
	this->geoStatus_->force.set(0.0, 0.0, 0.0);

	/* Joint space values */
	this->geoStatus_->PosAngles.set(0.0, 0.0, 0.0);
	this->geoStatus_->GimbalAngles.set(0.0, 0.0, 0.0);
	this->geoStatus_->effort.set(0.0, 0.0, 0.0);
	std::memset(this->geoStatus_->jointPosition, 0.0, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);
	std::memset(this->geoStatus_->jointVelocity, 0.0, sizeof(double) * GEOMAGIC_HAPTIC_JOINTS);

	// Initialize variables related to the buttons state
	this->geoStatus_->buttons[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->buttons[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus_->buttons_prev[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->buttons_prev[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus_->action[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->action[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus_->evHoldButton[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->evHoldButton[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus_->evRaiseEdge[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->evRaiseEdge[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus_->evTrailEdge[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->evTrailEdge[GEOMAGIC_HIGH_BUTTON] = false;


	// Utility data
	// linear history position/velocity
	this->hdUtils_.prvPos.set(0.0, 0.0, 0.0);
	this->hdUtils_.lstPos.set(0.0, 0.0, 0.0);

	this->hdUtils_.prvInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils_.lstInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils_.vrLstInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils_.prvOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils_.lstOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils_.vrLstOutVel.set(0.0, 0.0, 0.0);

	this->hdUtils_.lvelocity.set(0.0, 0.0, 0.0);
	this->hdUtils_.lvelocityTemp.set(0.0, 0.0, 0.0);

	// history pos joint value/velocity
	this->hdUtils_.prvAng.set(0.0, 0.0, 0.0);
	this->hdUtils_.lstAng.set(0.0, 0.0, 0.0);

	this->hdUtils_.prvInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils_.lstInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils_.vrLstInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils_.prvOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils_.lstOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils_.vrLstOutAngVel.set(0.0, 0.0, 0.0);

	this->hdUtils_.avelocity.set(0.0, 0.0, 0.0);
	this->hdUtils_.avelocityTemp.set(0.0, 0.0, 0.0);

	// history gimbal joint value/velocity
	this->hdUtils_.prvAng0.set(0.0, 0.0, 0.0);
	this->hdUtils_.lstAng0.set(0.0, 0.0, 0.0);

	this->hdUtils_.prvInputAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils_.lstInputAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils_.vrLstInputAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils_.prvOutAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils_.lstOutAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils_.vrLstOutAngVel0.set(0.0, 0.0, 0.0);

	this->hdUtils_.avelocity0.set(0.0, 0.0, 0.0);
	this->hdUtils_.avelocityTemp0.set(0.0, 0.0, 0.0);

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
	this->dvcHandle_ = hdInitDevice(HD_MY_DEVICE);
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
	this->schHandle_ = hdScheduleAsynchronous(forceFeedbackCallback, this, HD_MAX_SCHEDULER_PRIORITY);

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

		while (!hdWaitForCompletion(schHandle_, HD_WAIT_CHECK_STATUS));
		hdScheduleSynchronous(updateGeoStateCallback, this, HD_DEFAULT_SCHEDULER_PRIORITY);

		//----------------------------------------------------------------//

	}

	// Stop the scheduler
	hdStopScheduler();

	// Unschedule the tasks
	hdUnschedule(schHandle_);

	// Disable the device
	hdDisableDevice(dvcHandle_);

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
	int stylusButtons;

    hduMatrix transform_ref;

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Failed");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else {
		// hdrate varies with time 700-900
		hdGetDoublev(HD_UPDATE_RATE, &device->hdrate_);

		// Joint space values
		hdGetDoublev(HD_CURRENT_JOINT_ANGLES, device->geoStatus_->jointPosition);
		hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, device->geoStatus_->jointPosition+3);
		hdGetDoublev(HD_CURRENT_JOINT_TORQUE, device->geoStatus_->effort);

		device->geoStatus_->PosAngles[0] = -device->geoStatus_->jointPosition[0];
		device->geoStatus_->PosAngles[1] = device->geoStatus_->jointPosition[1];
		device->geoStatus_->PosAngles[2] = device->geoStatus_->jointPosition[2]-device->geoStatus_->jointPosition[1];
		device->geoStatus_->GimbalAngles[0] = -device->geoStatus_->jointPosition[3];
		device->geoStatus_->GimbalAngles[1] = device->geoStatus_->jointPosition[4]+1.49;
		device->geoStatus_->GimbalAngles[2] = device->geoStatus_->jointPosition[5];

		// update joint velocity vector
		device->updateJointVelocities();
		for (int i=0; i<3; i++){
			device->geoStatus_->jointVelocity[i] = device->geoStatus_->PosAnglesVel[i];
			device->geoStatus_->jointVelocity[i+3] = device->geoStatus_->GimbalAnglesVel[i];
		}

		// Cartesian space values
		// hdGetDoublev(HD_CURRENT_POSITION, device->geoStatus_->stylusPosition);
		hdGetDoublev(HD_CURRENT_TRANSFORM, transform_ref);
		device->geoStatus_->stylusPosition = hduVector3Dd(transform_ref[3][0], transform_ref[3][1], transform_ref[3][2]);
		transform_ref.getRotation(device->geoStatus_->stylusOrientation);
		// Convert from [mm] to [m]
		device->geoStatus_->stylusPosition *= 1e-3;
		
		// TODO:Update the velocity vector
		device->updateVelocities();

		// hdGetDoublev(HD_CURRENT_VELOCITY, device->geoStatus_->stylusLinearVelocity);
		// device->geoStatus_->stylusLinearVelocity *= 1e-3;
		// hd api coule not get the angular velocity
		// hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, device->geoStatus_->stylusAngularVelocity);

		// 
		hdGetDoublev(HD_CURRENT_JACOBIAN, device->geoStatus_->jacobian);
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				J(i, j) = device->geoStatus_->jacobian[i * 6 + j];
			}
		}

		// Test for jacobian linear vel should be divide by 1000?
		// for (int i=0; i<6; i++){
		// 	qdot[i] = device->geoStatus_->jointVelocity[i];
		// }
		// vel = J * qdot;
		// if (vel.norm() > 1e3) vel.setZero();
		// device->geoStatus_->stylusLinearVelocityjac = hduVector3Dd(vel(0), vel(1), vel(2));
		// device->geoStatus_->stylusAngularVelocityjac = hduVector3Dd(vel(3), vel(4), vel(5));

		// Button state
		hdGetIntegerv(HD_CURRENT_BUTTONS, &stylusButtons);

		for (int i = 0; i < GEOMAGIC_BUTTONS_NUM; i++) {
			device->geoStatus_->buttons[i] = (stylusButtons == i + 1 || stylusButtons == PRESSED_BOTH) ? true : false;

			device->catchButtonPressEvent(device->geoStatus_->buttons[i], device->geoStatus_->evHoldButton[i],
				device->geoStatus_->evRaiseEdge[i],
				device->geoStatus_->evTrailEdge[i],
				device->geoStatus_->action[i]);
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

	device->dvcHandle_ = hdGetCurrentDevice();
	HDErrorInfo error;
	hdBeginFrame(device->dvcHandle_);

	hdSetDoublev(HD_CURRENT_FORCE, device->geoStatus_->force);
	hdEndFrame(device->dvcHandle_);

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
	vel_buff = (this->geoStatus_->stylusPosition * 3 - 4 * hdUtils_.prvPos + hdUtils_.lstPos) / 0.002; //mm/s, 2nd order backward dif
	hdUtils_.lvelocity = (.2196 * (vel_buff + hdUtils_.vrLstInputVel) + .6588 * (hdUtils_.prvInputVel + hdUtils_.lstInputVel)) / 1000.0 - (-2.7488 * hdUtils_.prvOutVel + 2.5282 * hdUtils_.lstOutVel - 0.7776 * hdUtils_.vrLstOutVel); //cutoff freq of 20 Hz
	hdUtils_.lstPos = hdUtils_.prvPos;
	hdUtils_.prvPos = this->geoStatus_->stylusPosition;
	hdUtils_.vrLstInputVel = hdUtils_.lstInputVel;
	hdUtils_.lstInputVel = hdUtils_.prvInputVel;
	hdUtils_.prvInputVel = vel_buff;
	hdUtils_.vrLstOutVel = hdUtils_.lstOutVel;
	hdUtils_.lstOutVel = hdUtils_.prvOutVel;
	hdUtils_.prvOutVel = hdUtils_.lvelocity;
	hdUtils_.lvelocityTemp = hdUtils_.lvelocityTemp + ((hdUtils_.lvelocity - hdUtils_.lvelocityTemp) * (0.001 / (0.001 + 0.07)));

	this->geoStatus_->stylusLinearVelocity = hdUtils_.lvelocityTemp;


	// Compute angular velocities
	// TODO: update angular velocity
	vel_buff.set(0.0, 0.0, 0.0);
	vel_buff = (this->geoStatus_->GimbalAngles * 3 - 4 * hdUtils_.prvAng + hdUtils_.lstAng) / 0.002; //mm/s, 2nd order backward dif
	hdUtils_.avelocity = (.2196 * (vel_buff + hdUtils_.vrLstInputAngVel) + .6588 * (hdUtils_.prvInputAngVel + hdUtils_.lstInputAngVel)) / 1000.0 - (-2.7488 * hdUtils_.prvOutAngVel + 2.5282 * hdUtils_.lstOutAngVel - 0.7776 * hdUtils_.vrLstOutAngVel); //cutoff freq of 20 Hz
	hdUtils_.lstAng = hdUtils_.prvAng;
	hdUtils_.prvAng = this->geoStatus_->GimbalAngles;
	hdUtils_.vrLstInputAngVel = hdUtils_.lstInputAngVel;
	hdUtils_.lstInputAngVel = hdUtils_.prvInputAngVel;
	hdUtils_.prvInputAngVel = vel_buff;
	hdUtils_.vrLstOutAngVel = hdUtils_.lstOutAngVel;
	hdUtils_.lstOutAngVel = hdUtils_.prvOutAngVel;
	hdUtils_.prvOutAngVel = hdUtils_.avelocity;
	hdUtils_.avelocityTemp = hdUtils_.avelocityTemp + ((hdUtils_.avelocity - hdUtils_.avelocityTemp) * (0.001 / (0.001 + 0.07)));

	this->geoStatus_->stylusAngularVelocity = hdUtils_.avelocityTemp;

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
	vel_buff = (this->geoStatus_->PosAngles * 3 - 4 * hdUtils_.prvAng0 + hdUtils_.lstAng0) / 0.002; //mm/s, 2nd order backward dif
	hdUtils_.avelocity0 = (.2196 * (vel_buff + hdUtils_.vrLstInputAngVel0) + .6588 * (hdUtils_.prvInputAngVel0 + hdUtils_.lstInputAngVel0)) / 1000.0 
	                  - (-2.7488 * hdUtils_.prvOutAngVel0 + 2.5282 * hdUtils_.lstOutAngVel0 - 0.7776 * hdUtils_.vrLstOutAngVel0); //cutoff freq of 20 Hz
	hdUtils_.lstAng0 = hdUtils_.prvAng0;
	hdUtils_.prvAng0 = this->geoStatus_->PosAngles;
	hdUtils_.vrLstInputAngVel0 = hdUtils_.lstInputAngVel0;
	hdUtils_.lstInputAngVel0 = hdUtils_.prvInputAngVel0;
	hdUtils_.prvInputAngVel0 = vel_buff;
	hdUtils_.vrLstOutAngVel0 = hdUtils_.lstOutAngVel0;
	hdUtils_.lstOutAngVel0 = hdUtils_.prvOutAngVel0;
	hdUtils_.prvOutAngVel0 = hdUtils_.avelocity0;
	hdUtils_.avelocityTemp0 = hdUtils_.avelocityTemp0 + ((hdUtils_.avelocity0 - hdUtils_.avelocityTemp0) * (0.001 / (0.001 + 0.07)));

	this->geoStatus_->PosAnglesVel = hdUtils_.avelocityTemp0;	

	// Compute gimbal joint velocities
	vel_buff.set(0.0, 0.0, 0.0);
	vel_buff = (this->geoStatus_->GimbalAngles * 3 - 4 * hdUtils_.prvAng + hdUtils_.lstAng) / 0.002; //mm/s, 2nd order backward dif
	hdUtils_.avelocity = (.2196 * (vel_buff + hdUtils_.vrLstInputAngVel) + .6588 * (hdUtils_.prvInputAngVel + hdUtils_.lstInputAngVel)) / 1000.0 
	                  - (-2.7488 * hdUtils_.prvOutAngVel + 2.5282 * hdUtils_.lstOutAngVel - 0.7776 * hdUtils_.vrLstOutAngVel); //cutoff freq of 20 Hz
	hdUtils_.lstAng = hdUtils_.prvAng;
	hdUtils_.prvAng = this->geoStatus_->GimbalAngles;
	hdUtils_.vrLstInputAngVel = hdUtils_.lstInputAngVel;
	hdUtils_.lstInputAngVel = hdUtils_.prvInputAngVel;
	hdUtils_.prvInputAngVel = vel_buff;
	hdUtils_.vrLstOutAngVel = hdUtils_.lstOutAngVel;
	hdUtils_.lstOutAngVel = hdUtils_.prvOutAngVel;
	hdUtils_.prvOutAngVel = hdUtils_.avelocity;
	hdUtils_.avelocityTemp = hdUtils_.avelocityTemp + ((hdUtils_.avelocity - hdUtils_.avelocityTemp) * (0.001 / (0.001 + 0.07)));

	this->geoStatus_->GimbalAnglesVel = hdUtils_.avelocityTemp;

}


/**
* @brief Set function
* Set the feedback haptic force on the Geomagic device
* @param f: the feedback force to be set
*/
void GeomagicProxy::setHapticForce(const Eigen::VectorXf& f) {

	for (int i = 0; i < GEOMAGIC_HAPTIC_DOF; i++) {
		this->geoStatus_->force[i] = f(i);
	}
}

/**
* @brief Set function
* Set the 3D vector of the hip position of the device
* @param p: the 3D vector of the hip position of the device
*/
void GeomagicProxy::setHIPPosition(const Eigen::Vector3f& p) {

	for (int i = 0; i < SPACE_DIM; i++) {
		this->geoStatus_->stylusPosition[i] = p(i);
	}

}


/**
* @brief Set function
* Set the 3D vector of the hip orientation of the device
* @param p: the 3D vector of the hip orientation of the device
*/
void GeomagicProxy::setHIPGimbalAngles(const Eigen::Vector3f& ga) {

	for (int i = 0; i < SPACE_DIM; i++) {
		this->geoStatus_->GimbalAngles[i] = ga(i);
	}

}

/**
* @brief Set function
* Set the 6D vector of the hip velocity of the device
* @param v: the 6D vector of the hip velocity of the device
*/
void GeomagicProxy::setHIPVelocity(const Eigen::Vector6f& v) {

	for (int i = 0; i < SPACE_DIM; i++) {
		this->geoStatus_->stylusLinearVelocity[i] = v(i);
		this->geoStatus_->stylusAngularVelocity[i] = v(i + SPACE_DIM);
	}

}


/**
* @brief Set function
* Set the dynamic array with the state (on/off) of the buttons placed on the device
* @param state: the dynamic array with the state (on/off) of the buttons placed on the device
*/
void GeomagicProxy::setButtonState(const bool* state) {

	this->geoStatus_->action[GEOMAGIC_LOW_BUTTON] = state[GEOMAGIC_LOW_BUTTON];
	this->geoStatus_->action[GEOMAGIC_HIGH_BUTTON] = state[GEOMAGIC_HIGH_BUTTON];

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
		ret(i) = this->geoStatus_->force[i];
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
		ret(i) = this->geoStatus_->stylusPosition[i];
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
		ret(i) = this->geoStatus_->GimbalAngles[i];
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
		ret(i) = this->geoStatus_->stylusLinearVelocity[i];
		ret(i + SPACE_DIM) = this->geoStatus_->stylusAngularVelocity[i];
	}

	return ret;
}


/**
* @brief Get function
* Get the dynamic array with the state (on/off) of the buttons placed on the device
* @return the dynamic array with the state (on/off) of the buttons placed on the device
*/
bool* GeomagicProxy::getButtonState() {

	return (this->geoStatus_->action);

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