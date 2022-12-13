/** GeomagicProxy.cpp
 * 
 * This code is cloned from https://github.com/marcofer/portable-dvrk-simulator.git
 * and modified by Songjie Xiao (songjiexiao@zju.edu.cn) mainly for academic use.
 * 
 */

// Project Header files
#include <touch_driver/GeomagicProxy.h>

GeomagicProxy::GeomagicProxy() 
{
	this->geoStatus_ = std::make_shared<GeomagicStatus>();

	/* Cartesian space values */
	this->geoStatus_->stylusPosition.set(0.0, 0.0, 0.0);
	this->geoStatus_->stylusOrientation = hduQuaternion(1.0, hduVector3Dd(0.0, 0.0, 0.0));
	this->geoStatus_->stylusLinearVelocity.set(0.0, 0.0, 0.0);
	this->geoStatus_->stylusAngularVelocity.set(0.0, 0.0, 0.0);
	this->geoStatus_->stylusForce.set(0.0, 0.0, 0.0);
	this->geoStatus_->cartPose.resize(7);
	this->geoStatus_->cartTwist.resize(6);
	this->geoStatus_->cartWrench.resize(3);

	std::memset(this->geoStatus_->jacobian, 0.0, sizeof(double) * (SPACE_DIM * 2) * GEOMAGIC_HAPTIC_JOINTS);

	/* Joint space values */
	this->geoStatus_->PosAngles.set(0.0, 0.0, 0.0);
	this->geoStatus_->GimbalAngles.set(0.0, 0.0, 0.0);
	// this->geoStatus_->PosAnglesVel.set(0.0, 0.0, 0.0);
	// this->geoStatus_->GimbalAnglesVel.set(0.0, 0.0, 0.0);
	this->geoStatus_->PosAnglesEff.set(0.0, 0.0, 0.0);
	// this->geoStatus_->GimbalAnglesEff.set(0.0, 0.0, 0.0);
	this->geoStatus_->jointPosition.resize(GEOMAGIC_HAPTIC_JOINTS);
	this->geoStatus_->jointVelocity.resize(GEOMAGIC_HAPTIC_JOINTS);
	this->geoStatus_->jointEffort.resize(GEOMAGIC_HAPTIC_DOF);

	/* Button state */
	this->geoStatus_->buttons[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->buttons[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus_->buttons_prev[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->buttons_prev[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus_->action[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->action[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus_->evRaiseEdge[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->evRaiseEdge[GEOMAGIC_HIGH_BUTTON] = false;
	this->geoStatus_->evTrailEdge[GEOMAGIC_LOW_BUTTON] = false;
	this->geoStatus_->evTrailEdge[GEOMAGIC_HIGH_BUTTON] = false;


	// Utility data
	this->hdUtils_ = std::make_shared<HDUtilityData>();
	// linear history position/velocity
	this->hdUtils_->prvPos.set(0.0, 0.0, 0.0);
	this->hdUtils_->lstPos.set(0.0, 0.0, 0.0);

	this->hdUtils_->prvInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils_->lstInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils_->vrLstInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils_->prvOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils_->lstOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils_->vrLstOutVel.set(0.0, 0.0, 0.0);

	this->hdUtils_->lvelocity.set(0.0, 0.0, 0.0);
	this->hdUtils_->lvelocityTemp.set(0.0, 0.0, 0.0);

	// history pos joint value/velocity
	this->hdUtils_->prvAng.set(0.0, 0.0, 0.0);
	this->hdUtils_->lstAng.set(0.0, 0.0, 0.0);

	this->hdUtils_->prvInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils_->lstInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils_->vrLstInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils_->prvOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils_->lstOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils_->vrLstOutAngVel.set(0.0, 0.0, 0.0);

	this->hdUtils_->avelocity.set(0.0, 0.0, 0.0);
	this->hdUtils_->avelocityTemp.set(0.0, 0.0, 0.0);

	// history gimbal joint value/velocity
	this->hdUtils_->prvAng0.set(0.0, 0.0, 0.0);
	this->hdUtils_->lstAng0.set(0.0, 0.0, 0.0);

	this->hdUtils_->prvInputAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils_->lstInputAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils_->vrLstInputAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils_->prvOutAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils_->lstOutAngVel0.set(0.0, 0.0, 0.0);
	this->hdUtils_->vrLstOutAngVel0.set(0.0, 0.0, 0.0);

	this->hdUtils_->avelocity0.set(0.0, 0.0, 0.0);
	this->hdUtils_->avelocityTemp0.set(0.0, 0.0, 0.0);

	this->setAvailable(true);
	this->setRunning(false);
	this->setJointForceMode();

	this->command_.set(0.0, 0.0, 0.0);
}

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

void GeomagicProxy::catchButtonPressEvent(const bool& button, bool& button_prev, bool& raise, bool& trail, bool& trigger) 
{
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
}

void GeomagicProxy::run() 
{
	////////////////////////////////////////////////////////////////
    // Init Phantom
    ////////////////////////////////////////////////////////////////
	HDErrorInfo error;
	this->dvcHandle_ = hdInitDevice(HD_MY_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		std::cout << "Failed to initialize haptic device. Code: 0x0" << std::endl;
		this->setAvailable(false);
	}
	std::cout << "Found Device: " << hdGetString(HD_DEVICE_MODEL_TYPE) << std::endl;

	// Calibrate the device
	HHD_Auto_Calibration();

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

		while (!hdWaitForCompletion(this->schHandle_, HD_WAIT_CHECK_STATUS));
		hdScheduleSynchronous(updateGeoStateCallback, this, HD_DEFAULT_SCHEDULER_PRIORITY);

		//----------------------------------------------------------------//

	}

	// Stop the scheduler
	hdStopScheduler();

	// Unschedule the tasks
	hdUnschedule(this->schHandle_);

	// Disable the device
	hdDisableDevice(this->dvcHandle_);

}

void GeomagicProxy::stop() 
{
	this->setRunning(false);
}

HDCallbackCode updateGeoStateCallback(void* data) {

	GeomagicProxy* device = (GeomagicProxy*)data;

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Failed");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else {
		/* Device state */
		// hdGetDoublev(HD_CURRENT_INKWELL_SWITCH, &device->geoStatus_->inwell_switch);
		// hdrate varies with time 700-900
		// hdGetDoublev(HD_UPDATE_RATE, &device->geoStatus_->hdrate);

		/* Joint space values */
		hdGetDoublev(HD_CURRENT_JOINT_ANGLES, device->geoStatus_->PosAngles);
		hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, device->geoStatus_->GimbalAngles);
		hdGetDoublev(HD_CURRENT_JOINT_TORQUE, device->geoStatus_->PosAnglesEff);
		// hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, device->geoStatus_->GimbalAnglesEff);

		// update joint value in jointPosition according to urdf frames
		device->geoStatus_->jointPosition[0] = -device->geoStatus_->PosAngles[0];
		device->geoStatus_->jointPosition[1] = device->geoStatus_->PosAngles[1];
		device->geoStatus_->jointPosition[2] = device->geoStatus_->PosAngles[2]-device->geoStatus_->PosAngles[1];
		device->geoStatus_->jointPosition[3] = -device->geoStatus_->GimbalAngles[0];
		device->geoStatus_->jointPosition[4] = device->geoStatus_->GimbalAngles[1] + 1.49;
		device->geoStatus_->jointPosition[5] = device->geoStatus_->GimbalAngles[2];

		// update joint velocity in jointVelocity
		device->updateJointVelocities();

		// for (int i=0; i<3; i++){
		// 	device->geoStatus_->PosAnglesVel[i] = device->geoStatus_->jointVelocity[i];
		// 	device->geoStatus_->GimbalAnglesVel[i] = device->geoStatus_->jointVelocity[i+3];
		// }

		// update effort value in PosAnglesEff and GimbalAnglesEff
		device->geoStatus_->jointEffort[0] = - device->geoStatus_->PosAnglesEff[0];
		device->geoStatus_->jointEffort[1] = device->geoStatus_->PosAnglesEff[1];
		device->geoStatus_->jointEffort[2] = device->geoStatus_->PosAnglesEff[2];
		// device->geoStatus_->jointEffort[3] = device->geoStatus_->GimbalAnglesEff[0];
		// device->geoStatus_->jointEffort[4] = device->geoStatus_->GimbalAnglesEff[1];
		// device->geoStatus_->jointEffort[5] = device->geoStatus_->GimbalAnglesEff[2];

		/* Cartesian space values */ 
		// update Cartesian Pose
		hduMatrix transform_ref;
		// hdGetDoublev(HD_CURRENT_POSITION, device->geoStatus_->stylusPosition);
		hdGetDoublev(HD_CURRENT_TRANSFORM, transform_ref);
		device->geoStatus_->stylusPosition = hduVector3Dd(transform_ref[3][0], transform_ref[3][1], transform_ref[3][2]);
		transform_ref.getRotation(device->geoStatus_->stylusOrientation);
		device->geoStatus_->stylusPosition *= 1e-3; // Convert from [mm] to [m]
		
		// update Cartesian Velocity
		hdGetDoublev(HD_CURRENT_VELOCITY, device->geoStatus_->stylusLinearVelocity);
		device->geoStatus_->stylusLinearVelocity *= 1e-3; // Convert from [mm/s] to [m/s]
		// HD(HD_CURRENT_ANGULAR_VELOCITY) does not work
		// hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, device->geoStatus_->stylusAngularVelocity);

		// update Cartesian Force (only works when you set HD_CURRENT_FORCE)
		hdGetDoublev(HD_CURRENT_FORCE, device->geoStatus_->stylusForce);

		// device->updateCartesianVelocities();

		// transform frame TODO: transform orientation
		device->geoStatus_->cartPose[0] = - device->geoStatus_->stylusPosition[0];
		device->geoStatus_->cartPose[1] = device->geoStatus_->stylusPosition[2];
		device->geoStatus_->cartPose[2] = device->geoStatus_->stylusPosition[1];
		device->geoStatus_->cartPose[3] = device->geoStatus_->stylusOrientation.v()[0];
		device->geoStatus_->cartPose[4] = device->geoStatus_->stylusOrientation.v()[1];
		device->geoStatus_->cartPose[5] = device->geoStatus_->stylusOrientation.v()[2];
		device->geoStatus_->cartPose[6] = device->geoStatus_->stylusOrientation.s();

		device->geoStatus_->cartTwist[0] = - device->geoStatus_->stylusLinearVelocity[0];
		device->geoStatus_->cartTwist[1] = device->geoStatus_->stylusLinearVelocity[2];
		device->geoStatus_->cartTwist[2] = device->geoStatus_->stylusLinearVelocity[1];
		device->geoStatus_->cartTwist[3] = device->geoStatus_->stylusAngularVelocity[0];
		device->geoStatus_->cartTwist[4] = device->geoStatus_->stylusAngularVelocity[1];
		device->geoStatus_->cartTwist[5] = device->geoStatus_->stylusAngularVelocity[2];

		device->geoStatus_->cartWrench[0] = - device->geoStatus_->stylusForce[0];
		device->geoStatus_->cartWrench[1] = device->geoStatus_->stylusForce[2];
		device->geoStatus_->cartWrench[2] = device->geoStatus_->stylusForce[1];

		// update Jacobian matrix
		hdGetDoublev(HD_CURRENT_JACOBIAN, device->geoStatus_->jacobian);

		// Test for Jacobian
		// TODO: jacobian linear vel should be divide by 1000?
		/* Eigen::Matrix<double, (SPACE_DIM * 2), GEOMAGIC_HAPTIC_JOINTS> J;
		Eigen::Matrix<double, (SPACE_DIM * 2), 1> vel;
		Eigen::Matrix<double, GEOMAGIC_HAPTIC_JOINTS, 1> qdot;

		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				J(i, j) = device->geoStatus_->jacobian[i * 6 + j];
			}
		}

		for (int i=0; i<6; i++){
			qdot[i] = device->geoStatus_->jointVelocity[i];
		}
		vel = J * qdot;
		if (vel.norm() > 1e3) vel.setZero();
		device->geoStatus_->stylusLinearVelocity = hduVector3Dd(vel(0), vel(1), vel(2));
		device->geoStatus_->stylusAngularVelocity = hduVector3Dd(vel(3), vel(4), vel(5)); */

		// Button state
		int stylusButtons;
		hdGetIntegerv(HD_CURRENT_BUTTONS, &stylusButtons);

		for (int i = 0; i < GEOMAGIC_BUTTONS_NUM; i++) {
			device->geoStatus_->buttons_prev[i] = device->geoStatus_->buttons[i];
			device->geoStatus_->buttons[i] = (stylusButtons == i + 1 || stylusButtons == PRESSED_BOTH) ? true : false;

			device->catchButtonPressEvent(device->geoStatus_->buttons[i], device->geoStatus_->buttons_prev[i],
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
	if (device->mode_ == JOINT_SPACE){
		hdSetDoublev(HD_CURRENT_JOINT_TORQUE, device->command_);
	}
	else if(device->mode_ == CARTESIAN_SPACE){
		hdSetDoublev(HD_CURRENT_FORCE, device->command_);
	}
	else{
		hdSetDoublev(HD_CURRENT_JOINT_TORQUE, hduVector3Dd(0.0, 0.0, 0.0));
	}
	
	hdEndFrame(device->dvcHandle_);

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Error during force scheduler callback");
		device->setAvailable(false);
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else { device->setAvailable(true); }
	return HD_CALLBACK_CONTINUE;
}

void GeomagicProxy::updateCartesianVelocities()
{
	hduVector3Dd vel_buff(0.0, 0.0, 0.0);

	// Compute linear velocities
	vel_buff = (this->geoStatus_->stylusPosition * 3 - 4 * hdUtils_->prvPos + hdUtils_->lstPos) / 0.002; //mm/s, 2nd order backward dif
	hdUtils_->lvelocity = (.2196 * (vel_buff + hdUtils_->vrLstInputVel) + .6588 * (hdUtils_->prvInputVel + hdUtils_->lstInputVel)) / 1000.0 - (-2.7488 * hdUtils_->prvOutVel + 2.5282 * hdUtils_->lstOutVel - 0.7776 * hdUtils_->vrLstOutVel); //cutoff freq of 20 Hz
	hdUtils_->lstPos = hdUtils_->prvPos;
	hdUtils_->prvPos = this->geoStatus_->stylusPosition;
	hdUtils_->vrLstInputVel = hdUtils_->lstInputVel;
	hdUtils_->lstInputVel = hdUtils_->prvInputVel;
	hdUtils_->prvInputVel = vel_buff;
	hdUtils_->vrLstOutVel = hdUtils_->lstOutVel;
	hdUtils_->lstOutVel = hdUtils_->prvOutVel;
	hdUtils_->prvOutVel = hdUtils_->lvelocity;
	hdUtils_->lvelocityTemp = hdUtils_->lvelocityTemp + ((hdUtils_->lvelocity - hdUtils_->lvelocityTemp) * (0.001 / (0.001 + 0.07)));

	this->geoStatus_->stylusLinearVelocity = hdUtils_->lvelocityTemp;


	// Compute angular velocities
	vel_buff.set(0.0, 0.0, 0.0);
	vel_buff = (this->geoStatus_->GimbalAngles * 3 - 4 * hdUtils_->prvAng + hdUtils_->lstAng) / 0.002; //mm/s, 2nd order backward dif
	hdUtils_->avelocity = (.2196 * (vel_buff + hdUtils_->vrLstInputAngVel) + .6588 * (hdUtils_->prvInputAngVel + hdUtils_->lstInputAngVel)) / 1000.0 - (-2.7488 * hdUtils_->prvOutAngVel + 2.5282 * hdUtils_->lstOutAngVel - 0.7776 * hdUtils_->vrLstOutAngVel); //cutoff freq of 20 Hz
	hdUtils_->lstAng = hdUtils_->prvAng;
	hdUtils_->prvAng = this->geoStatus_->GimbalAngles;
	hdUtils_->vrLstInputAngVel = hdUtils_->lstInputAngVel;
	hdUtils_->lstInputAngVel = hdUtils_->prvInputAngVel;
	hdUtils_->prvInputAngVel = vel_buff;
	hdUtils_->vrLstOutAngVel = hdUtils_->lstOutAngVel;
	hdUtils_->lstOutAngVel = hdUtils_->prvOutAngVel;
	hdUtils_->prvOutAngVel = hdUtils_->avelocity;
	hdUtils_->avelocityTemp = hdUtils_->avelocityTemp + ((hdUtils_->avelocity - hdUtils_->avelocityTemp) * (0.001 / (0.001 + 0.07)));

	this->geoStatus_->stylusAngularVelocity = hdUtils_->avelocityTemp;

}

void GeomagicProxy::updateJointVelocities()
{
	hduVector3Dd vel_buff(0.0, 0.0, 0.0);
	hduVector3Dd angles(0.0, 0.0, 0.0);

	// Compute pos joint velocities
	vel_buff.set(0.0, 0.0, 0.0);
	for (int i=0; i<3; i++){
		angles[i] = this->geoStatus_->jointPosition[i];
	}
	vel_buff = (angles * 3 - 4 * hdUtils_->prvAng0 + hdUtils_->lstAng0) / 0.002; //mm/s, 2nd order backward dif
	hdUtils_->avelocity0 = (.2196 * (vel_buff + hdUtils_->vrLstInputAngVel0) + .6588 * (hdUtils_->prvInputAngVel0 + hdUtils_->lstInputAngVel0)) / 1000.0 
	                  - (-2.7488 * hdUtils_->prvOutAngVel0 + 2.5282 * hdUtils_->lstOutAngVel0 - 0.7776 * hdUtils_->vrLstOutAngVel0); //cutoff freq of 20 Hz
	hdUtils_->lstAng0 = hdUtils_->prvAng0;
	hdUtils_->prvAng0 = angles;
	hdUtils_->vrLstInputAngVel0 = hdUtils_->lstInputAngVel0;
	hdUtils_->lstInputAngVel0 = hdUtils_->prvInputAngVel0;
	hdUtils_->prvInputAngVel0 = vel_buff;
	hdUtils_->vrLstOutAngVel0 = hdUtils_->lstOutAngVel0;
	hdUtils_->lstOutAngVel0 = hdUtils_->prvOutAngVel0;
	hdUtils_->prvOutAngVel0 = hdUtils_->avelocity0;
	hdUtils_->avelocityTemp0 = hdUtils_->avelocityTemp0 + ((hdUtils_->avelocity0 - hdUtils_->avelocityTemp0) * (0.001 / (0.001 + 0.07)));

	for (int i=0; i<3; i++){
		this->geoStatus_->jointVelocity[i] = hdUtils_->avelocityTemp0[i];
	}

	// Compute gimbal joint velocities
	vel_buff.set(0.0, 0.0, 0.0);
	for (int i=0; i<3; i++){
		angles[i] = this->geoStatus_->jointPosition[i+3];
	}
	vel_buff = (angles* 3 - 4 * hdUtils_->prvAng + hdUtils_->lstAng) / 0.002; //mm/s, 2nd order backward dif
	hdUtils_->avelocity = (.2196 * (vel_buff + hdUtils_->vrLstInputAngVel) + .6588 * (hdUtils_->prvInputAngVel + hdUtils_->lstInputAngVel)) / 1000.0 
	                  - (-2.7488 * hdUtils_->prvOutAngVel + 2.5282 * hdUtils_->lstOutAngVel - 0.7776 * hdUtils_->vrLstOutAngVel); //cutoff freq of 20 Hz
	hdUtils_->lstAng = hdUtils_->prvAng;
	hdUtils_->prvAng = angles;
	hdUtils_->vrLstInputAngVel = hdUtils_->lstInputAngVel;
	hdUtils_->lstInputAngVel = hdUtils_->prvInputAngVel;
	hdUtils_->prvInputAngVel = vel_buff;
	hdUtils_->vrLstOutAngVel = hdUtils_->lstOutAngVel;
	hdUtils_->lstOutAngVel = hdUtils_->prvOutAngVel;
	hdUtils_->prvOutAngVel = hdUtils_->avelocity;
	hdUtils_->avelocityTemp = hdUtils_->avelocityTemp + ((hdUtils_->avelocity - hdUtils_->avelocityTemp) * (0.001 / (0.001 + 0.07)));

	for (int i=0; i<3; i++){
		this->geoStatus_->jointVelocity[i+3] = hdUtils_->avelocityTemp[i];
	}
}

void GeomagicProxy::setForceCommand(double Fx, double Fy, double Fz)
{
	this->command_ = hduVector3Dd(-Fx, Fy, Fz);
}

void GeomagicProxy::setForceCommand(std::vector<double> vec)
{
	// for(int i=0; i<3; i++){
	// 	this->command_[i] = vec[i];
	// }
	this->command_[0] = -vec[0];
	this->command_[1] = vec[1];
	this->command_[2] = vec[2];
}

