/*
 * ukfApp.cpp
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file ukfApp.cpp
 *  @ingroup ukfApp
 *  @brief ukfApp class
 */

/**
 *  @addtogroup ukfApp
 *  @{
 */
#include<iostream>
#include "ukfApp.h"

#define M_PI		3.14159265358979323846

/*******************************************************************************
 *  Helper Functions Declaration                                                *
 *******************************************************************************/
Eigen::VectorXd PredictionModel (const Eigen::VectorXd &sig_pred_ , const void *args);
Eigen::VectorXd calc_covar (const Eigen::VectorXd &sig_pred , const void *args);
Eigen::VectorXd PredictionModelMeasurement (const Eigen::VectorXd &sig_pred_ , const void *args);
Eigen::VectorXd calc_covar_measurement (const Eigen::VectorXd &sig_pred , const void *args);

/**
 * @brief ukfApp The constructor for ukfApp.
 *
 * @param[in] num_states which is number of kalman filter states{int}.
 *
 * @param[in] aug_states which is number of kalman filter augmented state{int}.
 */
ukfApp::ukfApp(int num_states ,int aug_states)
{
	///* set kalman filter data
	kd_.setKalmanData(num_states, aug_states );

	//set example covariance matrix
	kd_.Q = Eigen::MatrixXd(aug_states,aug_states);

	///* set kalman filter mode
	kf_helper_.setMode(new KfPredictUT(), new KfUpdateUT(), new KfUt, new KfUpdateLinear());

	///* initially set to false, set to true in first call of ProcessMeasurement
	is_initialized_ = false;

	//set time to be zero initially
	previous_timestamp_ = 0;

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ =  4.0;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 5.0;

	//DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
	// Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.05;

	// Laser measurement noise standard deviation position2 in m
	std_laspy_ =0.05;

	// Radar measurement noise standard deviation radius in m
	std_radr_ = 0.4;

	// Radar measurement noise standard deviation angle in rad
	std_radphi_ = 0.04;

	// Radar measurement noise standard deviation radius change in m/s
	std_radrd_ =0.45;
}

/**
 * @brief ukfApp The destructor for ukfApp.
 */
ukfApp::~ukfApp() {}

/**
 * @brief ProcessMeasurement Perform the Prediction step using kalman filter.
 *
 * @param[in] meas_package the
 * measurement pack that contains sensor type time stamp and readings{MeasurementPackage}.
 *
 */
void ukfApp::ProcessMeasurement(const MeasurementPackage &meas_package)
{

	/**********************************************************************
	 *    Initialization
	 *********************************************************************/

	if (!is_initialized_)
	{
		kd_.P = Eigen::MatrixXd::Identity(kd_.m_numState,kd_.m_numState);
		//check if radar readings
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
		{
			// Convert radar measurement from polar to cartesian coordinates
			float rho = meas_package.raw_measurements_(0);
			float phi = meas_package.raw_measurements_(1);
			float rhodot = meas_package.raw_measurements_(2);

			float px = rho * cos(phi);
			float py = rho * sin(phi);
			float vx = rhodot * cos(phi);
			float vy = rhodot * sin(phi);

			kd_.x << px,py,0,0,0;
		}

		//check if lidar readings
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
		{
 			kd_.x << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1),0,0,0;
		}

		previous_timestamp_ = meas_package.timestamp_;
		is_initialized_ = true;
		// It is only initialization, so no need to predict or update
		return;
	}

	/**********************************************************************
	 *    sample time claculations
	 *********************************************************************/
	//calculate delta time.
	double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;

	//save the latest time stamp.
	previous_timestamp_ = meas_package.timestamp_;

	/**********************************************************************
	 *    Prediction
	 *********************************************************************/

	//perform Prediction step.
	Prediction(dt);

	/**********************************************************************
	 *    Update
	 *********************************************************************/
	//perform Update lidar
	if (meas_package.sensor_type_ == MeasurementPackage::LASER)
	{
		UpdateLidar(meas_package);
	}

	//perform Update radar
	else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
	{
		UpdateRadar(meas_package);
	}

	else
	{
		/*Do nothing */
	}
}

/**
 * @brief Prediction  Update Prediction model.
 *
 * @param[in] delta_t the change in time (in seconds) between the last measurement and this one {double}.
 *
 */
void ukfApp::Prediction(const double delta_t)
{
	/**********************************************************************
	 *    1. Set the process covariance matrix Q
	 *********************************************************************/
	kd_.Q.fill(0.0);
	kd_.Q.diagonal()<<pow(std_a_, 2),pow(std_yawdd_, 2);

  /*******************************************************************************
   *  2.Perform Prediction UT step                                              *
   *******************************************************************************/
  	kf_helper_.predict(kd_, PredictionModel, calc_covar, &delta_t) ;
}

/**
 * @brief UpdateLidar Updates the state and the state covariance matrix using a laser measurement.
 * 
 * @param[in]  measurement_pack
 *  The measurement pack that contains sensor type time stamp and readings {MeasurementPackage}.
 */
void ukfApp::UpdateLidar(MeasurementPackage meas_package)
{
	/**********************************************************************
	 *    extract measurement size
	 *********************************************************************/
	int measSize = meas_package.raw_measurements_.size();

	/**********************************************************************
	 *    set output matrix H
	 *********************************************************************/
	kd_.H = Eigen::MatrixXd(measSize, kd_.m_numState);
	kd_.H.setIdentity(measSize, kd_.m_numState);

	/**********************************************************************
	 *    add measurement noise covariance matrix
	 *********************************************************************/
	kd_.R = Eigen::MatrixXd(measSize, measSize);
	kd_.R.fill(0.0);
	kd_.R.diagonal()<<pow(std_laspx_, 2),pow(std_laspy_, 2);

	/**********************************************************************
	 *    Calculate Measurement Prediction
	 *********************************************************************/
	kd_.zpred = kd_.H * kd_.x;

	/**********************************************************************
	 *    Update Linear
	 *********************************************************************/
	kf_helper_.update(kd_,meas_package);
}


/**
 * @brief UpdateRadar Updates the state and the state covariance matrix using a radar measurement.
 * 
 * @param[in] meas_package
 *  The measurement pack that contains sensor type , time stamp and readings {MeasurementPackage} .
 *
 */
void ukfApp::UpdateRadar(MeasurementPackage meas_package)
{
	/**********************************************************************
	 *    extract measurement size
	 *********************************************************************/
	int measSize = meas_package.raw_measurements_.size();

	/**********************************************************************
	 *    add measurement noise covariance matrix
	 *********************************************************************/
	kd_.R = Eigen::MatrixXd(measSize, measSize);
	kd_.R.fill(0.0);
	kd_.R.diagonal()<<pow(std_radr_, 2),pow(std_radphi_, 2),pow(std_radrd_, 2);

	/**********************************************************************
	 *    Calculate Measurement Prediction
	 *********************************************************************/
	kf_helper_.update(kd_, meas_package, PredictionModelMeasurement, calc_covar_measurement, calc_covar,NULL);
}

/*******************************************************************************
 *  Helper Functions Definitions                                                *
 *******************************************************************************/
/**
 * @brief PredictionModel used to propagate the sigma points throgh the prediction model.
 * 
 * @param[in] sig_pred_
 *  The sigma points calculated by the unscented transform {VectorXd} .
 * 
 * @param[in]  p  A void pointer is used to pass arguments to the helper function.
 * 
 * @param[out] Xsig_pred The propagated sigma points{VectorXd}.
 *
 */
Eigen::VectorXd PredictionModel (const Eigen::VectorXd &sig_pred_ , const void *args)
{
	Eigen::VectorXd Xsig_pred = Eigen::VectorXd(5);
	Xsig_pred.fill(0.0);
	double dt = *(double*)args;
	// Extract values for readability
	double p_x      = sig_pred_(0);
	double p_y      = sig_pred_(1);
	double v        = sig_pred_(2);
	double yaw      = sig_pred_(3);
	double yawd     = sig_pred_(4);
	double nu_a     = sig_pred_(5);
	double nu_yawdd = sig_pred_(6);

	// predicted state values
	double px_p, py_p;

	// avoid divison ny zero
	if (fabs(yawd) > 0.001)
	{
		// General equations
		px_p = p_x + v / yawd * ( sin(yaw + yawd * dt) - sin(yaw));
		py_p = p_y + v / yawd * (-cos(yaw + yawd * dt) + cos(yaw));
	}
	else
	{
		// Special case
		px_p = p_x + v * dt * cos(yaw);
		py_p = p_y + v * dt * sin(yaw);
	}

	double v_p = v;
	double yaw_p = yaw + yawd * dt;
	double yawd_p = yawd;

	// add noise
	double dt2 = dt * dt;
	px_p = px_p + 0.5 * nu_a * dt2 * cos(yaw);
	py_p = py_p + 0.5 * nu_a * dt2 * sin(yaw);
	v_p = v_p + nu_a * dt;

	yaw_p  = yaw_p + 0.5 * nu_yawdd * dt2;
	yawd_p = yawd_p + nu_yawdd * dt;

	// write predicted sigma point into right column
	Xsig_pred(0) = px_p;
	Xsig_pred(1) = py_p;
	Xsig_pred(2) = v_p;
	Xsig_pred(3) = yaw_p;
	Xsig_pred(4) = yawd_p;

	return Xsig_pred;
}

/**
 * @brief PredictionModelMeasurement used to propagate the sigma points throgh the measurement model.
 * 
 * @param[in] sig_pred_
 *  The sigma points calculated by the unscented transform {VectorXd} .
 * 
 * @param[in]  p  A void pointer is used to pass arguments to the helper function.
 * 
 * @param[out] Xsig_pred The propagated sigma points{VectorXd}.
 *
 */
Eigen::VectorXd PredictionModelMeasurement (const Eigen::VectorXd &sig_pred_ , const void *args)
{
	VectorXd Xsig_pred = VectorXd(3);
	Xsig_pred.fill(0.0);
	// extract values for better readability
	double p_x = sig_pred_(0);
	double p_y = sig_pred_(1);
	double v   = sig_pred_(2);
	double yaw = sig_pred_(3);

	double vx = v * cos(yaw);
	double vy = v * sin(yaw);

    // Avoid division by zero
    if(fabs(p_x) <= 0.0001)
	{
        p_x = 0.0001;
    }

    if(fabs(p_y) <= 0.0001)
	{
        p_y = 0.0001;
    }

	// measurement model
	double p_x2 = p_x * p_x;
	double p_y2 = p_y * p_y;

	double r     = sqrt(p_x2 + p_y2);
	double phi   = atan2(p_y, p_x);
	double r_dot =  (p_x * vx + p_y * vy) / r;


	// write predicted sigma point into right column
	Xsig_pred(0) = r;
	Xsig_pred(1) = phi;
	Xsig_pred(2) = r_dot;


	return Xsig_pred;
}


/**
 * @brief calc_covar Helper function fix rounding issues in calculating covariance in prediction step.
 * 
 * @param[in] sig_pred_
 *  The state vector during the calculation of covarince {VectorXd} .
 * 
 * @param[in]  p  A void pointer is used to pass arguments to the helper function.
 * 
 * @param[out] x_diff The state vector after applying the proper rounding to angles{VectorXd}.
 *
 */
Eigen::VectorXd calc_covar (const Eigen::VectorXd &sig_pred , const void *args)
{
	Eigen::VectorXd x_diff;
	x_diff = sig_pred;
	// angle normalization
	while (x_diff(3) > M_PI)
		x_diff(3) -= 2.0 * M_PI;
	while (x_diff(3) < -M_PI)
		x_diff(3) += 2.0 * M_PI;
	return x_diff;
}

/**
 * @brief calc_covar_measurement Helper function fix rounding issues in calculating covariance in update step.
 * 
 * @param[in] sig_pred_
 *  The state vector during the calculation of covarince {VectorXd} .
 * 
 * @param[in]  p  A void pointer is used to pass arguments to the helper function.
 * 
 * @param[out] x_diff The state vector after applying the proper rounding to angles{VectorXd}.
 *
 */
Eigen::VectorXd calc_covar_measurement (const Eigen::VectorXd &sig_pred , const void *args)
{
	VectorXd x_diff;
	x_diff = sig_pred;
	//angle normalization
	while (x_diff(1) > M_PI)
		x_diff(1) -= 2.0 * M_PI;
	while (x_diff(1) < -M_PI)
		x_diff(1) += 2.0 * M_PI;
	return x_diff;
}
/**
 *  @}
 */
