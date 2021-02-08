/*
 * kfApp.cpp
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file kfApp.cpp
 *  @ingroup kfApp
 *  @brief kfApp class
 */

/**
 *  @addtogroup kfApp
 *  @{
 */
#define _USE_MATH_DEFINES
#include<iostream>
#include<math.h>
#include "kfApp.h"
#include "kalmanFilter.h"

/**
 * @brief kfApp The constructor for kfApp.
 *
 * @param[in] num_states which is number of kalman filter states {int}.
 *
 */
kfApp::kfApp(int num_states)
{

    ///* set kalman filter data.
    kd_.setKalmanData(num_states);

    // initially set to false, set to true in first call of ProcessMeasurement.
    is_initialized_ = false;

    //set time to be zero initially.
    previous_timestamp_ = 0;

    // Process noise standard deviation longitudinal acceleration in m/s^2.
    noise_ax= 5;

    // Process noise standard deviation longitudinal acceleration in m/s^2.
    noise_ay= 5;

    // Laser measurement noise standard deviation position1 in m.
    std_laspx_ = 0.05;

    // Laser measurement noise standard deviation position2 in m.
    std_laspy_ = 0.05;

    // Radar measurement noise standard deviation radius in m.
    std_radr_ = 0.5;

    // Radar measurement noise standard deviation angle in rad.
    std_radphi_ = 0.05;

    // Radar measurement noise standard deviation radius change in m/s.
    std_radrd_ = 0.5;
}

kfApp::~kfApp()
{

}

/**
 * @brief ProcessMeasurement Perform the Prediction step using kalman filter.
 *
 * @param[in] measurement_pack
 *  The measurement pack that contains sensor type , time stamp and readings {MeasurementPackage}.
 *
 */
void kfApp::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{

    /*****************************************************************************
     * Initialization
     ****************************************************************************/
    if (!is_initialized_)
    {
        kd_.x << 1, 1, 1, 1;
        kd_.P = Eigen::MatrixXd::Identity(kd_.m_numState,kd_.m_numState);
        kd_.P(2,2) = 1000.0;
        kd_.P(3,3) = 1000.0;

        //check if radar readings
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            // Convert radar measurement from polar to cartesian coordinates
            float rho = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);

            float px = rho * cos(phi);
            float py = rho * sin(phi);

            kd_.x << px,py,0.0,0.0;
        }

        //check if lidar readings
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
             kd_.x << measurement_pack.raw_measurements_(0),measurement_pack.raw_measurements_(1),0,0;
        }


        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    /**********************************************************************
     *    sample time claculations
     *********************************************************************/
    //compute the time elapsed between the current and previous measurements
    //dt - expressed in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    /**********************************************************************
     *    Prediction
     *********************************************************************/
    //perform Prediction step.
    Prediction(dt);

    /**********************************************************************
     *    Update
     *********************************************************************/
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        UpdateLidar(measurement_pack);
    }
    else if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        UpdateRadar(measurement_pack);
    }
}


/**
 * @brief Prediction Update Prediction model.
 *
 * @param[in] dt the change in time (in seconds) between the last.
 * measurement and this one {double} .
 *
 */
void kfApp::Prediction(const double dt)
{
    /**********************************************************************
     *    1. Set the process covariance matrix Q
     *********************************************************************/
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    Eigen::MatrixXd Q = Eigen::MatrixXd(kd_.m_numState, kd_.m_numState);
    Q <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

    /**********************************************************************
     *    2. Perform the predict step
     *********************************************************************/
    //set kalman paramters for prediction step
    kalmanFilter::predict(kd_.x, kd_.P, Q, g_, g_prime_, &dt);
}

/**
 * @brief Update the state and the state covariance matrix using a laser measurement.
 *
 * @param[in]  meas_package
 *  The measurement pack that contains sensor type time stamp and readings {MeasurementPackage}.
 */
void kfApp::UpdateLidar(MeasurementPackage meas_package)
{
    /**********************************************************************
     *    extract measurement size
     *********************************************************************/
    int measSize = meas_package.raw_measurements_.size();

    /**********************************************************************
     *    set output matrix H
     *********************************************************************/
    Eigen::MatrixXd H = Eigen::MatrixXd(measSize, kd_.m_numState);
    H.setIdentity(measSize, kd_.m_numState);

    /**********************************************************************
     *    add measurement noise covariance matrix
     *********************************************************************/
    Eigen::MatrixXd R = Eigen::MatrixXd(measSize, measSize);
    R.fill(0.0);
    R.diagonal()<<pow(std_laspx_, 2),pow(std_laspy_, 2);

    /**********************************************************************
     *    Calculate Innovation
     *********************************************************************/
    Eigen::VectorXd zpred = H * kd_.x;
    Eigen::VectorXd Y = meas_package.raw_measurements_ - zpred;
    Eigen::MatrixXd S = (H * kd_.P * H.transpose()) + R;
    kd_.nis = Y.transpose() * S.inverse() * Y;

    /**********************************************************************
     *    Calculate Kalman Gain
     *********************************************************************/
    Eigen::MatrixXd K = kalmanFilter::CalculateKalmanGain(kd_.P, H, R);

    /**********************************************************************
     *    Update Linear
     *********************************************************************/
    kalmanFilter::update(kd_.x, kd_.P, Y, H, K);
}

/**
 * @brief Update the state and the state covariance matrix using a radar measurement.
 *
 * @param[in]  meas_package
 *  The measurement pack that contains sensor type time stamp and readings {MeasurementPackage}.
 */
void kfApp::UpdateRadar(MeasurementPackage meas_package)
{
    /**********************************************************************
     *    extract measurement size
     *********************************************************************/
    int measSize = meas_package.raw_measurements_.size();

    /**********************************************************************
     *    set output matrix H
     *********************************************************************/
    Eigen::MatrixXd H = Eigen::MatrixXd(measSize, kd_.m_numState);
    H = h_prime_(kd_.x);

    /**********************************************************************
     *    add measurement noise covariance matrix
     *********************************************************************/
    Eigen::MatrixXd R = Eigen::MatrixXd(measSize, measSize);
    R.fill(0.0);
    R.diagonal()<<pow(std_radr_, 2),pow(std_radphi_, 2),pow(std_radrd_, 2);

    /**********************************************************************
     *    Calculate Measurement Prediction
     *********************************************************************/
    Eigen::VectorXd zpred = h_(kd_.x ,measSize);
    Eigen::VectorXd Y = meas_package.raw_measurements_ - zpred;
    Y = Innovationhelper(Y);
    Eigen::MatrixXd S = (H * kd_.P * H.transpose()) + R;
    kd_.nis = Y.transpose() * S.inverse() * Y;

    /**********************************************************************
     *    Calculate Kalman Gain
     *********************************************************************/
    Eigen::MatrixXd K = kalmanFilter::CalculateKalmanGain(kd_.P, H, R);

    /**********************************************************************
     *    Update Linear
     *********************************************************************/
    kalmanFilter::update(kd_.x, kd_.P, Y, H, K);
}

/**
 * @brief g Function 
 *  which calculates the mean state vector based dynamic model.
 *
 * @param[in] mean
 *  the state vector {VectorXd&}.
 * 
 * @param[in] p_args
 *  Extra arguments {const void *}.
 * 
 * @return F.x
 *  the mean state vector {{VectorXd}}.
 */
Eigen::VectorXd kfApp::g_(const Eigen::VectorXd &mean, const void *p_args)
{
    //create state transition matrix for predicted state covariance.
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(mean.rows(), mean.rows());
    double dt = *(double*)p_args;
    F << 1, 0, 1, 0,
         0, 1, 0, 1,
         0, 0, 1, 0,
         0, 0, 0, 1;
    F(0, 2) = dt;
    F(1, 3) = dt;
    return F* mean;
}

/**
 * @brief g_prime the derivative of g_function.
 *  In linear case it shall return the state transition Matrix.
 *  In non-linear it shall return the jacobians. 
 *
 * @param[in] mean
 *  the state vector {VectorXd&}.
 * 
 * @param[in] p_args
 *  Extra arguments {const void *}.
 * 
 * @return F 
 *  the state transition matrix {MatrixXd}.
 */
Eigen::MatrixXd kfApp::g_prime_ (const Eigen::VectorXd &mean, const void *p_args)
{
    //create state transition matrix for predicted state covariance.
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(mean.rows(), mean.rows());
    double dt = *(double*)p_args;
    F << 1, 0, 1, 0,
         0, 1, 0, 1,
         0, 0, 1, 0,
         0, 0, 0, 1;
    F(0, 2) = dt;
    F(1, 3) = dt;
    return F;
}

/**
 * @brief h_ Function 
 * which calculates the mean state vector for measurements based on sensor model.
 *
 * @param[in]  x
 *  The Kalman filter state matrix {VectorXd&}.
 *
 * @param[in]  size
 *  The size of measurement matrix {size_t}.
 *
 *  @return  h_x 
 *  the mean state predicted vector {VectorXd}.
 */
Eigen::VectorXd kfApp:: h_(const Eigen::VectorXd &x , size_t size)
{
    Eigen::VectorXd h_x(size);
    float rho = sqrt(x(0) * x(0) + x(1) * x(1));

    //check division by zero
    if(fabs(rho) < 0.0001)
    {
        h_x(0) = rho;
        h_x(1) = atan2(x(1), x(0));
        h_x(2) = 0;
    }
    else
    {
        h_x(0) = rho;
        h_x(1) = atan2(x(1), x(0));
        h_x(2) = (x(0) * x(2) + x(1) * x(3)) / rho;
    }
    return h_x;
}

/**
 * @brief h_prime the derivative of h_function.
 *  In linear case it shall return the state transition Matrix.
 *  In non-linear it shall return the jacobians.
 * 
 * @param[in]  x_state
 *  The state vector {VectorXd&}.
 *
 *  @return   Hj 
 *  the state transition matrix of measurements{MatrixXd}.
 */
Eigen::MatrixXd kfApp::h_prime_(const Eigen::VectorXd& x_state)
{
    /**
  TODO:
     * Calculate a Jacobian here.
     */

    Eigen::MatrixXd Hj(3,4);
    Hj.setZero();

    //recover state parameters
    float x     = x_state(0);
    float y     = x_state(1);
    float x_dot = x_state(2);
    float y_dot = x_state(3);

    /*check division by zero*/
    float x2_y2 = pow(x, 2) + pow(y, 2);

    if(fabs(x2_y2) < 0.00001)
    {
        return Hj;
    }

    Hj(0, 0) = x/sqrt(x2_y2);
    Hj(0, 1) = y/sqrt(x2_y2);

    Hj(1, 0) = -y/x2_y2;
    Hj(1, 1) = x/x2_y2;

    Hj(2, 0) = y* (x_dot*y - y_dot*x)/pow(x2_y2, 3/2.0);
    Hj(2, 1) = x * (y_dot*x - x_dot*y)/pow(x2_y2, 3/2.0);
    Hj(2, 2) = x/sqrt(x2_y2);
    Hj(2, 3) = y/sqrt(x2_y2);

    return Hj;
}

/**
 * @brief Innovationhelper, Innovation helper function for angle rounding issues.
 *
 * @param[in] sig_pred
 *  The sigma points vector {VectorXd &} .
 *
 *  @return z_diff 
 *  The innovation vector after rounding fixes {VectorXd}.
 */
Eigen::VectorXd kfApp::Innovationhelper(const Eigen::VectorXd &sig_pred)
{
    Eigen::VectorXd z_diff;
    z_diff = sig_pred;
    //angle normalization
    while(z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while(z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
    return z_diff;
}
/**
 *  @}
 */

