/** @file ukf.cpp
 *  @ingroup highway
 *  @brief ukf class
 */

/**
 *  @addtogroup highway
 *  @{
 */

#include "ukf.h"
#include "Eigen/Dense"

#define M_PI		3.14159265358979323846

/**
 * @brief ukfApp 
 * The constructor for ukfApp.
 *
 * @param[in] num_states 
 * which is number of kalman filter states{int}.
 *
 * @param[in] aug_states 
 * which is number of kalman filter augmented state{int}.
 */
UKF::UKF(int num_states ,int aug_states)
{
    ///* set kalman filter data
    kd_.setKalmanData(num_states, aug_states);

    ///* initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    //set time to be zero initially
    previous_timestamp_ = 0;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    const auto expected_a_max = 12; // m/s² (e.g. 6 m/s² for inner-city dynamic driving)
    std_a_ = 0.5 * expected_a_max;  // m/s² acceleration noise

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = M_PI_2; // ±22.5°/s²

    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ =0.3;
}

/**
 * @brief ukf The destructor for ukfApp.
 */
UKF::~UKF() {}

/**
 * @brief ProcessMeasurement 
 * Perform the Prediction step using kalman filter.
 *
 * @param[in] meas_package 
 * the measurement pack that contains sensor type time stamp and readings{MeasurementPackage}.
 *
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    /*******************************************************************************************************************
     *    Initialization
     ******************************************************************************************************************/
    if (!is_initialized_)
    {
        //check if radar readings
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            const auto r(meas_package.raw_measurements_[0]);
            const auto phi(meas_package.raw_measurements_[1]);
            const auto r_dot(meas_package.raw_measurements_[2]);
            kd_.x << r * cos(phi),
                    r * sin(phi),
                    r_dot,
                    phi,
                    0.0;
        }
            //check if lidar readings
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            kd_.x << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1),0.0,0.0,0.0;
        }

        previous_timestamp_ = meas_package.timestamp_;
        is_initialized_ = true;
        // It is only initialization, so no need to predict or update
        return;
    }

    /*******************************************************************************************************************
     *    sample time claculations
     /*****************************************************************************************************************/
    //calculate delta time.
    double dt((meas_package.timestamp_ - previous_timestamp_) / 1000000.0);

    //save the latest time stamp.
    previous_timestamp_ = meas_package.timestamp_;

    /**********************************************************************
     *    Prediction
     *********************************************************************/

    //perform Prediction step.
    Prediction(dt);

    /*******************************************************************************************************************
     *    Update
     /*****************************************************************************************************************/
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
 * @brief Prediction  
 * Update Prediction model.
 *
 * @param[in] delta_t 
 * the change in time (in seconds) between the last measurement and this one {double}.
 *
 */
void UKF::Prediction(double delta_t)
{
    /*******************************************************************************************************************
     *    1. Set the process covariance matrix Q
     /*****************************************************************************************************************/
    Eigen::MatrixXd Q(Eigen::MatrixXd(kd_.m_numAug, kd_.m_numAug));
    Q.fill(0.0);
    Q.diagonal()<<pow(std_a_, 2),pow(std_yawdd_, 2);

    /*******************************************************************************************************************
     *    2. Calculate Sigma Points
     /*****************************************************************************************************************/
    // Calculate Sigma Points
    Eigen::MatrixXd sig(UT::CalculateSigmaPoints(kd_.x, kd_.P, Q));

    // Predict Sigma Points
    kd_.m_sig_pred = UT::PredictSigmaPoints(sig, PredictionModel, &delta_t, kd_.m_numAug);

    /*******************************************************************************************************************
     *    3. Calculate Predicted Mean & covariance
     /*****************************************************************************************************************/
    //calculate the weights
    kd_.m_weights = UT::CalculateWeigts(kd_.m_numSigmaPoints, (kd_.m_numState+kd_.m_numAug));

    // Calculate mean of Sigma Points
    kd_.x= UT::PredictMean(kd_.m_sig_pred, kd_.m_weights);

    // Calculate Covariance Sigma Points
    kd_.P = UT::PredictCovariance(kd_.x, kd_.m_sig_pred, kd_.m_weights, calc_covar);
}

/**
 * @brief UpdateLidar 
 * Updates the state and the state covariance matrix using a laser measurement.
 * 
 * @param[in]  measurement_pack
 *  The measurement pack that contains sensor type time stamp and readings {MeasurementPackage}.
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /*******************************************************************************************************************
     *    extract measurement size
     /*****************************************************************************************************************/
    int measSize(meas_package.raw_measurements_.size());

    /*******************************************************************************************************************
     *    set output matrix H
     /*****************************************************************************************************************/
    Eigen::MatrixXd H(Eigen::MatrixXd(measSize, kd_.m_numState));
    H.setIdentity(measSize, kd_.m_numState);

    /*******************************************************************************************************************
     *    add measurement noise covariance matrix
     /*****************************************************************************************************************/
    Eigen::MatrixXd R(Eigen::MatrixXd(measSize, measSize));
    R.fill(0.0);
    R.diagonal()<<pow(std_laspx_, 2),pow(std_laspy_, 2);

    /*******************************************************************************************************************
     *    Calculate Innovation
     /*****************************************************************************************************************/
    Eigen::VectorXd zpred = H * kd_.x;
    Eigen::VectorXd z_meas = meas_package.raw_measurements_;
    Eigen::VectorXd Y = z_meas - zpred;
    Eigen::MatrixXd S((H * kd_.P * H.transpose()) + R);
    kd_.nis = Y.transpose() * S.inverse() * Y;

    /*******************************************************************************************************************
     *    Calculate Kalman Gain
     /*****************************************************************************************************************/
    Eigen::MatrixXd K(kalmanFilter::CalculateKalmanGain(kd_.P, H, R));

    /*******************************************************************************************************************
     *    Update Linear
     /*****************************************************************************************************************/
    kalmanFilter::update(kd_.x, kd_.P, Y, H, K);
}

/**
 * @brief UpdateRadar 
 * Updates the state and the state covariance matrix using a radar measurement.
 * 
 * @param[in] meas_package
 *  The measurement pack that contains sensor type , time stamp and readings {MeasurementPackage} .
 *
 */
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    /*******************************************************************************************************************
     *    extract measurement size
     /*****************************************************************************************************************/
    int measSize(meas_package.raw_measurements_.size());

    /*******************************************************************************************************************
     *    add measurement noise covariance matrix
     /*****************************************************************************************************************/
    Eigen::MatrixXd R(Eigen::MatrixXd(measSize, measSize));
    R.fill(0.0);
    R.diagonal()<<pow(std_radr_, 2),pow(std_radphi_, 2),pow(std_radrd_, 2);

    /*******************************************************************************************************************
     *    Transform predicted sigma points from state space to measurement space
     /*****************************************************************************************************************/
    Eigen::MatrixXd tSig_pred(UT::TransformPredictedSigmaToMeasurement(kd_.m_sig_pred, measSize, PredictionModelMeasurement));

    /*******************************************************************************************************************
     *    Calculate Predicted Mean & covariance
     /*****************************************************************************************************************/
    // Calculate mean of Sigma Points
    Eigen::VectorXd zpred = UT::PredictMean(tSig_pred, kd_.m_weights);

    // Calculate Covariance Sigma Points
    Eigen::MatrixXd S(UT::PredictCovariance(zpred, tSig_pred, kd_.m_weights, calc_covar_measurement));
    S += R;

    /*******************************************************************************************************************
     *    calculate the kalman Gain
     /*****************************************************************************************************************/
    Eigen::MatrixXd K = UT::CalculateKalmanGainUT(kd_.x, zpred, kd_.m_weights, kd_.m_sig_pred, tSig_pred, S, calc_covar, calc_covar_measurement);

    /*******************************************************************************************************************
     *    update UT
     /*****************************************************************************************************************/
    Eigen::VectorXd z = meas_package.raw_measurements_;
    Eigen::VectorXd Y = z - zpred;
    kd_.nis = Y.transpose() * S.inverse() * Y;
    UT::updateUT(kd_.x, kd_.P, Y, S, K);
}

/***********************************************************************************************************************
 *  Helper Functions Definitions                                                *
 /*********************************************************************************************************************/
/**
 * @brief PredictionModel used to propagate the sigma points throgh the prediction model.
 * 
 * @param[in] sig_pred_
 *  The sigma points calculated by the unscented transform {VectorXd} .
 * 
 * @param[in] args
 *  Extra arguments {const void *}.
 * 
 * @return  Xsig_pred 
 * The propagated sigma points{VectorXd}.
 *
 */
Eigen::VectorXd UKF:: PredictionModel (const Eigen::Ref<const Eigen::VectorXd> &sig_pred_,const void *args)
{
    Eigen::VectorXd Xsig_pred = Eigen::VectorXd(5);
    Xsig_pred.fill(0.0);
    double dt = *(double*)args;
    // Extract values for readability
    double p_x      (sig_pred_(0));
    double p_y      (sig_pred_(1));
    double v        (sig_pred_(2));
    double yaw      (sig_pred_(3));
    double yawd     (sig_pred_(4));
    double nu_a     (sig_pred_(5));
    double nu_yawdd (sig_pred_(6));

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

    double v_p    (v);
    double yaw_p  (yaw + yawd * dt);
    double yawd_p (yawd);

    // add noise
    double dt2 (dt * dt);
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
 * @param[in] p_args
 *  Extra arguments {const void *}.
 * 
 * @return Xsig_pred 
 * The propagated sigma points{VectorXd}.
 *
 */
Eigen::VectorXd UKF:: PredictionModelMeasurement (const Eigen::Ref<const Eigen::VectorXd> &sig_pred_, const void *args)
{
    Eigen::VectorXd Xsig_pred = Eigen::VectorXd(3);
    Xsig_pred.fill(0.0);
    // extract values for better readability
    double p_x(sig_pred_(0));
    double p_y(sig_pred_(1));
    double v  (sig_pred_(2));
    double yaw(sig_pred_(3));

    double vx(v * cos(yaw));
    double vy(v * sin(yaw));

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
    double p_x2(p_x * p_x);
    double p_y2(p_y * p_y);

    double r(sqrt(p_x2 + p_y2));
    double phi(atan2(p_y, p_x));
    double r_dot((p_x * vx + p_y * vy) / r);

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
 * @return x_diff
 *  The state vector after applying the proper rounding to angles{VectorXd}.
 *
 */
Eigen::VectorXd UKF:: calc_covar (const Eigen::Ref<const Eigen::VectorXd>&sig_pred)
{
    Eigen::VectorXd x_diff(sig_pred);

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
 * @return x_diff 
 * The state vector after applying the proper rounding to angles{VectorXd}.
 *
 */
Eigen::VectorXd UKF::calc_covar_measurement (const Eigen::Ref<const Eigen::VectorXd> &sig_pred)
{
    Eigen::VectorXd x_diff(sig_pred);

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