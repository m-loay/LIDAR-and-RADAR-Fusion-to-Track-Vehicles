/*
 * kfApp.h
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

#ifndef SRC_KF_APP_H_
#define SRC_KF_APP_H_
#include <Eigen/Dense>
#include <cstdint>
#include "KalmanConfig.h"
#include "kalman_data.h"
#include "measurement_package.h"
#ifdef USE_UNIT_TESTING
#define private public
#endif

class kfApp
{
public:
    ///*Constructor.
    kfApp(int num_states);

    ///*/Destructor.
    virtual ~kfApp();

    ///* instance of kalman data
    KalmanData kd_;

    ///*Run the whole flow of the Kalman Filter from here.
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    // Prediction Predicts the state and the state covariance.
    // using the process model.
    // @param delta_T Time between k and k+1 in s.
    void Prediction(const double delta_t);

    // Updates the state and the state covariance matrix using a laser measurement.
    // @param meas_package The measurement at k+1.
    void UpdateLidar(MeasurementPackage meas_package);

    // Updates the state and the state covariance matrix using a radar measurement.
    //@param meas_package The measurement at k+1.
    void UpdateRadar(MeasurementPackage meas_package);

    enum kfData
    {
        XPOS=0,YPOS,XVEL,YVEL
    } kfData_;

private:
    ///* initially set to false, set to true in first call of ProcessMeasurement.
    bool is_initialized_;

    ///* set time to be zero initially.
    int64_t previous_timestamp_;
    
    ///* Process noise standard deviation longitudinal acceleration in m/s^2.
    float noise_ax;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2.
    float noise_ay;

    ///* Laser measurement noise standard deviation position1 in m.
    float std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m.
    float std_laspy_;
    ///* Radar measurement noise standard deviation radius in m.
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad.
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s.
    double std_radrd_ ;

    // calculates the mean state vector based dynamic model.
    //@param x The state vector.
    //@param[in] kd an object contains all kalman data {KalmanData}.
    //@return the mean state vector.
    static Eigen::VectorXd g_(const Eigen::VectorXd &mean, const void *p_args=NULL);

    // calculates the derivative of g_function.
    //@param x The state vector.
    //@param[in] kd an object contains all kalman data {KalmanData}.
    //@return the state transition matrix.
    static Eigen::MatrixXd g_prime_(const Eigen::VectorXd &mean, const void *p_args=NULL);

    // calculates the mean state vector for measurements based on sensor model.
    //@param x The state vector.
    //@param size The measurement vector.
    //@return the mean state predicted vector.
    Eigen::VectorXd h_(const Eigen::VectorXd &x , size_t size);
    
    // Calculate the derivative of h_function.
    //@param x_state The state vector.
    //@return the state transition matrix of measurements.
    Eigen::MatrixXd h_prime_(const Eigen::VectorXd& x_state);

    // Innovation helper function for angle rounding issues.
    //@param  sig_pred The sigma points vector .
    //@return z_diff The innovation vector after rounding fixes.
    Eigen::VectorXd Innovationhelper(const Eigen::VectorXd &sig_pred);

};

#endif /* SRC_KF_APP_H_ */
/**
 *  @}
 */

