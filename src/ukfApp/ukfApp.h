/*
 * ukfApp.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file ukfApp.h
 *  @ingroup ukfApp
 *  @brief ukfApp class
 */

/**
 *  @addtogroup ukfApp
 *  @{
 */
#ifndef UKF_H
#define UKF_H

#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include "kalman_data.h"
#include "UT.h"
#include "kalmanFilter.h"
#include "measurement_package.h"
using namespace Eigen;

class ukfApp
{
    public:

        ///*detructor 
        ukfApp(){}

        ///*Constructor
        ukfApp(int num_states ,int aug_states);

        ///*/Destructor
        virtual ~ukfApp();

        ///* instance of kalman data
        KalmanDataUT kd_;

        ///*Run the whole flow of the Kalman Filter from here.
        void ProcessMeasurement(const MeasurementPackage &measurement_pack);

        /**
         * Predicts sigma points, the state, and the state covariance matrix.
         * @param {double} delta_t the change in time (in seconds) between the last
         * measurement and this one.
         */
        void Prediction(const double delta_t);

        /**
         * Updates the state and the state covariance matrix using a laser measurement
         * @param meas_package The measurement at k+1
         */
        void UpdateLidar(MeasurementPackage meas_package);

        /**
         * Updates the state and the state covariance matrix using a radar measurement
         * @param meas_package The measurement at k+1
         */
        void UpdateRadar(MeasurementPackage meas_package);

        enum kfData
        {
            XPOS=0,YPOS,VEL,THETA,THETAD
        } kfData_;

    private:
        ///* initially set to false, set to true in first call of ProcessMeasurement
        bool is_initialized_;

        ///* set time to be zero initially
        double previous_timestamp_;

        ///* Run the whole flow of the Kalman Filter from here. Process noise standard deviation longitudinal acceleration in m/s^2
        double std_a_;

        ///* Run the whole flow of the Kalman Filter from here. Process noise standard deviation yaw acceleration in rad/s^2
        double std_yawdd_;

        ///*Run the whole flow of the Kalman Filter from here. Laser measurement noise standard deviation position1 in m
        double std_laspx_;

        ///* Run the whole flow of the Kalman Filter from here. Laser measurement noise standard deviation position2 in m
        double std_laspy_;

        ///* Run the whole flow of the Kalman Filter from here.Radar measurement noise standard deviation radius in m
        double std_radr_;

        ///*Run the whole flow of the Kalman Filter from here. Radar measurement noise standard deviation angle in rad
        double std_radphi_;

        ///*Run the whole flow of the Kalman Filter from here. Radar measurement noise standard deviation radius change in m/s
        double std_radrd_;

        // PredictionModel a model to propagate sigma points.
        // @param args used to help the advance state of the model.
        static Eigen::VectorXd PredictionModel (const Eigen::Ref<const Eigen::VectorXd> &,const void *args=NULL);

        // calc_covar helper function for covarince calculation in prediction step.
        // @param colum vector.
        static Eigen::VectorXd calc_covar (const Eigen::Ref<const Eigen::VectorXd> &);

        // PredictionModelMeasurement a sensor model to propagate sigma points.
        // @param args used to help the advance state of the model.
        static Eigen::VectorXd PredictionModelMeasurement (const Eigen::Ref<const Eigen::VectorXd>&, const void* p = NULL);

        // calc_covar_measurement helper function for covarince calculation in update step.
        // // @param colum vector.
        static Eigen::VectorXd calc_covar_measurement (const Eigen::Ref<const Eigen::VectorXd> &);

        //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer
};

#endif /* UKF_H */
/**
 *  @}
 */
