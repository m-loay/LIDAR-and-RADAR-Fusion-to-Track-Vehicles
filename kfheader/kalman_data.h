/*
 * kalman_data.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file kalman_data.h
 *  @ingroup kfheader
 *  @brief KalmanData class
 */

/**
 *  @addtogroup kfheader
 *  @{
 */

#ifndef KALMAN_DATA_PACKAGE_H_
#define KALMAN_DATA_PACKAGE_H_

#include <Eigen/Dense>
#include <vector>

class KalmanData
{
    public:
        void setKalmanData(int numState)
        {
            //save number of states
            m_numState = numState;

            //create a 4D state vector, we don't know yet the values of the x state
            x = Eigen::VectorXd(m_numState);
            x.fill(0.0);

            //state covariance matrix P
            P = Eigen::MatrixXd::Identity(m_numState, m_numState);
            P.diagonal().fill(0.1);
        }

        Eigen::VectorXd x;	// object state
        Eigen::MatrixXd P;	// object covariance matrix
        int m_numState;
        double nis;
};

class KalmanDataUT
{
    public:
        void setKalmanData(int numState, int numAug)
        {
            //save number of states
            m_numState = numState;

            //save number of Augmented states
            m_numAug = numAug;

            m_numSigmaPoints = 2 * (m_numState + numAug) +1;

            //create a 4D state vector, we don't know yet the values of the x state
            x = Eigen::VectorXd(m_numState);
            x.fill(0.0);

            //state covariance matrix P
            P = Eigen::MatrixXd::Identity(m_numState, m_numState);
            P.diagonal().fill(0.1);
        }

        Eigen::VectorXd x;	// object state
        Eigen::MatrixXd P;	// object covariance matrix
        Eigen::MatrixXd m_sig_pred;	// object covariance matrix
        Eigen::MatrixXd m_weights;	// object covariance matrix
        int m_numState;
        int m_numAug;
        int m_numSigmaPoints; 
        double nis;
};

#endif /* KALMAN_DATA_PACKAGE_H_ */
/**
 *  @}
 */
