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

//Concrete Class KfPredict --> KfPredictLinear
class KalmanData
{
public:
  void setKalmanData(int numState,int numAddedAug=0, int numMeasurements=0)
  {
    //save number of states
    m_numState = numState;

    //save number of states
    m_numAddedAug = numAddedAug;

    //save number of Measurements
    m_numMeasurements = numMeasurements;

    //create a 4D state vector, we don't know yet the values of the x state
    x = Eigen::VectorXd(m_numState);
    x.fill(0.0);

    //state covariance matrix P
    P = Eigen::MatrixXd::Identity(m_numState, m_numState);
    P.diagonal().fill(0.1);

    //Fill state transition matrix
    F = Eigen::MatrixXd::Identity(m_numState, m_numState);

    //create a 4D state vector, we don't know yet the values of the x state
    zpred = Eigen::VectorXd(m_numMeasurements);
    zpred.fill(0.0);
  }

  Eigen::VectorXd x;	// object state
  Eigen::MatrixXd P;	// object covariance matrix
  Eigen::MatrixXd F;  // state transition matrix
  Eigen::MatrixXd H;	// measurement matrix
  Eigen::MatrixXd R;	// measurement covariance matrix
  Eigen::MatrixXd Q;	// process covariance matrix
  Eigen::VectorXd zpred;	// prediction of measurement model
  double NIS;
  int m_numState;
  int m_numMeasurements;
  int m_numAddedAug;
};

#endif /* KALMAN_DATA_PACKAGE_H_ */
/**
 *  @}
 */
