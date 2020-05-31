/*
 * kf_ut.cpp.cpp
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file kf_ut.cpp
 *  @ingroup kf
 *  @brief KfUt  class
 */

/**
 *  @addtogroup kf
 *  @{
 */

#include <Eigen/Dense>
#include "kf_ut.h"

  /**
 * @brief setKalmanData,  set Kalman filter data for unscented transform.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 * 
 * @param[in] use_predicted_points an input boolean to re-use the predicte sigma points {bool}.
 * 
 * @param[in] is_update_cycle an input boolean to define kalman filter state (predict/update) {bool}.
 * 
 * @param[in] Func A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in] HelperFunc A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in]  p  A void pointer is used to pass arguments to the helper function.
 *
 */
void KfUt::setKalmanData(KalmanData &kd,
                         bool use_predicted_points,
                         bool is_update_cycle,
                         std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)> Func,
                         std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)> HelperFunc,
                         const void *args)
{
  if(is_update_cycle && !use_predicted_points)
  {
    m_numUTState = kd.m_numMeasurements;
    m_numAug = kd.m_numMeasurements;
  }
  else
  {
    m_numUTState = kd.m_numState;
    m_numAug = kd.m_numState + kd.m_numAddedAug;
  }

  //store functions used & arguments
  m_Func = Func;
  m_HelperFunc = HelperFunc;
  m_args = args;
  m_use_predicted_points = use_predicted_points;

  //calculate lambda and sigma points
  m_lambda = 3 - m_numAug;
  m_sigmaPoints = 2*m_numAug +1;

  if(!use_predicted_points)
  {

    ///* Weights of sigma points
    m_weights = Eigen::VectorXd(m_sigmaPoints);
    m_weights.fill(1 / float(2 *(m_lambda + m_numAug)));
    m_weights(0) = m_lambda /float(m_lambda + m_numAug);

    // Create augmented mean vector and covariance matrix
    x_aug = Eigen::VectorXd(m_numAug);
    P_aug = Eigen::MatrixXd(m_numAug,m_numAug);

    if(is_update_cycle)
    {
      x_aug.fill(0.0);
      x_aug.head(m_numAug) = kd.zpred;

      P_aug.fill(0.0);
    }
    else
    {
      //in update cycle x-->z , Q --> S
      x_aug.fill(0.0);
      x_aug.head(m_numUTState) = kd.x;

      P_aug.fill(0.0);
      P_aug.topLeftCorner(m_numUTState,m_numUTState) = kd.P;
    }

    //check if there is an augmented states
    if(m_numAug > m_numUTState && !is_update_cycle)
    {
      int diff =  m_numAug - m_numUTState;
      P_aug.bottomRightCorner(diff , diff) = kd.Q;
    }
    else
    {
      /* do nothing */
    }

    //Initialize mean, covar, sigma points
    mean = Eigen::VectorXd(m_numUTState);
    mean.fill(0);
    covar = Eigen::MatrixXd(m_numUTState,m_numUTState);
    covar.fill(0);
  }
  else
  {
    //Initialize mean, covar, sigma points
    mean = Eigen::VectorXd(kd.m_numMeasurements);
    mean.fill(0);

    covar = Eigen::MatrixXd(kd.m_numMeasurements,kd.m_numMeasurements);
    covar.fill(0);

    m_sig = Eigen::MatrixXd(m_numUTState, m_sigmaPoints);
    m_sig = m_sig_pred;
    m_sig_pred = Eigen::MatrixXd(kd.m_numMeasurements, m_sigmaPoints);

    //for prediction calculation using prediction sigma points
    m_numAug = m_numUTState;
    m_numUTState = kd.m_numMeasurements;
  }
}
/**
 * @brief UT, Perform the Unscented transform after setting the correct data using setKalmanData .
 */
void KfUt:: UT()
{
  if(!m_use_predicted_points)
  {
    CalculateSigmaPoints();
  }
  else
  {
    /* do nothing*/
  }
  
  PredictSigmaPoints();
  CalculateMean();
  CalculateCovariance();
}

/**
 * @brief CalculateSigmaPoints, Calculate the sigma Points based on UT.
 *
 */
void KfUt:: CalculateSigmaPoints()
{
  // Create square root matrix
  Eigen::MatrixXd L = P_aug.llt().matrixL();

  //create sigma points
  m_sig = Eigen::MatrixXd(m_numAug,m_sigmaPoints);
  m_sig.col(0) = x_aug;
  for (int i = 0; i < m_numAug; i++)
  {
    m_sig.col(i+1)                   = x_aug + sqrt(m_lambda + m_numAug) * L.col(i);
    m_sig.col(i+1 + m_numAug) = x_aug - sqrt(m_lambda + m_numAug) * L.col(i);
  }
}

/**
 * @brief PredictSigmaPoints predict sigma points using prediction model provided through setKalmanData.
 *
 */

void KfUt:: PredictSigmaPoints()
{
  m_sig_pred = Eigen::MatrixXd(m_numUTState,m_sigmaPoints);

  for (int i = 0; i < m_sigmaPoints ; i++)
  {
    Eigen::VectorXd col(m_numAug);
    col = m_sig.col(i);
    m_sig_pred.col(i) =m_Func(col,m_args);
  }
}

/**
 * @brief CalculateMean, Calculate the mean from predicted Sigma Points.
 *
 *
 */
void KfUt::CalculateMean()
{
  Eigen::VectorXd z_pred_old = Eigen::VectorXd(3);
  Eigen::VectorXd z_sig = Eigen::VectorXd(3);
  //iterate over the all sigma points
  for (int i = 0; i < m_sigmaPoints ; i++)
  {
    z_pred_old = mean;
    z_sig = m_sig_pred.col(i);
    mean = mean + m_weights(i) * m_sig_pred.col(i);
  }
}

/**
 * @brief CalculateCovariance, Calculate the covariance from predicted Sigma Points.
 *
 *
 */
void KfUt::CalculateCovariance()
{
  for (int i = 0; i < m_sigmaPoints; i++)
  {
    // state difference
    Eigen::VectorXd x_diff = m_sig_pred.col(i) - mean;
    if(m_HelperFunc != NULL)
    {
      x_diff = m_HelperFunc(x_diff,m_args);
    }
    else
    {
      /*do nothing*/
    }    
    covar = covar+ m_weights(i) * x_diff * x_diff.transpose();
  }
}


/**
 *  @}
 */

