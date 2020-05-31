/*
 * kf_update.cpp
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file kf_update.cpp
 *  @ingroup kf
 *  @brief KfUpdate class
 */

/**
 *  @addtogroup kf
 *  @{
 */

#include <Eigen/Dense>
#include "kf_update.h"

  /**
 * @brief updateLinear, Perform the Prediction step using linear kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 *
 * @param[in] measurement_pack is measurement object  {MeasurementPackage}.
 * 
 * @param[in] Func A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in] HelperFunc A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in]  p  A void pointer is used to pass arguments to the helper function. 
 */
void KfUpdateLinear::update(KalmanData &kd,
                            const MeasurementPackage &measurement_pack,
                             KfUt* const p_kfut,
                            std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func,
                            std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc,
                            std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>gainXFunc,
                            const void *args)
{
	// extract measurement z
	Eigen::VectorXd z_meas = measurement_pack.raw_measurements_;

  //calculate Innovation(Y = z_meas - H*x)
  Eigen::VectorXd Y = Eigen::VectorXd (z_meas.rows());
  Y = z_meas - kd.zpred;

  if(Func!=NULL)
  {
    Y = Func(Y, args);
  }
  else
  {
    /* do no thing */
  }
  
  //calculate Kalman Gain
  Eigen::MatrixXd Ht = kd.H.transpose();
  Eigen::MatrixXd S = (kd.H * kd.P * Ht) + kd.R;
  Eigen::MatrixXd K = kd.P *Ht * S.inverse();

  //calculate NIS_Laser
  double nis = Y.transpose() * S.inverse() * Y;
  kd.NIS = nis;

  // update new estimate
  kd.x = kd.x + (K * Y);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(kd.x.size(), kd.x.size());
  kd.P = (I - K * kd.H) * kd.P;
}


  /**
 * @brief updateLinear, Perform the Prediction step using linear kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 *
 * @param[in] measurement_pack is measurement object  {MeasurementPackage}.
 * 
 * @param[in] KfUt is kalman filter unscented transform object {KfUt}.
 * 
 * @param[in] Func A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in] HelperFunc A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in]  p  A void pointer is used to pass arguments to the helper function. 
 */
void KfUpdateUT::update(KalmanData &kd,
                        const MeasurementPackage &measurement_pack,
                         KfUt* const p_kfut,
                        std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func,
                        std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc,
                        std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>gainXFunc,
                        const void *args)
{
  kd.m_numMeasurements = measurement_pack.raw_measurements_.size();
  p_kfut->setKalmanData(kd,true,true,Func,HelperFunc,args);
  p_kfut->UT();
  Eigen::VectorXd x = kd.x;
  Eigen::VectorXd weights_ = p_kfut->m_weights;
  Eigen::VectorXd z = p_kfut->mean;
  Eigen::MatrixXd S = p_kfut->covar + kd.R;
  Eigen::MatrixXd Xsig_pred_ = p_kfut->m_sig;
  Eigen::MatrixXd Zsig_pred_ = p_kfut->m_sig_pred;
  int sigmaPoints = p_kfut->m_sigmaPoints;

  Eigen::MatrixXd Tc = Eigen::MatrixXd (kd.m_numState, kd.m_numMeasurements);
  Tc.fill(0.0);
  for (int i = 0; i <sigmaPoints; i++)
  {
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x;
    Eigen::VectorXd z_diff = Zsig_pred_.col(i) - z;

    // angle normalization
    if(gainXFunc!=NULL)
    {
      x_diff = gainXFunc(x_diff,NULL);
    }
    else
    {
      /* do nothing */
    }

    if(HelperFunc!=NULL)
    {
      z_diff = HelperFunc(z_diff,NULL);
    }
    else
    {
      /* do nothing */
    }
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  Eigen::MatrixXd K = Tc * S.inverse();

  Eigen::VectorXd Y = measurement_pack.raw_measurements_ - z;
    if(HelperFunc!=NULL)
    {
      Y = HelperFunc(Y,NULL);
    }
    else
    {
      /* do nothing */
    }

  //calculate NIS_Radar
  double nis = Y.transpose() * S.inverse() * Y;
  kd.NIS = nis;

  // update state mean and covariance
  kd.x = x + K * Y;
  kd.P = kd.P - K * S * K.transpose();
}

/**
 *  @}
 */

