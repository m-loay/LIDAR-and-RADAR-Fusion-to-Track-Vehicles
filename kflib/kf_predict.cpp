/*
 * kf_predict.cpp
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file kf_predict.cpp
 *  @ingroup kf
 *  @brief KfPredictf class
 */

/**
 *  @addtogroup kf
 *  @{
 */

#include "kf_predict.h"

  /**
 * @brief PredictLinear, Perform the Prediction step using linear kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 * 
 * @param[in] Func A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in] HelperFunc A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in]  p  A void pointer is used to pass arguments to the helper function.
 *
 */
void KfPredictLinear::predict(KalmanData &kd,
                              KfUt *p_kfut ,
                              std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func,
                              std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc,
                              const void *args)
{
  kd.x = kd.F * kd.x;
  kd.P = (kd.F* kd.P * kd.F.transpose()) + kd.Q;
}

  /**
 * @brief PredictionUT, Perform the Prediction step using Unsented kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 * 
 * @param[in] KfUt is kalman filter unscented transform object {KfUt}.
 * 
 * @param[in] Func A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in] HelperFunc A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in]  p  A void pointer is used to pass arguments to the helper function.
 *
 */
void KfPredictUT::predict(KalmanData &kd,
                          KfUt *p_kfut ,
                          std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func,
                          std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc,
                          const void *args)
{
  p_kfut->setKalmanData(kd,false,false,Func,HelperFunc,args);
  p_kfut->UT();
  kd.x = p_kfut->mean;
  kd.P = p_kfut->covar;
}

/**
 *  @}
 */

