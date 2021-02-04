/*
 * kf_lib.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */
/** @file kf_lib.h
 *  @ingroup kf
 *  @brief kf class
 */

/**
 *  @addtogroup kf
 *  @{
 */

#ifndef KALMAN_FILER_H_
#define KALMAN_FILER_H_
#include <Eigen/Dense>
#include <vector>
#include "KalmanConfig.h"
#include"kalman_data.h"

class kalmanFilter
{
public:
/**
 * @brief predict, Perform the Prediction step in kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 *
 */
static void predict(KalmanData &kd,
               const Eigen::MatrixXd &Q,
               std::function<Eigen::VectorXd(const KalmanData &, const void *p)>g,
               std::function<Eigen::MatrixXd(const KalmanData &, const void *p)>g_prime,
               const void *args)
  {
    kd.x = g(kd,args);
    Eigen::MatrixXd G = g_prime(kd,args);
    kd.P = (G* kd.P * G.transpose());
  }

static Eigen::MatrixXd CalculateKalmanGain(KalmanData &kd,
               const Eigen::MatrixXd &H,
               const Eigen::MatrixXd &R)
  {
    Eigen::MatrixXd Ht = H.transpose();
    Eigen::MatrixXd S = (H * kd.P * Ht) + R;
    Eigen::MatrixXd K = kd.P *Ht * S.inverse();
    return K;
  }

  static void update(KalmanData &kd,
               const Eigen::VectorXd &Y,
               const Eigen::MatrixXd &H,
               const Eigen::MatrixXd &K)
  {
    kd.x = kd.x + (K * Y);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(kd.x.size(), kd.x.size());
    kd.P = (I - K * H) * kd.P;
  }
 

  
/**
 * @brief update, Perform the update step in kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 *
 * @param[in] measurement_pack is measurement object  {MeasurementPackage}. 
 */
  // void update(KalmanData &kd)
  // {


  // };
};

#endif /* KALMAN_FILER_H_ */
/**
 *  @}
 */
