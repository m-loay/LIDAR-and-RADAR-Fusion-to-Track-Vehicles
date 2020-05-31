/*
 * kf_ut.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */
/** @file kf_ut.h
 *  @ingroup kf
 *  @brief KfUt  class
 */

/**
 *  @addtogroup kf
 *  @{
 */

#ifndef KF_UT_H_
#define KF_UT_H_

#include"kalman_data.h"
#include"measurement_package.h"
#include<functional>
#include <Eigen/Dense>

#if defined(USE_UNIT_TESTING)
#define private public
#endif

// Class KfUt
class KfUt
{
public:
  ~KfUt() { /* ... */ }
  void UT();

 void setKalmanData(KalmanData &kd,
                     bool use_predicted_points,
                     bool is_update_cycle,
                     std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)> Func,
                     std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)> HelperFunc,
                     const void *args);

  Eigen::VectorXd getUtMean();
  Eigen::MatrixXd getUtCovariance();

  Eigen::VectorXd mean;	// mean vector
  Eigen::MatrixXd covar;	// covariance matrix
  Eigen::VectorXd x_aug;	// mean vector
  Eigen::MatrixXd P_aug;	// covariance matrix
  Eigen::MatrixXd m_sig;	// sigma points
  Eigen::MatrixXd m_sig_pred;	// predicted sigma points
  Eigen::VectorXd m_weights;	// weights

  int m_numUTState;
  int m_numAug;
  int m_sigmaPoints;
  int m_lambda;
  bool m_use_predicted_points;

  std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)> m_Func;
  std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)> m_HelperFunc;
  const void *m_args;


private:
  void CalculateSigmaPoints();
  void PredictSigmaPoints();
  void CalculateMean();
  void CalculateCovariance();
};

#endif /* KF_UT_H_ */
/**
 *  @}
 */
