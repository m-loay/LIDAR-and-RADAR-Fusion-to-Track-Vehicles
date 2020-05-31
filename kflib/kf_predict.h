/*
 * kf_predict.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */
/** @file kf_predict.h
 *  @ingroup kf
 *  @brief KfPredictf class
 */

/**
 *  @addtogroup kf
 *  @{
 */

#ifndef KF_PREDICT_H_
#define KF_PREDICT_H_

#include"kalman_data.h"
#include"kf_ut.h"

//Virtual Class KfPredict
class KfPredict
{
public:
  virtual ~KfPredict() { /* ... */ }
  virtual void predict(KalmanData &kd,
                       KfUt* const p_kfut=NULL,
                       std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func=NULL,
                       std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc=NULL,
                       const void *args = NULL)=0;
};


//Concrete Class KfPredict --> KfPredictLinear
class KfPredictLinear : public KfPredict
{
public:
  ~KfPredictLinear() { /* ... */ }
  
  void predict(KalmanData &kd,
               KfUt* const p_kfut=NULL,
               std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func=NULL,
               std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc=NULL,
               const void *args = NULL);
};

//Concrete Class KfPredict --> KfPredictUT
class KfPredictUT : public KfPredict
{
public:
  ~KfPredictUT() { /* ... */ }
  
  void predict(KalmanData &kd,
               KfUt* const p_kfut=NULL ,
               std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func=NULL,
               std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc=NULL,
               const void *args = NULL);
};

#endif /* KF_PREDICT_H_ */
/**
 *  @}
 */
