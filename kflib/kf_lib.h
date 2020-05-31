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

#ifndef SRC_KF_H_
#define SRC_KF_H_
#include <Eigen/Dense>
#include <vector>
#include "KalmanConfig.h"
#include"kf_predict.h"
#include"kf_update.h"
#include"kf_ut.h"
#include"measurement_package.h"

class KfLib
{
public:
/**
 * @brief kflib, Kalman filter destructor.
 */
  virtual ~KfLib ()
  {
    // delete m_kfPred;
    // delete m_kfUpdate;
    // delete m_KfUt;
    // delete m_kfUpdateExt;
  }

/**
 * @brief setMode, set Kalman filter (prediction type , update type).
 *
 * @param[in] kfPred is kalman filter prediction pointer object {KfPredict}.
 *
 * @param[in] kfPred is kalman filter update pointer object {KfUpdate}.
 * 
 * @param[in] KfUt is kalman filter unscented transform object {KfUt}.
 * 
 * @param[in] kfUpdateExt is kalman filter extra update pointer object {KfUpdate}.  
 */
  void setMode(KfPredict* const kfPred, KfUpdate* const kfUpdate, KfUt* const KfUt=NULL, KfUpdate* const kfUpdateExt=NULL )
  {
    m_kfPred = kfPred;
    m_kfUpdate = kfUpdate;
    m_KfUt = KfUt;
    m_kfUpdateExt = kfUpdateExt;
  }

/**
 * @brief predict, Perform the Prediction step in kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 *
 */
  void predict(KalmanData &kd)
  {
    m_kfPred->predict(kd, m_KfUt);
  }

  /**
 * @brief predict, Perform the Prediction step in kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 * 
 * @param[in] Func A call-back function used to calculate the prediction model  {VectorXd:(VectorXd,const void *p)}.
 *
 */
  void predict(KalmanData &kd,
               std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func)
  {
    m_kfPred->predict(kd, m_KfUt, Func);
  }

  /**
 * @brief predict, Perform the Prediction step in kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 * 
 * @param[in] Func A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in] HelperFunc A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 *
 */
  void predict(KalmanData &kd,
               std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func,
               std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc)
  {
    m_kfPred->predict(kd, m_KfUt, Func, HelperFunc);
  }

  /**
 * @brief predict, Perform the Prediction step in kalman filter.
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
  void predict(KalmanData &kd,
               std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func,
               std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc,
               const void *args)
  {
    m_kfPred->predict(kd, m_KfUt, Func,HelperFunc,args);
  }
  
/**
 * @brief update, Perform the update step in kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 *
 * @param[in] measurement_pack is measurement object  {MeasurementPackage}. 
 */
  void update(KalmanData &kd, const MeasurementPackage &measurement_pack)
  {
      if (m_KfUt != NULL && m_kfUpdateExt != NULL)
      {
          m_kfUpdateExt->update(kd, measurement_pack, NULL);
      }
      else
      {
          m_kfUpdate->update(kd, measurement_pack, m_KfUt);
      }
  }

  /**
 * @brief update, Perform the update step in kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 *
 * @param[in] measurement_pack is measurement object  {MeasurementPackage}.
 * 
 * @param[in] Func A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}. 
 */
  void update(KalmanData &kd, 
              const MeasurementPackage &measurement_pack,
              std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func)
  {
    if(m_KfUt != NULL && m_kfUpdateExt != NULL)
    {
      m_kfUpdateExt->update(kd,measurement_pack,NULL,Func);
    }
    else
    {
      m_kfUpdate->update(kd, measurement_pack,m_KfUt,Func);
    }
  }

  /**
 * @brief update, Perform the update step in kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 *
 * @param[in] measurement_pack is measurement object  {MeasurementPackage}.
 * 
 * @param[in] Func A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in] HelperFunc A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}. 
 */
  void update(KalmanData &kd, 
              const MeasurementPackage &measurement_pack,
              std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func,
              std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc)
  {
    m_kfUpdate->update(kd, measurement_pack,m_KfUt,Func,HelperFunc);
  }

  /**
 * @brief update, Perform the update step in kalman filter.
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
  void update(KalmanData &kd, 
              const MeasurementPackage &measurement_pack,
              std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func,
              std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc,
              const void *args)
  {
    m_kfUpdate->update(kd, measurement_pack,m_KfUt,Func,HelperFunc,NULL,args);
  }

  /**
 * @brief update, Perform the update step in kalman filter.
 *
 * @param[in,out] kd is kalman data object  {KalmanData}.
 *
 * @param[in] measurement_pack is measurement object  {MeasurementPackage}. 
 * 
 * @param[in] Func A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in] HelperFunc A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in] gainXFunc A call-back function used to calculate the prediction model {VectorXd:(VectorXd,const void *p)}.
 * 
 * @param[in]  p  A void pointer is used to pass arguments to the helper function.
 */
  void update(KalmanData &kd, 
              const MeasurementPackage &measurement_pack,
              std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func,
              std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc,
              std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>gainXFunc,
              const void *args)
  {
    m_kfUpdate->update(kd, measurement_pack,m_KfUt,Func,HelperFunc,gainXFunc,args);
  }

private:
  ///* base pointer from prediction
  KfPredict *m_kfPred;

  ///* base pointer from update
  KfUpdate *m_kfUpdate;

  ///* base pointer from update
  KfUpdate *m_kfUpdateExt;

//* base pointer from UT
  KfUt *m_KfUt;
};

#endif /* SRC_KF_H_ */
/**
 *  @}
 */
