/*
 * kf_update.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */
/** @file kf_update.h
 *  @ingroup kf
 *  @brief KfUpdate class
 */

/**
 *  @addtogroup kf
 *  @{
 */

#ifndef KF_UPDATE_H_
#define KF_UPDATE_H_

#include"kalman_data.h"
#include "kf_ut.h"
#include"measurement_package.h"
#include<functional>
#include <Eigen/Dense>
//Virtual Class KfUpdate
class KfUpdate
{
public:
  virtual ~KfUpdate() { /* ... */ }
  virtual void update(KalmanData &kd,
                      const MeasurementPackage &measurement_pack,
                      KfUt* const p_kfut=NULL,
                      std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func=NULL,
                      std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc=NULL,
                      std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>gainXFunc=NULL,
                      const void *args = NULL)=0;
};


//Concrete Class KfUpdate --> KfUpdateLinear
class KfUpdateLinear : public KfUpdate
{
public:
  ~KfUpdateLinear() { /* ... */ }
  
void update(KalmanData &kd,
            const MeasurementPackage &measurement_pack,
            KfUt* const p_kfut=NULL,
            std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func=NULL,
            std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc=NULL,
            std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>gainXFunc=NULL,
            const void *args = NULL);
};

//Concrete Class KfUpdate --> KfUpdateUT
class KfUpdateUT : public KfUpdate
{
public:
  ~KfUpdateUT() { /* ... */ }
  
void update(KalmanData &kd,
            const MeasurementPackage &measurement_pack,
            KfUt* const p_kfut=NULL,
            std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>Func=NULL,
            std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>HelperFunc=NULL,
            std::function<Eigen::VectorXd(const Eigen::VectorXd, const void *p)>gainXFunc=NULL,
            const void *args = NULL);
};

#endif /* KF_UPDATE_H_ */
/**
 *  @}
 */
