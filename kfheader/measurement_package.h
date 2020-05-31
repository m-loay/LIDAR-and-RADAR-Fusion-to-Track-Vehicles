/*
 * measurement_package.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file measurement_package.h
 *  @ingroup kfheader
 *  @brief measurement_package class
 */

/**
 *  @addtogroup kfheader
 *  @{
 */

#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
  long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
/**
 *  @}
 */