/*
 * ground_truth_package.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file ground_truth_package.h
 *  @ingroup kfheader
 *  @brief ground_truth class
 */

/**
 *  @addtogroup kfheader
 *  @{
 */

#ifndef SRC_GROUND_TRUTH_PACKAGE_H_
#define SRC_GROUND_TRUTH_PACKAGE_H_

#include <Eigen/Dense>

class GroundTruthPackage
{
public:
  long timestamp_;

  Eigen::VectorXd gt_values_;

};

#endif /* SRC_GROUND_TRUTH_PACKAGE_H_ */
/**
 *  @}
 */
