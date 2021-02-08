/*
 * test_cases.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */
/** @file test_cases.h
 *  @ingroup testCases
 *  @brief test cases
 */

/**
 *  @addtogroup testCases
 *  @{
 */
#ifndef TEST_CASES_H_
#define TEST_CASES_H_

#include "ground_truth_package.h"
#include "measurement_package.h"
#include "kalman_data.h"
#include "tools.h"
#include "../kfApp/kfApp.h"

//TestCase1
bool linearFilter(void);
//TestCase2
bool trackLinearFilter(void);

//TestCase3
bool CalculateJacobian(void);

//TestCase4
bool trackEKF(void);

//TestCase5
bool calculateRMSE(void);

//TestCase6
bool CalculateSigmaPointsNoAugmentation(void);

//TestCase7
bool CalculateSigmaPointsWithAugmentation(void);

//TestCase8
bool CalculateSigmaPointsAugPred(void);

//TestCase9
bool CalculateSigmaPointsMeanCovar(void);

//TestCase10
bool PredictUT(void);

//TestCase11
bool CalculateMeasurementsMeanCovar(void);

//TestCase12
bool UpdateUT(void);

#endif /* TEST_CASES_H_ */
/**
 *  @}
 */

