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
#include "kf_lib.h"
#include "tools.h"
#include "kf_ut.h"
#include "../kfApp/kfApp.h"

//TestCase1
bool linearFilter(void);
//TestCase2
bool trackLinearFilter(void);

//TestCase3
bool CalculateJacobian(void);

//TestCase4
bool calculateRMSE(void);

//TestCase5
bool CalculateSigmaPoints(void);

//TestCase6
bool CalculateSigmaPointsAug(void);

//TestCase7
bool CalculateSigmaPointsAugPred(void);

//TestCase8
bool CalculateSigmaPointsMeanCovar(void);

//TestCase9
bool CalculateSigmaPointsMeanCovar2(void);

//TestCase10
bool CalculateMeasSigmaPointsMeanCovar(void);

//TestCase11
bool CalculateMeanCovarUT1(void);

//TestCase12
bool CalculateMeanCovarUT2(void);

//TestCase13
bool PredictionUT(void);

//TestCase14
bool PredictionUTKFLIB(void);

//TestCase15
bool MeasPredictionUT(void);

//TestCase16
bool PredictionMeasUTKFLIB(void);

#endif /* TEST_CASES_H_ */
/**
 *  @}
 */

