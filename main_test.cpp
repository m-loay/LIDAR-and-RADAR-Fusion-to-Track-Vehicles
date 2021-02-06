/*
 * TestCase.cpp
 *
 *  Created on: Apr 24, 2019
 *      Author: mloay
 */

#include <gtest/gtest.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include"test_cases.h"

using namespace std;
using namespace Eigen;

TEST(KFTest,linearFilter)
{
    EXPECT_EQ(linearFilter(),true);
}

TEST(KFTest,trackLinearFilter)
{
    EXPECT_EQ(trackLinearFilter(),true);
}

TEST(EKFTest,CalculateJacobian)
{
    EXPECT_EQ(CalculateJacobian(),true);
}

TEST(EKFTest,trackEKF)
{
    EXPECT_EQ(trackEKF(),true);
}

TEST(RMSE,calculateRMSE)
{
    EXPECT_EQ(calculateRMSE(),true);
}

// TEST(UTTEST,CalculateSigmaPoints)
// {
//     EXPECT_EQ(CalculateSigmaPoints(),true);
// }

// TEST(UTTEST,CalculateSigmaPointsAug)
// {
//     EXPECT_EQ(CalculateSigmaPointsAug(),true);
// }

// TEST(UTTEST,CalculateSigmaPointsAugPred)
// {
//     EXPECT_EQ(CalculateSigmaPointsAugPred(),true);
// }

// TEST(UTTEST,CalculateSigmaPointsMeanCovar)
// {
//     EXPECT_EQ(CalculateSigmaPointsMeanCovar(),true);
// }

// TEST(UTTEST,CalculateSigmaPointsMeanCovar2)
// {
//     EXPECT_EQ(CalculateSigmaPointsMeanCovar2(),true);
// }

// TEST(UTTEST,CalculateMeasSigmaPointsMeanCovar)
// {
//     EXPECT_EQ(CalculateMeasSigmaPointsMeanCovar(),true);
// }

// TEST(UTTEST,CalculateMeanCovarUT1)
// {
//     EXPECT_EQ(CalculateMeanCovarUT1(),true);
// }

// TEST(UTTEST,CalculateMeanCovarUT2)
// {
//     EXPECT_EQ(CalculateMeanCovarUT2(),true);
// }

// TEST(UTTEST,PredictionUT)
// {
//     EXPECT_EQ(PredictionUT(),true);
// }

// TEST(UTTEST,PredictionUTKFLIB)
// {
//     EXPECT_EQ(PredictionUTKFLIB(),true);
// }

// TEST(UTTEST,MeasPredictionUT)
// {
//     EXPECT_EQ(MeasPredictionUT(),true);
// }

// TEST(UTTEST,PredictionMeasUTKFLIB)
// {
//     EXPECT_EQ(PredictionMeasUTKFLIB(),true);
// }

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

