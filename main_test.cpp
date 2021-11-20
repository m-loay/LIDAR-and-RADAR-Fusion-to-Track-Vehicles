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

GTEST_TEST(KFTest,linearFilter)
{
    EXPECT_EQ(linearFilter(),true);
}

GTEST_TEST(KFTest,trackLinearFilter)
{
    EXPECT_EQ(trackLinearFilter(),true);
}

GTEST_TEST(EKFTest,CalculateJacobian)
{
    EXPECT_EQ(CalculateJacobian(), true);
}

GTEST_TEST(EKFTest,trackEKF)
{
    EXPECT_EQ(trackEKF(),true);
}

GTEST_TEST(RMSE,calculateRMSE)
{
    EXPECT_EQ(calculateRMSE(),true);
}

GTEST_TEST(UTTEST,CalculateSigmaPoints)
{
    EXPECT_EQ(CalculateSigmaPointsNoAugmentation(),true);
}

GTEST_TEST(UTTEST,CalculateSigmaPointsAug)
{
    EXPECT_EQ(CalculateSigmaPointsWithAugmentation(),true);
}

GTEST_TEST(UTTEST,CalculateSigmaPointsAugPred)
{
    EXPECT_EQ(CalculateSigmaPointsAugPred(),true);
}

GTEST_TEST(UTTEST,CalculateSigmaPointsMeanCovar)
{
    EXPECT_EQ(CalculateSigmaPointsMeanCovar(),true);
}

GTEST_TEST(UTTEST,PredictUT)
{
    EXPECT_EQ(PredictUT(),true);
}

GTEST_TEST(UTTEST,CalculateMeasurementsMeanCovar)
{
    EXPECT_EQ(CalculateMeasurementsMeanCovar(),true);
}

GTEST_TEST(UTTEST,UpdateUT)
{
    EXPECT_EQ(UpdateUT(),true);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

