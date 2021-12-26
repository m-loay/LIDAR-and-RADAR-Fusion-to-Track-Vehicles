import unittest
from testScripts.testCases import *


class LinearKalmanFilter(unittest.TestCase):
    def test_linearFilter(self):
        self.assertEqual(True, linearFilter())

    def test_linearTracking(self):
        self.assertEqual(True, trackingLinearFilter())

class ExtendedKalmanFilter(unittest.TestCase):
    def test_CalculateJacobian(self):
        self.assertEqual(True, CalculateJacobian())

    def test_trackingEKF(self):
        self.assertEqual(True, trackingEKF())

class UnscentedKalmanFilter(unittest.TestCase):
    def test_CalculateSigmaPointsNoAugmentation(self):
        self.assertEqual(True, CalculateSigmaPointsNoAugmentation())

    def test_CalculateSigmaPointsWithAugmentation(self):
        self.assertEqual(True, CalculateSigmaPointsWithAugmentation())

    def test_CalculateSigmaPointsAugPred(self):
        self.assertEqual(True, CalculateSigmaPointsAugPred())

    def test_CalculateWeigts(self):
        self.assertEqual(True, CalculateWeigts())

    def test_predictMean(self):
        self.assertEqual(True, predictMean())

    def test_predictCovariance(self):
        self.assertEqual(True, predictCovariance())

    def test_predictUT(self):
        self.assertEqual(True, predictUT())

    def test_TransformPredictedSigmaToMeasurement(self):
        self.assertEqual(True, TransformPredictedSigmaToMeasurement())

    def test_CalculateMeasurementsMeanCovar(self):
        self.assertEqual(True, CalculateMeasurementsMeanCovar())

    def test_updateUT(self):
        self.assertEqual(True, updateUT())

if __name__ == '__main__':
    unittest.main()
