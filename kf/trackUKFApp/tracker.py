###################################################### Includes ########################################################
import math
import numpy as np
from kflib.kalmanData import kalmanDataUT
from kflib.kf import KalmanFilter as KalmanFilter
from kflib.UT import UT as UT
from headers.measurementPackage import sensotType as sensotType
########################################################################################################################

##################################################### Track Class ######################################################
class TrackUKF:
    def __init__(self, numState:int, numAug:int) -> None:
        #set kalman filter data.
        self.kd = kalmanDataUT(numState, numAug)

        #initially set to false, set to true in first call of ProcessMeasurement.
        self._is_initialized_ = False

        #set time to be zero initially.
        self._previous_timestamp = 0

        #Process noise standard deviation longitudinal acceleration in m/s^2.
        self.std_a = 1.0

        #Process noise standard deviation longitudinal acceleration in m/s^2.
        self.std_yawdd = 1.0

        #Laser measurement noise standard deviation position1 in m.
        self.std_laspx = 0.05

        #Laser measurement noise standard deviation position2 in m.
        self.std_laspy_ = 0.05

        #Radar measurement noise standard deviation radius in m.
        self.std_radr = 0.4

        #Radar measurement noise standard deviation angle in rad.
        self.std_radphi = 0.04

        #Radar measurement noise standard deviation radius change in m/s.
        self.std_radrd = 0.4
        return

    def predictionModelUT(self, predSigCol, *args):
        XsigPredCol = np.zeros((5, 1))
        dt = args[0]
        dt = dt[0]
        # Extract values
        p_x, p_y, v, yaw, yawd, nu_a, nu_yawdd = predSigCol[0], predSigCol[1], predSigCol[2], predSigCol[3],\
                                                 predSigCol[4],predSigCol[5], predSigCol[6]

        # Predicted state values
        px_p, py_p, v_p, yaw_p, yawd_p = 0, 0, 0, 0, 0

        # model equations and aviod division by zero
        if (abs(yawd) > 0.001):
            px_p = p_x + v / yawd * (math.sin(yaw + yawd * dt) - math.sin(yaw))
            py_p = p_y + v / yawd * (-math.cos(yaw + yawd * dt) + math.cos(yaw))
        else:
            # Special case
            px_p = p_x + v * dt * math.cos(yaw)
            py_p = p_y + v * dt * math.sin(yaw)

        v_p = v;
        yaw_p = yaw + yawd * dt
        yawd_p = yawd

        # add noise
        dt2 = dt * dt;
        px_p = px_p + 0.5 * nu_a * dt2 * math.cos(yaw)
        py_p = py_p + 0.5 * nu_a * dt2 * math.sin(yaw)
        v_p = v_p + nu_a * dt
        yaw_p = yaw_p + 0.5 * nu_yawdd * dt2
        yawd_p = yawd_p + nu_yawdd * dt

        # write predicted sigma point into right column
        XsigPredCol = np.array([[px_p, py_p, v_p, yaw_p, yawd_p]]).T
        return XsigPredCol

    def predictionMeasurementModelUT(self, predSigCol, *args):
        ZsigPredCol = np.zeros((3, 1))

        # Extract values
        p_x, p_y, v, yaw = predSigCol[0], predSigCol[1], predSigCol[2], predSigCol[3]
        vx = v * math.cos(yaw)
        vy = v * math.sin(yaw)

        # model equations and aviod division by zero
        if abs(p_x) <= 0.0001:
            p_x = 0.0001

        if abs(p_y) <= 0.0001:
            p_y = 0.0001

        p_x2 = p_x ** 2
        p_y2 = p_y ** 2

        r = math.sqrt(p_x2 + p_y2)
        phi = math.atan2(p_y, p_x)
        r_dot = (p_x * vx + p_y * vy) / r

        # write predicted sigma point into right column
        ZsigPredCol = np.array([[r, phi, r_dot]]).T
        return ZsigPredCol

    def normalizeAngle(self, anglein):
        angle = anglein
        if angle > np.pi:
            angle -= 2 * np.pi
        if angle < -np.pi:
            angle = 2 * np.pi
        return angle

    def calc_covar(self, predSigCol, mean):
        numRows, numCols = mean.shape
        xdiff = predSigCol.reshape(numRows, 1) - mean
        angle = xdiff[3]
        angle = angle[0]
        angle = self.normalizeAngle(angle)
        xdiff[3] = angle
        return xdiff

    def calc_covar_measurement(self, predSigCol, mean):
        numRows, numCols = mean.shape
        xVector = predSigCol[0:numRows]
        zdiff = xVector.reshape(numRows, 1) - mean
        angle = zdiff[1]
        angle = angle[0]
        angle = self.normalizeAngle(angle)
        zdiff[1] = angle
        return zdiff

    def prediction(self, dt):
        # *******************************************************************************
        #  1. Set the process covariance matrix Q                                       *
        # *******************************************************************************/
        Q = np.array([[self.std_a ** 2, 0.0],
                      [0.0, self.std_yawdd ** 2]])

        # *******************************************************************************
        #  2. Calculate & Predict Sigma Points                                           *
        # *******************************************************************************/
        # Calculate sigma points
        sigmaPoints = UT.CalculateSigmaPointsAug(self.kd.x, self.kd.P, Q)

        # Predict Sigma Points
        self.kd.sigmaPoints = UT.PredictSigmaPoints(sigmaPoints, self.predictionModelUT, self.kd.NumAug, dt)

        # *******************************************************************************
        #  3. Calculate Predicted Mean & covariance                                           *
        # *******************************************************************************/
        # Calculate Weights
        weights = UT.CalculateWeigts(self.kd.numSigmaPoints, self.kd.numStateAug)

        # Calculate mean
        self.kd.x = UT.PredictMean(self.kd.sigmaPoints, weights)

        # Calculate Covariance
        self.kd.P = UT.PredictCovar(self.kd.x, self.kd.sigmaPoints, weights, self.calc_covar)
        return

    def updateLidar(self, measPack):
        # *******************************************************************************
        #  extract measurement size                                                     *
        # *******************************************************************************/
        numRow, numcol = measPack.rawMeasurement.shape
        measSize:int = numRow

        # *******************************************************************************
        #  add measurement noise covariance matrix                                      *
        # *******************************************************************************/
        H = np.eye(measSize, self.kd.numState)

        # *******************************************************************************
        #  add measurement noise covariance matrix                                      *
        # *******************************************************************************/
        R = np.diag([self.std_laspx ** 2, self.std_laspy_** 2])

        # *******************************************************************************
        #  Calculate Innovation                                                         *
        # *******************************************************************************/
        zpred = np.dot(H, self.kd.x)
        z_meas = measPack.rawMeasurement
        Y = z_meas - zpred

        # *******************************************************************************
        # Calculate Kalman Gain                                                         *                                                         *
        # *******************************************************************************/
        K = KalmanFilter.calculateKalmanGain(self.kd.P, H, R)

        # *******************************************************************************
        # Update Linear                                                         *                                                         *
        # *******************************************************************************/
        self.kd.x, self.kd.P = KalmanFilter.correct(self.kd.x, self.kd.P,Y,H,K)
        return

    def updateRadar(self, measPack):
        # *******************************************************************************
        #  1.extract measurement size                                                     *
        # *******************************************************************************/
        numRow, numcol = measPack.rawMeasurement.shape
        measSize:int = numRow
        z = measPack.rawMeasurement.reshape(measSize, 1)

        # *******************************************************************************
        #  2. add measurement noise covariance matrix                                      *
        # *******************************************************************************/
        R = np.zeros((measSize, measSize))
        R = np.diag([self.std_radr ** 2, self.std_radphi ** 2, self.std_radrd ** 2])

        # *******************************************************************************
        #  3. Calculate Sigma Points                                                    *
        # *******************************************************************************/
        tSig_pred = UT.TransformPredictedSigmaToMeasurement(self.kd.sigmaPoints, measSize, self.predictionMeasurementModelUT)

        # *******************************************************************************
        # Calculate Measurement mean and covraiance                                      *
        # *******************************************************************************/
        # Calculate the weights
        weights = UT.CalculateWeigts(self.kd.numSigmaPoints, self.kd.numStateAug)

        # Calculate Mean
        zpred = UT.PredictMean(tSig_pred, weights)

        # Calculate covraiance
        S = UT.PredictCovar(zpred, tSig_pred, weights, self.calc_covar_measurement)
        S = S + R

        # *******************************************************************************
        # Update                                                                       *                                                         *
        # *******************************************************************************/
        # calculate Kalman gain
        K = UT.CalculateKalmanGainUT(self.kd.x, zpred, weights, self.kd.sigmaPoints, tSig_pred, S, self.calc_covar,self.calc_covar_measurement)
        Y = z - zpred
        self.kd.x, self.kd.P = UT.updateUT(self.kd.x, self.kd.P, Y, S, K)
        return


    def processMeasurement(self, measPack):
        # *******************************************************************************
        #  Initialization                                                             *
        # *******************************************************************************/
        if not self._is_initialized_:
            if(measPack.sensorType == sensotType.LASER):
                px = measPack.rawMeasurement[0,0]
                py = measPack.rawMeasurement[1,0]
                self.kd.x = np.array([[px, py, 0.0, 0.0, 0.0]]).T
            elif(measPack.sensorType == sensotType.RADAR):
                rho = measPack.rawMeasurement[0,0]
                phi = measPack.rawMeasurement[1,0]
                px = rho * math.cos(phi)
                py = rho * math.sin(phi)
                self.kd.x = np.array([[px, py, 0.0, 0.0, 0.0]]).T
            self._previous_timestamp = measPack.timeStamp
            self._is_initialized_ = True
            return
        # *******************************************************************************
        #  sample time calculations                                                     *
        # *******************************************************************************/
        #compute the time elapsed between the current and previous measurements
        #dt - expressed in seconds
        dt = (measPack.timeStamp - self._previous_timestamp) / 1000000.0
        self._previous_timestamp = measPack.timeStamp

        # *******************************************************************************
        #  Prediction                                                                   *
        # *******************************************************************************/
        #perform Prediction step.
        self.prediction(dt)

        # *******************************************************************************
        #  Update                                                                       *
        # *******************************************************************************/
        #perform Prediction step.
        if (measPack.sensorType == sensotType.LASER):
            self.updateLidar(measPack)
        elif (measPack.sensorType == sensotType.RADAR):
            self.updateRadar(measPack)



########################################################################################################################