###################################################### Includes ########################################################
import math
import numpy as np
from kflib.kalmanData import kalmanData
from kflib.kf import KalmanFilter as KalmanFilter
from headers.measurementPackage import sensotType as sensotType
########################################################################################################################

##################################################### Track Class ######################################################
class TrackFusion:
    def __init__(self, numState:int) -> None:
        #set kalman filter data.
        self.kd = kalmanData(numState)

        #initially set to false, set to true in first call of ProcessMeasurement.
        self._is_initialized_ = False

        #set time to be zero initially.
        self._previous_timestamp = 0

        #Process noise standard deviation longitudinal acceleration in m/s^2.
        self.noise_ax= 5

        #Process noise standard deviation longitudinal acceleration in m/s^2.
        self.noise_ay= 5

        #Laser measurement noise standard deviation position1 in m.
        self.std_laspx = 0.05

        #Laser measurement noise standard deviation position2 in m.
        self.std_laspy = 0.05

        #Radar measurement noise standard deviation radius in m.
        self.std_radr = 0.5

        #Radar measurement noise standard deviation angle in rad.
        self.std_radphi = 0.05

        #Radar measurement noise standard deviation radius change in m/s.
        self.std_radrd = 0.5
        return

    def predictionModel(self, x, dt):
        num_rows, num_cols = x.shape
        F = np.eye(num_rows)
        F[0, 2] = dt
        F[1, 3] = dt
        return F

    def prediction(self, dt):
        # *******************************************************************************
        #  1. Set the process covariance matrix Q                                       *
        # *******************************************************************************/
        dt2 = dt * dt
        dt3 = dt * dt2
        dt4 = dt * dt3
        x, y = self.noise_ax, self.noise_ay
        r11 = dt4 * x / 4
        r13 = dt3 * x / 2
        r22 = dt4 * y / 4
        r24 = dt3 * y / 2
        r31 = dt3 * x / 2
        r33 = dt2 * x
        r42 = dt3 * y / 2
        r44 = dt2 * y
        self.kd.Q = np.array([[r11, 0, r13, 0],
                              [0, r22, 0, r24],
                              [r31, 0, r33, 0],
                              [0, r42, 0, r44]])

        # *******************************************************************************
        #  2. Perform the predict step                                                  *
        # *******************************************************************************/
        self.kd.F = self.predictionModel(self.kd.x, dt)
        self.kd.x, self.kd.P = KalmanFilter.predict(self.kd.x, self.kd.P, self.kd.Q, self.kd.F)
        return

    def h_(self,x, size):
        # *******************************************************************************
        #  1. extract rho from state vector                                       *
        # *******************************************************************************/
        rho = math.sqrt(x[0]**2 + x[1]**2)

        # *******************************************************************************
        #  2. get output matrix based on sensor model                                   *
        # *******************************************************************************/
        h_x = np.zeros((size, 1))
        if(abs(rho)<0.0001):
            h_x[0] = rho
            h_x[1] = math.atan2(x[1],x[0])
            h_x[2] = 0
        else:
            h_x[0] = rho
            h_x[1] = math.atan2(x[1],x[0])
            h_x[2] = (x[0] * x[2] + x[1] * x[3]) / rho

        return h_x

    def h_prime(self,x, Meassize):
        # *******************************************************************************
        #  1. extract data from state vector                                       *
        # *******************************************************************************/
        px = x[0]
        py = x[1]
        x_dot = x[2]
        y_dot = x[3]
        x2_y2 = px**2 + py**2
        x1_y1 = math.sqrt(x2_y2)
        x3_y3 = x1_y1 * x2_y2

        # *******************************************************************************
        #  2. calculate jacobian                                   *
        # *******************************************************************************/
        numRow,numCol = x.shape
        hJ = np.zeros((Meassize, numRow))
        if (abs(x2_y2) < 0.0001):
            return hJ
        hJ[0, 0] = px / x1_y1
        hJ[0, 1] = py / x1_y1

        hJ[1, 0] = -py / x2_y2
        hJ[1, 1] = px / x2_y2

        hJ[2, 0] = py * (x_dot * py - y_dot * px) / (x3_y3)
        hJ[2, 1] = px * (y_dot * px - x_dot * py) / (x3_y3)
        hJ[2, 2] = px / x1_y1
        hJ[2, 3] = py / x1_y1

        return hJ

    def innovationHelper(self,zpred):
        angle = zpred[1]
        angle = math.fmod(angle+math.pi, 2.0*math.pi)
        if(angle<0):
            angle = angle + 2.0*math.pi
        zpred[1] = angle - math.pi
        return  zpred

    def updateLidar(self, measPack):
        # *******************************************************************************
        #  extract measurement size                                                     *
        # *******************************************************************************/
        numRow, numcol = measPack.rawMeasurement.shape
        measSize:int = numRow

        # *******************************************************************************
        #  add measurement noise covariance matrix                                      *
        # *******************************************************************************/
        H = np.eye(measSize, self.kd.dim_x)

        # *******************************************************************************
        #  add measurement noise covariance matrix                                      *
        # *******************************************************************************/
        R = np.diag([self.std_laspx ** 2, self.std_laspy ** 2])

        # *******************************************************************************
        #  Calculate Innovation                                                         *
        # *******************************************************************************/
        zpred = np.dot(H, self.kd.x)
        z_meas = measPack.rawMeasurement
        Y = z_meas - zpred
        Y = self.innovationHelper(Y)

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

        # *******************************************************************************
        #  2.set output matrix                                      *
        # *******************************************************************************/
        H = np.eye(measSize, self.kd.dim_x)
        H = self.h_prime(self.kd.x, measSize)

        # *******************************************************************************
        #  3.add measurement noise covariance matrix                                      *
        # *******************************************************************************/
        R = np.zeros((measSize, measSize))
        R = np.diag([self.std_radr ** 2, self.std_radphi ** 2, self.std_radrd ** 2])

        # *******************************************************************************
        #  Calculate Innovation                                                         *
        # *******************************************************************************/
        zpred = self.h_(self.kd.x, measSize)
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


    def processMeasurement(self, measPack):
        # *******************************************************************************
        #  Initialization                                                             *
        # *******************************************************************************/
        if not self._is_initialized_:
            self.kd.P[2,2] = 1000.0
            self.kd.P[3,3] = 1000.0

            if(measPack.sensorType == sensotType.LASER):
                px = measPack.rawMeasurement[0,0]
                py = measPack.rawMeasurement[1,0]
                self.kd.x = np.array([[px, py, 0.0, 0.0]]).T
            elif(measPack.sensorType == sensotType.RADAR):
                rho = measPack.rawMeasurement[0,0]
                phi = measPack.rawMeasurement[1,0]
                px = rho * math.cos(phi)
                py = rho * math.sin(phi)
                self.kd.x = np.array([[px, py, 0.0, 0.0]]).T
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