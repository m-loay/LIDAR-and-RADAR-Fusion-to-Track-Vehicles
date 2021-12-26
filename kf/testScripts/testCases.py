###################################################### Includes ########################################################
import math
import numpy as np
import copy
from testScripts.tools import Parser
from kflib.kf import KalmanFilter as tracker
from kflib.UT import UT as UT
from headers.measurementPackage import measurementPackage as measurementPackage
from trackApp.tracker import TrackFusion as trackFilter
from headers.measurementPackage import sensotType as sensotType
########################################################################################################################

################################################### Helper Function ####################################################

def predictionModel(x):
    num_rows, num_cols = x.shape
    F = np.ones((num_rows, num_rows))
    F[1,0] = 0
    return F

def predictionModelUT(predSigCol, *args):
    XsigPredCol = np.zeros((5, 1))
    dt= args[0]
    dt = dt[0]
    #Extract values
    p_x, p_y, v, yaw, yawd, nu_a, nu_yawdd = predSigCol[0], predSigCol[1],predSigCol[2],predSigCol[3],predSigCol[4],\
                                             predSigCol[5],predSigCol[6]

    #Predicted state values
    px_p, py_p, v_p, yaw_p, yawd_p   =0, 0, 0, 0, 0

    #model equations and aviod division by zero
    if(abs(yawd)> 0.001):
        px_p = p_x + v / yawd * (math.sin(yaw + yawd * dt) - math.sin(yaw))
        py_p = p_y + v / yawd * (-math.cos(yaw + yawd * dt) + math.cos(yaw))
    else:
        #Special case
        px_p = p_x + v * dt * math.cos(yaw)
        py_p = p_y + v * dt * math.sin(yaw)

    v_p = v;
    yaw_p = yaw + yawd * dt
    yawd_p = yawd

    #add noise
    dt2 = dt * dt;
    px_p = px_p + 0.5 * nu_a * dt2 * math.cos(yaw)
    py_p = py_p + 0.5 * nu_a * dt2 * math.sin(yaw)
    v_p = v_p + nu_a * dt
    yaw_p  = yaw_p + 0.5 * nu_yawdd * dt2
    yawd_p = yawd_p + nu_yawdd * dt

    #write predicted sigma point into right column
    XsigPredCol = np.array([[px_p, py_p, v_p, yaw_p, yawd_p]]).T
    return XsigPredCol

def predictionMeasurementModelUT(predSigCol, *args):
    ZsigPredCol = np.zeros((3, 1))

    #Extract values
    p_x, p_y, v, yaw = predSigCol[0], predSigCol[1],predSigCol[2],predSigCol[3]
    vx = v*math.cos(yaw)
    vy = v*math.sin(yaw)

    #model equations and aviod division by zero
    if abs(p_x) <= 0.0001:
        p_x = 0.0001

    if abs(p_y) <= 0.0001:
        p_y = 0.0001

    p_x2 = p_x **2
    p_y2 = p_y **2

    r = math.sqrt(p_x2 + p_y2)
    phi = math.atan2(p_y, p_x)
    r_dot = (p_x*vx + p_y*vy)/r


    #write predicted sigma point into right column
    ZsigPredCol = np.array([[r, phi, r_dot]]).T
    return ZsigPredCol

def normalizeAngle(anglein):
    angle = anglein
    if angle > np.pi:
        angle -= 2 * np.pi
    if angle < -np.pi:
        angle = 2 * np.pi
    return angle

def calc_covar(predSigCol, mean):
    numRows, numCols = mean.shape
    xdiff = predSigCol.reshape(numRows,1) - mean
    angle = xdiff[3]
    angle = angle[0]
    angle = normalizeAngle(angle)
    xdiff[3] = angle
    return  xdiff

def calc_covar_measurement(predSigCol, mean):
    numRows, numCols = mean.shape
    xVector = predSigCol[0:numRows]
    zdiff = xVector.reshape(numRows,1) - mean
    angle = zdiff[1]
    angle = angle[0]
    angle = normalizeAngle(angle)
    zdiff[1] = angle
    return  zdiff
########################################################################################################################


##################################################### Test Cases #######################################################

def linearFilter()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set state dimension
    n_x = 2

    #set measurement dimension
    n_z = 1

    #create example std::vector for predicted state mean.
    x = np.zeros((n_x, 1)) # state

    #create example matrix for predicted state covariance.
    P = np.eye(n_x)*1000

    #output matrix
    H = np.zeros((n_z, n_x))
    H[0,0] = 1

    #Sensor Noise Covariance matrix
    R = np.zeros((n_z, n_z))
    R = np.diag([[1.0]])

    #precoess noise covariance matrix Q
    Q = np.zeros((n_x, n_x))

    #*******************************************************************************
    #  Set correct Answer                                                         *
    #*******************************************************************************/
    #Correct Answer
    #x-state
    xCorr = np.zeros((n_x, 1)) # state
    xCorrList = list()

    xCorr = np.array([[0.999001,0.0]]).T
    xCorrList.append(xCorr)
    xCorr = np.array([[2.998,0.999002]]).T
    xCorrList.append(xCorr)
    xCorr = np.array([[3.99967,1.0]]).T
    xCorrList.append(xCorr)

    #P-state
    pCorr = np.zeros((n_x, n_x)) # state
    pCorrList = list()

    pCorr = np.array([[1001, 1000], [1000, 1000]])
    pCorrList.append(pCorr)
    pCorr = np.array([[4.99002, 2.99302], [2.99302, 1.99501]])
    pCorrList.append(pCorr)
    pCorr = np.array([[2.33189, 0.999168], [0.999168, 0.499501]])
    pCorrList.append(pCorr)

    #*******************************************************************************
    #  Set Measurement Input                                                      *
    #*******************************************************************************/
    #set the measurement
    measPack = measurementPackage()
    measPackList = list()

    measPack.rawMeasurement = np.array([[1.0]]).T
    measPackList.append(copy.deepcopy(measPack))
    measPack.rawMeasurement = np.array([[2.0]]).T
    measPackList.append(copy.deepcopy(measPack))
    measPack.rawMeasurement = np.array([[3.0]]).T
    measPackList.append(copy.deepcopy(measPack))

    #*******************************************************************************
    # *  Run Main Algorithm Loop                                                    *
    # *******************************************************************************/
    xResultList = list()
    pResultList = list()

    for meas in measPackList:
        #Calculate Innovation
        zpred = np.dot(H, x)
        z_meas = meas.rawMeasurement
        Y = z_meas - zpred

        #Calculate Kalman Gain
        K = tracker.calculateKalmanGain(P, H, R)

        #perform Update step
        x,P = tracker.correct(x,P,Y,H,K)

        #perform Predict step
        F = predictionModel(x)
        x,P = tracker.predict(x,P,Q,F)

        #collect result
        xResultList.append(x)
        pResultList.append((P))

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    max = len(measPackList)
    for counter in range(max):
        r = r and (np.linalg.norm(xResultList[counter] - xCorrList[counter]) <0.001)
        r = r and (np.linalg.norm(pResultList[counter] - pCorrList[counter]) < 0.001)

    return r

def trackingLinearFilter()->bool:
    #*******************************************************************************
    #  Parse input file                                                             *
    #*******************************************************************************/
    par = Parser("obj_pose-laser-radar-synthetic-input.txt", startLineNo=0,lineNo=4,laser=True,radar=False, delimiter="\t")
    measPackList = par.extractData()

    #*******************************************************************************
    #  Set correct Answer                                                         *
    #*******************************************************************************/
    #Correct Answer
    #x-state
    if (measPackList[0].sensorType == sensotType.LASER):
        px = measPackList[0].rawMeasurement[0, 0]
        py = measPackList[0].rawMeasurement[1, 0]
        xFirst = np.array([[px, py, 0.0, 0.0]]).T
    elif (measPackList[0].sensotType == sensotType.RADAR):
        rho = measPackList[0].rawMeasurement[0, 0]
        phi = measPackList[0].rawMeasurement[1, 0]
        px = rho * math.cos(phi)
        py = rho * math.sin(phi)
        xFirst = np.array([[px, py, 0.0, 0.0]]).T
    pFirst = np.eye(4)
    pFirst[2,2] = 1000.0
    pFirst[3,3] = 1000.0

    xCorr = np.zeros((4, 1)) # state
    xCorrList = list()

    xCorr = xFirst
    xCorrList.append(xCorr)
    xCorr = np.array([[0.96749,0.405862,4.58427,-1.83232]]).T
    xCorrList.append(xCorr)
    xCorr = np.array([[0.958365,0.627631,0.110368, 2.04304]]).T
    xCorrList.append(xCorr)
    xCorr = np.array([[1.34291,0.364408, 2.32002,-0.722813]]).T
    xCorrList.append(xCorr)

    #P-state
    pCorr = np.zeros((4, 4)) # state
    pCorrList = list()

    pCorr = pFirst
    pCorrList.append(pCorr)
    pCorr = np.array([[0.0224541, 0, 0.204131, 0], [0, 0.0224541, 0, 0.204131], [0.204131, 0, 92.7797, 0], [0, 0.204131, 0, 92.7797]])
    pCorrList.append(pCorr)
    pCorr = np.array([[0.0220006, 0, 0.210519, 0], [0, 0.0220006, 0, 0.210519], [0.210519, 0, 4.08801, 0], [0, 0.210519, 0, 4.08801]])
    pCorrList.append(pCorr)
    pCorr = np.array([[0.0185328, 0, 0.109639, 0], [0, 0.0185328, 0, 0.109639], [0.109639, 0, 1.10798, 0], [0, 0.109639, 0, 1.10798]])
    pCorrList.append(pCorr)

    #*******************************************************************************
    # *  Run Main Algorithm Loop                                                    *
    # *******************************************************************************/
    xResultList = list()
    pResultList = list()
    trackf = trackFilter(4)
    trackf.noise_ax = 5.0
    trackf.noise_ay = 5.0
    trackf.std_laspx = 0.15
    trackf.std_laspy = 0.15

    for meas in measPackList:
        #Call KF process measurement
        trackf.processMeasurement(meas)

        #collect result
        xResultList.append(copy.deepcopy(trackf.kd.x))
        pResultList.append(copy.deepcopy(trackf.kd.P))

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    max = len(measPackList)
    for counter in range(max):
        r = r and (np.linalg.norm(xResultList[counter] - xCorrList[counter]) <0.001)
        r = r and (np.linalg.norm(pResultList[counter] - pCorrList[counter]) < 0.001)

    return r

def CalculateJacobian()->bool:
    #*******************************************************************************
    #  Set Jacobian Inputs                                                            *
    #*******************************************************************************/
    #set state dimension
    tracEkf = trackFilter(4)
    tracEkf.kd.x = np.array([[1, 2, 0.2, 0.4]]).T

    #*******************************************************************************
    #  Calculate the Jacobian                                                      *
    #*******************************************************************************/
    Hj = tracEkf.h_prime(tracEkf.kd.x,3)

    #*******************************************************************************
    #  Set correct Answer                                                         *
    #*******************************************************************************/
    #Correct Answer
    #jacobia
    hjCorrect = np.array([[0.447214, 0.894427, 0, 0],[-0.4, 0.2, 0, 0],[0, 0, 0.447214, 0.894427]])

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r = True
    r = r and (np.linalg.norm(Hj - hjCorrect) <0.001)
    return r

def trackingEKF()->bool:
    #*******************************************************************************
    #  Parse input file                                                             *
    #*******************************************************************************/
    par = Parser("obj_pose-laser-radar-synthetic-input.txt", startLineNo=0,lineNo=4,laser=True,radar=True, delimiter="\t")
    measPackList = par.extractData()

    #*******************************************************************************
    #  Set correct Answer                                                         *
    #*******************************************************************************/
    #Correct Answer
    #x-state
    if (measPackList[0].sensorType == sensotType.LASER):
        px = measPackList[0].rawMeasurement[0, 0]
        py = measPackList[0].rawMeasurement[1, 0]
        xFirst = np.array([[px, py, 0.0, 0.0]]).T
    elif (measPackList[0].sensotType == sensotType.RADAR):
        rho = measPackList[0].rawMeasurement[0, 0]
        phi = measPackList[0].rawMeasurement[1, 0]
        px = rho * math.cos(phi)
        py = rho * math.sin(phi)
        xFirst = np.array([[px, py, 0.0, 0.0]]).T
    pFirst = np.eye(4)
    pFirst[2,2] = 1000.0
    pFirst[3,3] = 1000.0

    xCorr = np.zeros((4, 1)) # state
    xCorrList = list()

    xCorr = xFirst
    xCorrList.append(xCorr)
    xCorr = np.array([[0.722628,0.567796,3.70663,-0.56481]]).T
    xCorrList.append(xCorr)
    xCorr = np.array([[0.969665,0.413432,5.6934,-2.08598]]).T
    xCorrList.append(xCorr)
    xCorr = np.array([[0.984782,0.681457, 2.3165,0.760458]]).T
    xCorrList.append(xCorr)

    #P-state
    pCorr = np.zeros((4, 4)) # state
    pCorrList = list()

    pCorr = pFirst
    pCorrList.append(pCorr)
    pCorr = np.array([[0.0744763,   0.0957463,   0.0140901,  -0.0088403], [0.0957463,    0.127007, -0.00884025,  0.00923985], [0.0140901, -0.00884025,     180.933,    -137.793], [-0.0088403,  0.00923985,    -137.793,     105.334]])
    pCorrList.append(pCorr)
    pCorr = np.array([[0.0212348, -0.000763264,     0.275495,    -0.208923], [-0.000763264,     0.020816,    -0.208923,      0.16087], [0.275495,    -0.208923,      5.94417,      -4.3339], [-0.208923,      0.16087,      -4.3339,      3.56638]])
    pCorrList.append(pCorr)
    pCorr = np.array([[0.012367,   0.00418933,    0.0424686,   -0.0499424], [0.00418933,   0.00439293,   0.00839503, -0.000486848], [0.0424686,   0.00839503,     0.265165  ,   -0.19538], [-0.0499424, -0.000486848,     -0.19538,     0.490509]])
    pCorrList.append(pCorr)

    #*******************************************************************************
    # *  Run Main Algorithm Loop                                                    *
    # *******************************************************************************/
    xResultList = list()
    pResultList = list()
    trackf = trackFilter(4)
    trackf.kd.x = np.zeros((4, 1))
    trackf.kd.P = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1000.0, 0], [0, 0, 0, 1000.0]])
    trackf.noise_ax = 5.0
    trackf.noise_ay = 5.0
    trackf.std_laspx = 0.15
    trackf.std_laspy = 0.15
    trackf.std_radr = 0.5
    trackf.std_radphi = 0.05
    trackf.std_radrd = 0.5
    for meas in measPackList:
        #Call KF process measurement
        trackf.processMeasurement(meas)

        #collect result
        xResultList.append(copy.deepcopy(trackf.kd.x))
        pResultList.append(copy.deepcopy(trackf.kd.P))

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    max = len(measPackList)
    for counter in range(max):
        r = r and (np.linalg.norm(xResultList[counter] - xCorrList[counter]) <0.001)
        r = r and (np.linalg.norm(pResultList[counter] - pCorrList[counter]) < 0.001)

    return r

def CalculateSigmaPointsNoAugmentation()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set mean
    mean = np.array([[5.7441, 1.3800, 2.2049, 0.5015, 0.3528]]).T

    #set covariance
    covariance = np.array([[0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020],
                           [-0.0013,    0.0077,    0.0011,    0.0071,    0.0060],
                           [0.0030,    0.0011,    0.0054,    0.0007,    0.0008],
                           [-0.0022,    0.0071,    0.0007,    0.0098,    0.0100],
                           [-0.0020,    0.0060,    0.0008,    0.0100,    0.0123]])

    #*******************************************************************************
    # Calculate the sigma points                                                   *
    #*******************************************************************************/
    sigmaPoints = UT.CalculateSigmaPoints(mean, covariance)

    #*******************************************************************************
    # Set correct Answer                                                  *
    #*******************************************************************************/
    sigmaPointsCorrect = np.array([[5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441],
                                   [1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38],
                                   [2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049],
                                   [0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015],
                                   [0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879]])

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    r = r and (np.linalg.norm(sigmaPoints - sigmaPointsCorrect) < 0.001)
    return r

def CalculateSigmaPointsWithAugmentation()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set mean
    mean = np.array([[5.7441, 1.3800, 2.2049, 0.5015, 0.3528]]).T

    #set covariance
    covariance = np.array([[0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020],
                           [-0.0013,    0.0077,    0.0011,    0.0071,    0.0060],
                           [0.0030,    0.0011,    0.0054,    0.0007,    0.0008],
                           [-0.0022,    0.0071,    0.0007,    0.0098,    0.0100],
                           [-0.0020,    0.0060,    0.0008,    0.0100,    0.0123]])

    #set prcoess noise covariance
    std_a = 0.2
    std_yawwd = 0.2
    Q = np.array([[std_a**2,   0.0],
                  [0.0,   std_yawwd**2]])

    #*******************************************************************************
    # Calculate the sigma points                                                   *
    #*******************************************************************************/
    sigmaPoints = UT.CalculateSigmaPointsAug(mean, covariance, Q)

    #*******************************************************************************
    # Set correct Answer                                                  *
    #*******************************************************************************/
    sigmaPointsCorrect = np.array([[5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441 ,  5.7441 ,  5.7441,],
                                   [1.38  ,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38 ,    1.38 ,    1.38],
                                   [2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049 ,  2.2049 ,  2.2049],
                                   [0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015 ,  0.5015 ,  0.5015],
                                   [0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879 ,  0.3528 ,  0.3528],
                                   [     0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0,        0 ,-0.34641 ,       0],
                                   [     0,        0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0 ,       0 ,-0.34641]])

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    r = r and (np.linalg.norm(sigmaPoints - sigmaPointsCorrect) < 0.001)
    return r

def CalculateSigmaPointsAugPred()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set sizes
    numState:int = 5
    numAug:int = 2
    numAugState:int = numState + numAug
    numSigaPoints:int = (2*numAugState) + 1

    #set sample time
    dt:float = 0.1

    #create Sigma Points
    sigmaPoints = np.array([[5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441 ,  5.7441 ,  5.7441,],
                            [1.38  ,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38 ,    1.38 ,    1.38],
                            [2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049 ,  2.2049 ,  2.2049],
                            [0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015 ,  0.5015 ,  0.5015],
                            [0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879 ,  0.3528 ,  0.3528],
                            [     0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0,        0 ,-0.34641 ,       0],
                            [     0,        0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0 ,       0 ,-0.34641]])

    #*******************************************************************************
    # Calculate the sigma points                                                   *
    #*******************************************************************************/
    predictedSigmaPoints = UT.PredictSigmaPoints(sigmaPoints, predictionModelUT, numAug, dt)

    #*******************************************************************************
    # Set correct Answer                                                  *
    #*******************************************************************************/
    predictedSigmaPointsCorrect = np.array([[5.93553,  6.06251 , 5.92217 ,  5.9415  ,   5.92361 , 5.93516  , 5.93705 , 5.93553  , 5.80832  ,5.94481  ,5.92935  ,5.94553  ,5.93589  ,5.93401 , 5.93553],
                                            [1.48939,  1.44673 , 1.66484 ,  1.49719 ,   1.508   , 1.49001  , 1.49022 , 1.48939  , 1.5308   ,1.31287  ,1.48182  ,1.46967  ,1.48876  ,1.48855 , 1.48939],
                                            [2.2049 ,  2.28414 , 2.24557 ,  2.29582 ,   2.2049  , 2.2049   , 2.23954 , 2.2049   , 2.12566  ,2.16423  ,2.11398  ,2.2049   ,2.2049   ,2.17026 , 2.2049],
                                            [0.53678,  0.473387, 0.678098,  0.554557,   0.643644, 0.543372 , 0.53678 , 0.538512 , 0.600173 ,0.395462 ,0.519003 ,0.429916 ,0.530188 ,0.53678 , 0.535048],
                                            [0.3528 ,  0.299973, 0.462123,  0.376339,   0.48417 , 0.418721 , 0.3528  , 0.387441 , 0.405627 ,0.243477 ,0.329261 ,0.22143  ,0.286879 ,0.3528  , 0.318159]])

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    r = r and (np.linalg.norm(predictedSigmaPoints - predictedSigmaPointsCorrect) < 0.001)
    return r

def CalculateWeigts()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set sizes
    numState:int = 5
    numAug:int = 2
    numAugState:int = numState + numAug
    numSigaPoints:int = (2*numAugState) + 1

    #*******************************************************************************
    # Calculate Weigts                                                             *
    #*******************************************************************************/
    weights = UT.CalculateWeigts(numSigaPoints, numAugState)

    #*******************************************************************************
    # Set correct Answer                                                  *
    #*******************************************************************************/
    weightsCorrect = np.array([[-1.33333, 0.166667,0.166667,0.166667,0.166667,0.166667,
                                0.166667,0.166667,0.166667,0.166667,0.166667,0.166667,
                                0.166667,0.166667,0.166667]])

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    r = r and (np.linalg.norm(weights - weightsCorrect) < 0.001)
    return r

def predictMean()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set sizes
    numState:int = 5
    numAug:int = 2
    numAugState:int = numState + numAug
    numSigaPoints:int = (2*numAugState) + 1

    #set weights
    weights = np.array([[-1.33333, 0.166667,0.166667,0.166667,0.166667,0.166667,
                                0.166667,0.166667,0.166667,0.166667,0.166667,0.166667,
                                0.166667,0.166667,0.166667]])
    # set predicted Sigma Points
    predictedSigmaPoints = np.array([[5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744],
                                     [1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486],
                                     [2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049],
                                     [0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048],
                                     [0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159]])

    #*******************************************************************************
    # Calculate mean                                                *
    #*******************************************************************************/
    predictedMean = UT.PredictMean(predictedSigmaPoints, weights)

    #*******************************************************************************
    # Set correct Answer                                                  *
    #*******************************************************************************/
    predictedMeanCorrect = np.array([[ 5.93637, 1.49035, 2.20528, 0.536853,0.353577]]).T

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    delta = predictedMean - predictedMeanCorrect
    r = r and (np.linalg.norm(predictedMean - predictedMeanCorrect) < 0.001)
    return r

def predictCovariance()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set sizes
    numState:int = 5
    numAug:int = 2
    numAugState:int = numState + numAug
    numSigaPoints:int = (2*numAugState) + 1

    #set weights
    weights = np.array([[-1.33333, 0.166667,0.166667,0.166667,0.166667,0.166667,
                                0.166667,0.166667,0.166667,0.166667,0.166667,0.166667,
                                0.166667,0.166667,0.166667]])
    # set predicted Sigma Points
    predictedSigmaPoints = np.array([[5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744],
                                     [1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486],
                                     [2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049],
                                     [0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048],
                                     [0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159]])

    #set mean
    mean = np.array([[5.93637, 1.49035, 2.20528, 0.536853, 0.353577]]).T

    #*******************************************************************************
    # Calculate mean                                                *
    #*******************************************************************************/
    predictedCovar = UT.PredictCovar(mean, predictedSigmaPoints, weights, calc_covar)

    #*******************************************************************************
    # Set correct Answer                                                  *
    #*******************************************************************************/
    predictedCovarCorrect = np.array([[0.00543425, -0.0024053  ,0.00341576 , -0.00348196 ,-0.00299378],
                                     [-0.0024053 ,  0.010845   ,0.0014923  , 0.00980182  , 0.00791091],
                                     [0.00341576,  0.0014923  ,0.00580129 , 0.000778632 , 0.000792973],
                                     [-0.00348196,  0.00980182 ,0.000778632, 0.0119238   , 0.0112491],
                                     [-0.00299378,  0.00791091 ,0.000792973, 0.0112491   , 0.0126972]])

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    r = r and (np.linalg.norm(predictedCovar - predictedCovarCorrect) < 0.001)
    return r

def predictUT()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set sizes
    numState:int = 5
    numAug:int = 2
    numAugState:int = numState + numAug
    numSigaPoints:int = (2*numAugState) + 1
    dt = 0.1
    #set mean
    mean = np.array([[5.7441, 1.3800, 2.2049, 0.5015, 0.3528]]).T

    #set covariance
    covariance = np.array([[0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020],
                           [-0.0013,    0.0077,    0.0011,    0.0071,    0.0060],
                           [0.0030,    0.0011,    0.0054,    0.0007,    0.0008],
                           [-0.0022,    0.0071,    0.0007,    0.0098,    0.0100],
                           [-0.0020,    0.0060,    0.0008,    0.0100,    0.0123]])

    #set process noise covariance (Q)
    std_a = 0.2
    std_yawwd = 0.2
    Q = np.array([[std_a ** 2, 0.0],
                  [0.0, std_yawwd ** 2]])

    #*******************************************************************************
    # Calculate Mean & covar of UT                                                  *
    #*******************************************************************************/
    # Calculate sigma points
    sigmaPoints = UT.CalculateSigmaPointsAug(mean, covariance, Q)

    # Predict Sigma Points
    predSig = UT.PredictSigmaPoints(sigmaPoints, predictionModelUT, numAug, dt)

    #Calculate Weights
    weights = UT.CalculateWeigts(numSigaPoints, numAugState)

    #Calculate mean
    mean = UT.PredictMean(predSig, weights)

    #Calculate Covariance
    covar = UT.PredictCovar(mean, predSig, weights, calc_covar)

    #*******************************************************************************
    # Set correct Answer                                                  *
    #*******************************************************************************/
    #correct mean state
    meanCorrect = np.array([[5.93445, 1.48885, 2.2049, 0.53678, 0.3528]]).T

    # correct covar state
    covarCorrect = np.array([[0.00543425, -0.0024053  ,0.00341576 , -0.00348196 ,-0.00299378],
                      [-0.0024053 ,  0.010845   ,0.0014923  , 0.00980182  , 0.00791091],
                      [0.00341576,  0.0014923  ,0.00580129 , 0.000778632 , 0.000792973],
                      [-0.00348196,  0.00980182 ,0.000778632, 0.0119238   , 0.0112491],
                      [-0.00299378,  0.00791091 ,0.000792973, 0.0112491   , 0.0126972]])
    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    r = r and (np.linalg.norm(mean - meanCorrect) < 0.001)
    r = r and (np.linalg.norm(covar - covarCorrect) < 0.001)
    return r

def TransformPredictedSigmaToMeasurement()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set sizes
    numState:int = 5
    numAug:int = 2
    numAugState:int = numState + numAug
    numSigaPoints:int = (2*numAugState) + 1
    dt = 0.1

    #set Predicted Sigma points
    predictedSigmaPoints = np.array([[5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744],
                                     [1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486],
                                     [2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049],
                                     [0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048],
                                     [0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159]])

    #*******************************************************************************
    # TransformPredictedSigmaToMeasurement                                               *
    #*******************************************************************************/
    # Calculate sigma points
    tSig_pred = UT.TransformPredictedSigmaToMeasurement(predictedSigmaPoints, 3,predictionMeasurementModelUT)

    #*******************************************************************************
    # Set correct Answer                                                  *
    #*******************************************************************************/
    # correct covar state
    tSig_predCorrect = np.array([[6.11908,  6.23346,  6.15315,  6.12835,  6.11436,  6.11908,  6.12218,  6.11908,  6.00792,  6.08839,  6.11255,  6.12488,  6.11908,  6.11886,  6.12057],
                                 [0.244289,  0.23371, 0.273165, 0.246166, 0.248461, 0.244289, 0.245307, 0.244289, 0.257001, 0.216927, 0.244336, 0.241934, 0.244289, 0.245157, 0.245239],
                                 [2.11044,  2.21881,  2.06391,   2.1875,  2.03413,  2.10616,  2.14509,  2.10929,  2.00166,   2.1298,  2.03466,  2.16518,  2.11454,  2.07862,  2.11295]])

    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    r = r and (np.linalg.norm(tSig_pred - tSig_predCorrect) < 0.001)
    return r

def CalculateMeasurementsMeanCovar()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set sizes
    numState:int = 5
    numAug:int = 2
    nz:int = 3
    numAugState:int = numState + numAug
    numSigaPoints:int = (2*numAugState) + 1
    dt = 0.1

    #radar measurement noise standard deviation radius in m
    std_radr = 0.3

    #radar measurement noise standard deviation angle in rad
    std_radphi = 0.0175

    #radar measurement noise standard deviation radius change in m/s
    std_radrd = 0.1

    #Set sensor noise
    R = np.array([[std_radr**2, 0, 0],
                  [0, std_radphi**2, 0],
                  [0,0,std_radrd**2]])

    #set Predicted Sigma points
    predictedSigmaPoints = np.array([[5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744],
                                     [1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486],
                                     [2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049],
                                     [0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048],
                                     [0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159]])

    #*******************************************************************************
    # Calculate Measurement mean and covraiance                                              *
    #*******************************************************************************/
    # Calculate sigma points(from Prediction Sigma Points)
    tSig_pred = UT.TransformPredictedSigmaToMeasurement(predictedSigmaPoints, 3,predictionMeasurementModelUT)

    # Calculate the weights
    weights = UT.CalculateWeigts(numSigaPoints, numAugState)

    # Calculate Mean
    zpred = UT.PredictMean(tSig_pred, weights)

    #Calculate covraiance
    S = UT.PredictCovar(zpred, tSig_pred, weights, calc_covar_measurement)
    S = S + R

    #*******************************************************************************
    # Set correct Answer                                                  *
    #*******************************************************************************/
    # correct mean state
    zpredCorrect = np.array([[6.12155, 0.245993, 2.10313]]).T

    # correct covar state
    sCorrect = np.array([[0.0946171, -0.000139448, 0.00407016],
                             [-0.000139448, 0.000617548, -0.000770652],
                             [0.00407016, -0.000770652, 0.0180917]])
    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    r = r and (np.linalg.norm(zpred - zpredCorrect) < 0.001)
    r = r and (np.linalg.norm(S - sCorrect) < 0.001)
    return r

def updateUT()->bool:
    #*******************************************************************************
    #  Initialization                                                             *
    #*******************************************************************************/
    #set sizes
    numState:int = 5
    numAug:int = 2
    nz:int = 3
    numAugState:int = numState + numAug
    numSigaPoints:int = (2*numAugState) + 1

    #set mean
    predictedMean = np.array([[5.93637,1.49035,2.20528,0.536853,0.353577]]).T

    #set covariance
    predictedCovariance = np.array([[0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378],
                                    [-0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091],
                                    [0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973],
                                    [-0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491],
                                    [-0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972]])

    #set Predicted Sigma points
    xSigPred = np.array([[5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744],
                         [1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486],
                         [2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049],
                         [0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048],
                         [0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159]])

    #create measurement Vector
    z = np.array([[5.9214, 0.2187, 2.0062]]).T

    #create measurement Vector
    z_pred = np.array([[6.12155,0.245993,2.10313]]).T

    # create measurement covariance Matrix
    S = np.array([[0.0946171, -0.000139448,   0.00407016],
                  [-0.000139448,  0.000617548, -0.000770652],
                  [0.00407016, -0.000770652,    0.0180917]])

    # create example matrix with sigma points in measurement space
    zSigPred = np.array([[6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057],
                     [0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239],
                     [2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295]])

    #*******************************************************************************
    # update cycle for measurement                                                    *
    #*******************************************************************************/
    # Calculate the weights
    weights = UT.CalculateWeigts(numSigaPoints, numAugState)

    #calculate Kalman gain
    K = UT.CalculateKalmanGainUT(predictedMean, z_pred, weights, xSigPred, zSigPred, S, calc_covar, calc_covar_measurement)

    #update
    Y = z - z_pred
    mean, covar = UT.updateUT(predictedMean, predictedCovariance, Y, S, K)

    #*******************************************************************************
    # Set correct Answer                                                  *
    #*******************************************************************************/
    # correct mean state
    meanCorrect = np.array([[5.92276, 1.41823, 2.15593, 0.489274,  0.321338]]).T

    # correct covar state
    covarCorrect = np.array([[0.00361579 ,-0.000357881 ,  0.00208316 ,-0.000937196 , -0.00071727],
                             [-0.000357881,   0.00539867,   0.00156846,   0.00455342,   0.00358885],
                             [0.00208316,   0.00156846,   0.00410651,   0.00160333,   0.00171811],
                             [-0.000937196,   0.00455342,   0.00160333,   0.00652634,   0.00669436],
                             [-0.00071719,   0.00358884,   0.00171811,   0.00669426,   0.00881797]])
    #*******************************************************************************
    #  Evaluation                                                    *
    # *******************************************************************************/
    r= True
    r = r and (np.linalg.norm(mean - meanCorrect) < 0.001)
    r = r and (np.linalg.norm(covar - covarCorrect) < 0.001)
    return r


