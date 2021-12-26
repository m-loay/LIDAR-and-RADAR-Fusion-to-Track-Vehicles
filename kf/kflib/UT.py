# imports
import math
import numpy as np
class UT():

    """ UT.


    """
    @staticmethod
    def CalculateSigmaPoints(mean, covariance):
        """ 
        CalculateSigmaPoints
        """
        # Extract sizes
        meanRows,meanCols= mean.shape
        numSigmaPoints:int = (2*meanRows) + 1
        lambdaVal:float = float(3 - meanRows)

        # create sigma points matrix
        sigmaPoints = np.zeros((meanRows, numSigmaPoints))

        #create square root matrix
        L = np.linalg.cholesky(covariance)

        # design coeff
        coeff:float = float(math.sqrt(lambdaVal+ meanRows))

        #create Augmented Matrix
        sigmaPoints[:,0] = mean.T
        for i in range(meanRows):
            sigmaPoints[:,i+1] = mean.T + (coeff * L[:,i])
            sigmaPoints[:, i + 1 + meanRows] = mean.T - (coeff * L[:, i])
        return sigmaPoints

    @staticmethod
    def CalculateSigmaPointsAug(mean, covariance, Q):
        """
        CalculateSigmaPoints
        """
        # Extract sizes
        meanRows,meanCols= mean.shape
        qRows, qcols = Q.shape
        numAugStates = meanRows + qRows
        numSigmaPoints:int = (2*numAugStates) + 1
        lambdaVal:float = float(3 - numAugStates)
        diff:int = numAugStates - meanRows

        #create Augmented mean vector
        meanAug = np.zeros((numAugStates, 1))
        meanAug[0:meanRows, :] = mean

        #create Augmented mean matrix
        covAug = np.zeros((numAugStates, numAugStates))
        covAug[0:meanRows , 0:meanRows] = covariance
        covAug[meanRows:, meanRows:] = Q

        # create sigma points matrix
        sigmaPoints = np.zeros((numAugStates, numSigmaPoints))

        #create square root matrix
        L = np.linalg.cholesky(covAug)

        # design coeff
        coeff:float = float(math.sqrt(lambdaVal+ numAugStates))

        #create Augmented Matrix
        sigmaPoints[:,0] = meanAug.T
        for i in range(numAugStates):
            colSig = (coeff * L[:,i])
            sigmaPoints[:,i+1] = meanAug.T + colSig
            sigmaPoints[:, i + 1 + numAugStates] = meanAug.T - colSig
        return sigmaPoints

    @staticmethod
    def PredictSigmaPoints(sigmaPoints, modelFn, numAug =0, *args):
        # Extract sizes
        numRow, numCol = sigmaPoints.shape
        numSigmaPoints = numCol
        numState = numRow - numAug
        colSize = numRow

        #create predicted sigma points
        predictedSigmaPoints = np.zeros((numState, numSigmaPoints))
        xx= predictedSigmaPoints[:,0]

        #Propagate sigma points in dynamic model
        for i in range(numSigmaPoints):
            modelCol  = modelFn(sigmaPoints[:,i], args)
            predictedSigmaPoints[:,i] = modelCol.reshape(numState,)

        return  predictedSigmaPoints

    @staticmethod
    def CalculateWeigts(numSigmaPoint, numAugStates):
        lambdaVal: float = float(3 - numAugStates)
        coeff_0 = lambdaVal/(lambdaVal + numAugStates)
        coeff = 1.0/(2.0 *(lambdaVal + numAugStates))
        weights = np.full((numSigmaPoint, 1), coeff).T
        weights[0,0] = coeff_0
        return weights

    @staticmethod
    def PredictMean(predictedSigmaPoints, weights):
        # extract sizes
        numState, numSigmaPoints = predictedSigmaPoints.shape

        #Initialize mean matrix
        mean = np.zeros((numState, 1))

        #calculate the mean
        for i in range(numSigmaPoints):
            wp = weights[0,i] * predictedSigmaPoints[:,i]
            mean = (mean + wp.reshape(numState, 1))

        return mean

    @staticmethod
    def PredictCovar(mean, predictedSigmaPoints, weights, helpFn=None):
        # extract sizes
        numRows, numSigmaPoints = predictedSigmaPoints.shape
        numState, numCols = mean.shape

        #Initialize covariance matrix
        covar = np.zeros((numState, numState))

        #select subtract method
        if helpFn is None:
            sub = np.subtract
        else:
            sub = helpFn

        #calculate the covariance
        for i in range(numSigmaPoints):
            predSigCol = predictedSigmaPoints[:,i]
            xdiff = sub(predSigCol, mean)
            covar = covar + weights[0,i] * xdiff * xdiff.T

        return covar

    @staticmethod
    def TransformPredictedSigmaToMeasurement(predictedSigmaPoints, measSize, modelFn, *args):
        # extract sizes
        numState, numSigmaPoints = predictedSigmaPoints.shape

        # create measurement predicted sigma points
        mPredictedSigmaPoints = np.zeros((measSize, numSigmaPoints))

        # Propagate sigma points in dynamic model
        for i in range(numSigmaPoints):
            modelCol = modelFn(predictedSigmaPoints[:, i], args)
            mPredictedSigmaPoints[:, i] = modelCol.reshape(measSize, )
        return  mPredictedSigmaPoints

    @staticmethod
    def CalculateKalmanGainUT(mean, zpred, weights, xPredSigPoints, zPredSigPoints, S, xFun, zFun):
        # extract sizes
        numState, numSigmaPoints = xPredSigPoints.shape
        numMeas, numCols = zpred.shape

        # create matrix for cross correlation Tc
        tC = np.zeros((numState, numMeas))

        #select subtract method
        if xFun is None:
            subX = np.subtract
        else:
            subX = xFun

        if zFun is None:
            subZ = np.subtract
        else:
            subZ = zFun

        # Propagate sigma points in dynamic model
        for i in range(numSigmaPoints):
            xDiff = subX(xPredSigPoints[:,i], mean)
            zDiff = subZ(zPredSigPoints[:, i], zpred)
            tC = tC + weights[0, i] * xDiff * zDiff.T

        # Calculate Kalman Gain
        SI  = np.linalg.inv(S)
        K = np.dot(tC, np.linalg.inv(S))
        return  K

    @staticmethod
    def updateUT(mean, covariance, innovation, S, K):
        # x = x + KY
        # predict new x with residual scaled by the kalman gain
        mean = mean + np.dot(K, innovation)

        # P = P - KSK'
        covariance = covariance - np.dot(K, np.dot(S,K.T))

        return mean, covariance