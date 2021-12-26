# imports
import numpy as np

class KalmanFilter():

    """ Kalman filter. 
    Parameters
    ----------
    x : numpy.array(numState, 1)
        State estimate vector

    P : numpy.array(numState, numState)
        Covariance matrix

    Q : numpy.array(numState, numState)
        Process noise matrix

    F : numpy.array()
        State Transition matrix

    zmeas : (dim_z, 1): array_like
            measurement for this update. z can be a scalar if dim_z is 1,
            otherwise it must be convertible to a column vector.

    zpred : (dim_z, 1): array_like
            its the propagation of state space into measurement space.

    R : np.array, scalar, or None
        Optionally provide R to override the measurement noise for this
        one call, otherwise  self.R will be used.

    H : np.array, or None
        Optionally provide H to override the measurement function for this
        one call, otherwise self.H will be used.

    """
    @staticmethod
    def predict(x,P,Q,F):
        """ 
        Predict next state using the motion model
        """
        x = np.dot(F, x)
        P = np.dot(F, P).dot(F.T) + Q
        return x, P

    @staticmethod
    def calculateKalmanGain(P, H, R):
        """
        Calculates Kalman Gain
        """
        # common subexpression for speed
        PHT = np.dot(P, H.T)

        # S = HPH' + R
        # project system uncertainty into measurement space
        S = np.dot(H, PHT) + R
        SI = np.linalg.inv(S)

        # K = PH'inv(S)
        # map system uncertainty into kalman gain
        K = np.dot(PHT, SI)
        return K

    @staticmethod
    def correct(x, P, Y, H, K):

        """
        Add a new measurement (z) to the Kalman filter.

        If z is None, nothing is computed. However, x_post and P_post are
        updated with the prior (x_prior, P_prior), and self.z is set to None.
        """

        # x = x + KY
        # predict new x with residual scaled by the kalman gain
        x = x + np.dot(K, Y)

        # P = (I-KH)P(I-KH)' + KRK'
        num_rows, num_cols = x.shape
        I =  np.eye(num_rows)
        I_KH = I - np.dot(K, H)
        P = np.dot(I_KH, P)
        return x, P