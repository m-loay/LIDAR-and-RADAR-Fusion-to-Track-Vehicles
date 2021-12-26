import numpy as np
class kalmanData:
    def __init__(self, dim_x):
        self.dim_x:int = dim_x
        self.x = np.zeros((dim_x, 1))   # state
        self.P = np.eye(dim_x)          # uncertainty covariance
        self.F = np.zeros(dim_x)        # state transition matrix
        self.Q = np.zeros(dim_x)        # process uncertainty

class kalmanDataUT:
    def __init__(self, dim_x,NumAug=0):
        self.numState:int = dim_x
        self.NumAug: int = NumAug
        self.numSigmaPoints:int = 2 * (self.numState + self.NumAug) + 1
        self.numStateAug = self.numState + self.NumAug
        self.x = np.zeros((dim_x, 1))   # state
        self.weights = np.zeros((dim_x, 1))  # weights
        self.P = np.eye(dim_x)          # uncertainty covariance
        self.F = np.zeros(dim_x)        # state transition matrix
        self.Q = np.zeros(dim_x)        # process uncertainty
        self.sigmaPoints = np.zeros(dim_x)  # process uncertainty