import numpy as np

class groundTruth(object):
    def __init__(self)-> None:
        self.timeStamp:int
        self.rawMeasurement:np.zeros((1, 1))