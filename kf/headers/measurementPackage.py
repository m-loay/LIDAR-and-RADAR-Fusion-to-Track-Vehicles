from enum import Enum
import numpy as np

class sensotType(Enum):
    LASER = 0
    RADAR = 1

class measurementPackage(object):
    def __init__(self)-> None:
        self.timeStamp:int
        self.rawMeasurement:np.zeros((1, 1))
        self.sensorType:sensotType