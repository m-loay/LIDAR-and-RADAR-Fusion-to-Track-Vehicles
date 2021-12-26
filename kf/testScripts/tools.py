###################################################### Includes ########################################################
import numpy as np
import copy
import math
from headers.measurementPackage import measurementPackage as measurementPackage
from headers.groundTruth import groundTruth as groundTruth
from headers.measurementPackage import sensotType as sensotType
########################################################################################################################


#################################################### Parser Class#######################################################

class Parser():
    def __init__(self,fileName:str,startLineNo:int, laser:bool, radar:bool, lineNo:int=0, allLines:bool=False ,delimiter=" ") -> None:
        self.fileName = fileName
        self.startLineNo = startLineNo
        self.lineNo = lineNo
        self.laser = laser
        self.radar = radar
        self.delimiter = delimiter
        self.allLines = allLines
        file = open(self.fileName, "r")
        self.lines = file.readlines()[self.startLineNo:]
        file.close()

    def extractData(self):
        measurePackList = list()
        counter:int = 0
        for line in self.lines:
            fields = line.split(self.delimiter)
            senType:str = fields[0]
            if(senType == "L" and self.laser == True):
                measPack = measurementPackage()
                measPack.sensorType = sensotType.LASER
                px = float(fields[1])
                py = float(fields[2])
                measPack.rawMeasurement =  np.array([[px,py]]).T
                measPack.timeStamp = int(fields[3])
                measurePackList.append(copy.deepcopy(measPack))
                counter = counter + 1
            elif(senType == "R" and self.radar == True):
                measPack = measurementPackage()
                measPack.sensorType = sensotType.RADAR
                rho = float(fields[1])
                phi = float(fields[2])
                rhoDot = float(fields[3])
                measPack.rawMeasurement =  np.array([[rho, phi, rhoDot]]).T
                measPack.timeStamp = int(fields[4])
                measurePackList.append(copy.deepcopy(measPack))
                counter = counter + 1
            else:
                continue

            if counter == self.lineNo and self.allLines is False:
                break
        return measurePackList

    def extractDataGt(self):
        measurePackList = list()
        gtPackList = list()
        counter:int = 0
        for line in self.lines:
            fields = line.split(self.delimiter)
            senType:str = fields[0]
            if(senType == "L" and self.laser == True):
                measPack = measurementPackage()
                measPack.sensorType = sensotType.LASER
                px = float(fields[1])
                py = float(fields[2])
                measPack.rawMeasurement =  np.array([[px,py]]).T
                measPack.timeStamp = int(fields[3])
                measurePackList.append(copy.deepcopy(measPack))
                gx = float(fields[4])
                gy = float(fields[5])
                gvx = float(fields[6])
                gvy = float(fields[7])
                gtPack = groundTruth()
                gtPack.timeStamp = int(fields[3])
                gtPack.rawMeasurement = np.array([[gx,gy, gvx, gvy]]).T
                gtPackList.append(copy.deepcopy(gtPack))
                counter = counter + 1
            elif(senType == "R" and self.radar == True):
                measPack = measurementPackage()
                measPack.sensorType = sensotType.RADAR
                rho = float(fields[1])
                phi = float(fields[2])
                rhoDot = float(fields[3])
                measPack.rawMeasurement =  np.array([[rho, phi, rhoDot]]).T
                measPack.timeStamp = int(fields[4])
                measurePackList.append(copy.deepcopy(measPack))
                gx = float(fields[5])
                gy = float(fields[6])
                gvx = float(fields[7])
                gvy = float(fields[8])
                gtPack = groundTruth()
                gtPack.timeStamp = int(fields[4])
                gtPack.rawMeasurement = np.array([[gx,gy, gvx, gvy]]).T
                gtPackList.append(copy.deepcopy(gtPack))
                counter = counter + 1
            else:
                continue

            if counter == self.lineNo and self.allLines is False:
                break
        return measurePackList, gtPackList


def get_RMSE(px, py, vx, vy, tpx, tpy, tvx, tvy):
    """
    Computes the root mean square errors (RMSE) of the attributes of two lists of DataPoint() instances

    Args:
      predictions - a list of DataPoint() instances
      truths - a list of DataPoint() instances

    Returns:
      px, py, vx, vy - The RMSE of each respective DataPoint() attributes (type: float)
    """
    rmsePx = math.sqrt(np.square(np.subtract(px, tpx)).mean())
    rmsePy = math.sqrt(np.square(np.subtract(py, tpy)).mean())
    rmseVx = math.sqrt(np.square(np.subtract(vx, tvx)).mean())
    rmseVy = math.sqrt(np.square(np.subtract(vy, tvy)).mean())

    return rmsePx, rmsePy, rmseVx, rmseVy

if __name__ == '__main__':
    par = Parser("obj_pose-laser-radar-synthetic-input.txt", 1, 3, True, False,"\t")
    measurePackList = par.extractData()