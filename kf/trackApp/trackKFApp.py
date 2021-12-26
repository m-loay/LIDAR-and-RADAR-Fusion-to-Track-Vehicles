###################################################### Includes ########################################################
import copy

import numpy as np
import math
from testScripts.tools import Parser
from testScripts.tools import get_RMSE
from matplotlib import pyplot as plt
from trackApp.tracker import TrackFusion
from headers.measurementPackage import sensotType as sensotType
########################################################################################################################
if __name__ == '__main__':

    # *******************************************************************************
    #  Parse input file                                                             *
    # *******************************************************************************/
    fileName:str = "sample-laser-radar-measurement-data-1.txt"
    par = Parser(fileName, startLineNo=0, allLines=True, laser=True, radar=True, delimiter="\t")
    measPackList, gtPackList = par.extractDataGt()

    # *******************************************************************************
    #  Run Kalman Filter and save the output                                        *
    # *******************************************************************************/
    size = len(measPackList)
    xEstimated, yEstimated, vEstimated, vxEstimated, vyEstimated  = np.zeros((size, 1)), np.zeros((size, 1)), np.zeros((size, 1)), np.zeros((size, 1)), np.zeros((size, 1))
    xGt, yGt, vGt, vxGt, vyGt = np.zeros((size, 1)), np.zeros((size, 1)), np.zeros((size, 1)), np.zeros((size, 1)), np.zeros((size, 1))
    xMeas, yMeas  = np.zeros((size, 1)), np.zeros((size, 1))
    k = np.zeros(size)

    trackf = TrackFusion(4)
    for counter in range(size):
        #Call KF process measurement
        trackf.processMeasurement(measPackList[counter])

        # save Estimation/ground Truth/ Measurement

        # save Estimation
        xEstimated[counter]  = trackf.kd.x[0]
        yEstimated[counter]  = trackf.kd.x[1]
        vx = trackf.kd.x[2]
        vy = trackf.kd.x[3]
        vxEstimated[counter] = vx
        vyEstimated[counter] = vy
        vEstimated[counter]  = (math.sqrt(vx**2 + vy**2))

        # save ground Truth
        xGt[counter] = gtPackList[counter].rawMeasurement[0]
        yGt[counter] = gtPackList[counter].rawMeasurement[1]
        vxGtTemp = gtPackList[counter].rawMeasurement[2]
        vyGtTemp = gtPackList[counter].rawMeasurement[3]
        vxGt[counter] = vxGtTemp
        vyGt[counter] = (vyGtTemp)
        vGt[counter]  = math.sqrt(vxGtTemp**2 + vyGtTemp**2)

        # save measurement
        if measPackList[counter].sensorType == sensotType.LASER:
            xMeas[counter] = measPackList[counter].rawMeasurement[0]
            yMeas[counter] = measPackList[counter].rawMeasurement[1]
        elif measPackList[counter].sensorType == sensotType.RADAR:
            rho = measPackList[counter].rawMeasurement[0]
            phi = measPackList[counter].rawMeasurement[1]
            xMeas[counter] = (rho * math.cos(phi))
            yMeas[counter] = (rho * math.sin(phi))

        #save time step
        k[counter] = counter

    # *******************************************************************************
    #  Run Kalman Filter and save the output                                        *
    # *******************************************************************************/
    rmsePx, rmsePy, rmseVx, rmseVy = get_RMSE(xEstimated, yEstimated, vxEstimated, vyEstimated,
                                              xGt, yGt, vxGt, vyGt)
    print("rmse Values")
    print("px=%f, py=%f, vx=%f, vy=%f"%(rmsePx, rmsePy, rmseVx, rmseVy))
    # *******************************************************************************
    #  Run Kalman Filter and save the output                                        *
    # *******************************************************************************/
    plt.title("object_trajectory ")
    plt.xlabel("x-pos meter")
    plt.ylabel("y-pos meter")
    plt.plot(xEstimated,yEstimated,linewidth=1, color="blue" ,linestyle='dashdot')
    plt.plot(xGt,yGt,linewidth=1,color="green", linestyle='dashed')
    plt.plot(xMeas, yMeas, linewidth=0.5,color="red", linestyle='dotted')
    plt.grid(True)
    plt.legend(["EST", "GT", "Meas"])
    plt.savefig(fileName+"-pos.jpg")

    plt.title("velocity")
    plt.xlabel("v")
    plt.ylabel("timeStep")
    plt.plot(k, vEstimated,linewidth=1, color="blue" ,linestyle='dashdot')
    plt.plot(k,vGt, linewidth=1,color="green", linestyle='dashed')
    plt.grid(True)
    plt.legend(["EST", "GT"])
    plt.savefig(fileName+"-velocity.jpg")

    plt.title("velocity X")
    plt.xlabel("vx")
    plt.ylabel("timeStep")
    plt.plot(k, vxEstimated,linewidth=1, color="blue" ,linestyle='dashed')
    plt.plot(k,vxGt, linewidth=1,color="green", linestyle='dashed')
    plt.grid(True)
    plt.legend(["EST", "GT"])
    plt.savefig(fileName+"-velocityX.jpg")

    plt.title("velocity Y")
    plt.xlabel("vy")
    plt.ylabel("timeStep")
    plt.plot(k, vyEstimated,linewidth=1, color="blue" ,linestyle='dashed')
    plt.plot(k,vyGt, linewidth=1,color="green", linestyle='dashed')
    plt.grid(True)
    plt.legend(["EST", "GT"])
    plt.savefig(fileName+"-velocityY.jpg")

########################################################################################################################