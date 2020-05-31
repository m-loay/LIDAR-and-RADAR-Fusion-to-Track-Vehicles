# kfApp(Pedestrian Tracking)

- In this application , It is required to detect a pedestrian moving(using only lidar sensor) from p1(x1,y1) -->p2(x2,y2) as shown in the following figure:

 ![](1_ped.PNG)

- To detect the pedestrian , a mathematical model is required.In this application  the pedestrian moves at constant velocity , actually this is not realistic , also there is no information about the acceleration , so it will be the stochastic part as shown in the following equations:

 ![](2_pd_motion_model.PNG)

where:

 ![](3_pd_motion_model2.PNG)

- After the derivation of stochastic parts , the motion model will be as following:

 ![](4_pd_motion_model3.PNG)

where:

 ![](5_pd_motion_model4.PNG)

- The final form of the process model(prediction model for standard KF)  will be as following :

 ![](6_pred_model_1.PNG)
 ![](7_pred_model_2.PNG)

- The measurement model for pedestrian tracking as follow:

 ![](8_meas_model_1.PNG)
Where the measurements are px , py the position of pedestrian in x-y coordinates using laser sensor .

- The innovation = The measurement vectore - predicted vector
 ![](9_meas_model_2.PNG)
 H will be identity matrix[1*2]

- The sensor used to detect the pedestrian position is LIDAR (Light Detection and Ranging).
- LIDAR uses infrared laser beam to determine the distance between the sensor and the object.
- LIDAR perform better even in case of rains and fog environment.
- LIDAR is the main source for generating point cloud.


