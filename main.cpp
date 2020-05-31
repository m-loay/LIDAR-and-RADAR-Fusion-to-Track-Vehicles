/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "KalmanConfig.h"

#ifdef USE_HIGHWAY
#include "highway.h"
#include "tools.h"
int main(int argc, char** argv)
{

	//create visulaizer object
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	//create tools object
	Tools tools;

	// set camera position and angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);

	Highway highway(viewer);

	//initHighway(viewer);

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	double egoVelocity = 25;

	while (frame_count < (frame_per_sec*sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		highway.stepHighway(egoVelocity,time_us, frame_per_sec, viewer, tools);
		viewer->spinOnce(1000/frame_per_sec);
		frame_count++;
		time_us = 1000000*frame_count/frame_per_sec;
	}
	tools.plotData("highway");
}
#endif


#ifdef USE_TEST
#include "tools.h"

#ifdef USE_EKF
#include "kfApp.h"
#endif

#ifdef USE_UKF
#include "ukfApp.h"
#endif

int main(int argc, char* argv[])
{
  /*******************************************************************************
   *  Parse input file                                                         *
   *******************************************************************************/
  Tools tools;
  tools.readData(argc,argv);

  /*******************************************************************************
   *  Run Kalman Filter and save the output                                    *
   *******************************************************************************/
#if defined(USE_EKF)
  // Create a KF instance
  kfApp tracking(4);
#endif

#if defined(USE_UKF)
  // Create a KF instance
  ukfApp tracking(5,2);
#endif

  // start filtering from the second frame (the speed is unknown in the first frame)
  size_t N = tools.measurement_pack_list.size();

  for (size_t k = 0; k < N; ++k)
  {
    // convert tracking x vector to cartesian to compare to ground truth
    Eigen::VectorXd temp_ukf_x_cartesian = Eigen::VectorXd(4);

    // Call the KF-based fusion
    tracking.ProcessMeasurement(tools.measurement_pack_list[k]);

      // 2.output the measurements
    if (tools.measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER)
    {
      tools.nis_lidar.push_back(tracking.kd_.NIS);
    }
    else if (tools.measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR)
    {
      tools.nis_radar.push_back(tracking.kd_.NIS);
    }

#if defined(USE_EKF)
    // 1.output the estimation
    double x  = tracking.kd_.x(kfApp::XPOS) ; // pos1 - est
    double y  = tracking.kd_.x(kfApp::YPOS) ; // pos2 - est
    double vx = tracking.kd_.x(kfApp::XVEL) ; // vx -est
    double vy = tracking.kd_.x(kfApp::YVEL) ; // vy -est
    temp_ukf_x_cartesian << x, y, vx, vy;
    tools.estimations.push_back(temp_ukf_x_cartesian);
#endif

#if defined(USE_UKF)
    // 1.output the estimation
    double x  = tracking.kd_.x(ukfApp::XPOS); // pos1 - est
    double y  = tracking.kd_.x(ukfApp::YPOS); // pos2 - est
    double vx = tracking.kd_.x(ukfApp::VEL)* cos(tracking.kd_.x(ukfApp::THETA)); // vx - calculated from v & theta
    double vy = tracking.kd_.x(ukfApp::VEL)* sin(tracking.kd_.x(ukfApp::THETA)); // vy - calculated from v & theta
    double v  = sqrt(pow(vx,2)*pow(vy,2)); // v -est
    temp_ukf_x_cartesian << x, y, vx, vy;
    tools.estimations.push_back(temp_ukf_x_cartesian);
#endif
  }
  // compute the accuracy (RMSE)
  std::cout << "Accuracy - RMSE:" << std::endl; 
  std::cout<< tools.CalculateRMSE(tools.estimations, tools.ground_truth) << std::endl;


  /*******************************************************************************
   *  plot the results                                                           *
   *******************************************************************************/
  std::string dataName; 
#if defined(USE_EKF)
  dataName = "EKF";
#endif

#if defined(USE_UKF)
  dataName = "UKF";
#endif

  tools.plotData(dataName);

  return 0;
}
#endif

