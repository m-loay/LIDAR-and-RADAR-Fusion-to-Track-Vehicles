/* \author Aaron Brown */
// Handle logic for creating traffic on highway and animating it

/** @file highway.cpp
 *  @ingroup highway
 *  @brief highway class
 */

/**
 *  @addtogroup highway
 *  @{
 */

#include "highway.h"

/**
 * @brief Highway a constructor which creates the rods, behicles and the sensors..
 *
 * @param[in] viewer Handle object to change and visulize the state of the created objects
 *       {pcl::visualization::PCLVisualizer::Ptr}.
 *
 */
Highway::Highway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

	egoCar = Car(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), 0, 0, 2, "egoCar");
	UKF ukf0(5,2);
	egoCar.setUKF(ukf0);
	
	Car car1(Vect3(-10, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), 5, 0, 2, "car1");
	
	std::vector<accuation> car1_instructions;
	accuation a = accuation(0.5*1e6, 0.5, 0.0);
	car1_instructions.push_back(a);
	a = accuation(2.2*1e6, 0.0, -0.2);
	car1_instructions.push_back(a);
	a = accuation(3.3*1e6, 0.0, 0.2);
	car1_instructions.push_back(a);
	a = accuation(4.4*1e6, -2.0, 0.0);
	car1_instructions.push_back(a);

	car1.setInstructions(car1_instructions);
	if( trackCars[0] )
	{
		UKF ukf1(5,2);
		car1.setUKF(ukf1);
	}
	traffic.push_back(car1);
	
	Car car2(Vect3(25, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), -6, 0, 2, "car2");
	std::vector<accuation> car2_instructions;
	a = accuation(4.0*1e6, 3.0, 0.0);
	car2_instructions.push_back(a);
	a = accuation(8.0*1e6, 0.0, 0.0);
	car2_instructions.push_back(a);
	car2.setInstructions(car2_instructions);
	if( trackCars[1] )
	{
		UKF ukf2(5,2);
		car2.setUKF(ukf2);
	}
	traffic.push_back(car2);

	Car car3(Vect3(-12, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), 1, 0, 2, "car3");
	std::vector<accuation> car3_instructions;
	a = accuation(0.5*1e6, 2.0, 1.0);
	car3_instructions.push_back(a);
	a = accuation(1.0*1e6, 2.5, 0.0);
	car3_instructions.push_back(a);
	a = accuation(3.2*1e6, 0.0, -1.0);
	car3_instructions.push_back(a);
	a = accuation(3.3*1e6, 2.0, 0.0);
	car3_instructions.push_back(a);
	a = accuation(4.5*1e6, 0.0, 0.0);
	car3_instructions.push_back(a);
	a = accuation(5.5*1e6, -2.0, 0.0);
	car3_instructions.push_back(a);
	a = accuation(7.5*1e6, 0.0, 0.0);
	car3_instructions.push_back(a);
	car3.setInstructions(car3_instructions);
	if( trackCars[2] )
	{
		UKF ukf3(5,2);
		car3.setUKF(ukf3);
	}
	traffic.push_back(car3);

	lidar = new Lidar(traffic,0);

	// render environment
	renderHighway(0,viewer);
	egoCar.render(viewer);
	car1.render(viewer);
	car2.render(viewer);
	car3.render(viewer);
}

/**
 * @brief stepHighway The cfunction moves all objects in the created scene
 *
 * @param[in] egoVelocity which is ego vehicle velocity in meter per second{double}.
 *
 * @param[in] timestamp the recorded time for each step in micro-second{long long}.
 * 
 * @param[in] frame_per_sec Number of refreshed frame per second from the created scene{int}.
 * 
 * @param[in] viewer Handle object to change and visulize the state of the created objects
 *       {pcl::visualization::PCLVisualizer::Ptr}.
 * 
 * @param[in] tools used to record estimation,measurements and ground truth
 *            Also used to calculate the RMS{Tools}.
 */
void Highway:: stepHighway(double egoVelocity, long long timestamp, int frame_per_sec,
                           pcl::visualization::PCLVisualizer::Ptr& viewer, Tools &tools)
{

	if(visualize_pcd)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr trafficCloud = tools.loadPcd("../src/sensors/data/pcd/highway_"+
		                                                   std::to_string(timestamp)+".pcd");
		renderPointCloud(viewer, trafficCloud, "trafficCloud", Color((float)184/256,(float)223/256,(float)252/256));
	}
	
	// render highway environment with poles
	renderHighway(egoVelocity*timestamp/1e6, viewer);
	egoCar.render(viewer);
	
	for (int i = 0; i < traffic.size(); i++)
	{
		traffic[i].move((double)1/frame_per_sec, timestamp);
		if(!visualize_pcd)
			traffic[i].render(viewer);
		// Sense surrounding cars with lidar and radar
		if(trackCars[i])
		{
			Eigen::VectorXd gt(4);
			gt << traffic[i].position.x, traffic[i].position.y,
			       traffic[i].velocity*cos(traffic[i].angle), traffic[i].velocity*sin(traffic[i].angle);
			tools.ground_truth.push_back(gt);
			lidarSense(traffic[i], viewer, tools.measurement_pack_list,tools.nis_lidar, timestamp, visualize_lidar);
			radarSense(traffic[i], egoCar, viewer, tools.measurement_pack_list, tools.nis_radar, timestamp, visualize_radar);
			ukfResults(traffic[i],viewer, projectedTime, projectedSteps);
			Eigen::VectorXd estimate(4);
			double v  = traffic[i].ukf.kd_.x(2);
			double yaw = traffic[i].ukf.kd_.x(3);
			double v1 = cos(yaw)*v;
			double v2 = sin(yaw)*v;
			estimate << traffic[i].ukf.kd_.x[0], traffic[i].ukf.kd_.x[1], v1, v2;
			tools.estimations.push_back(estimate);
		}
	}
	viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
	Eigen::VectorXd rmse = tools.CalculateRMSE(tools.estimations, tools.ground_truth);
	viewer->addText(" X: "+std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
	viewer->addText(" Y: "+std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
	viewer->addText("Vx: "	+std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
	viewer->addText("Vy: "	+std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");

	if(timestamp > 1.0e6)
	{

		if(rmse[0] > rmseThreshold[0])
		{
			rmseFailLog[0] = rmse[0];
			pass = false;
		}
		if(rmse[1] > rmseThreshold[1])
		{
			rmseFailLog[1] = rmse[1];
			pass = false;
		}
		if(rmse[2] > rmseThreshold[2])
		{
			rmseFailLog[2] = rmse[2];
			pass = false;
		}
		if(rmse[3] > rmseThreshold[3])
		{
			rmseFailLog[3] = rmse[3];
			pass = false;
		}
	}
	if(!pass)
	{
		viewer->addText("RMSE Failed Threshold", 30, 150, 20, 1, 0, 0, "rmse_fail");
		if(rmseFailLog[0] > 0)
			viewer->addText(" X: "+std::to_string(rmseFailLog[0]), 30, 125, 20, 1, 0, 0, "rmse_fail_x");
		if(rmseFailLog[1] > 0)
			viewer->addText(" Y: "+std::to_string(rmseFailLog[1]), 30, 100, 20, 1, 0, 0, "rmse_fail_y");
		if(rmseFailLog[2] > 0)
			viewer->addText("Vx: "+std::to_string(rmseFailLog[2]), 30, 75, 20, 1, 0, 0, "rmse_fail_vx");
		if(rmseFailLog[3] > 0)
			viewer->addText("Vy: "+std::to_string(rmseFailLog[3]), 30, 50, 20, 1, 0, 0, "rmse_fail_vy");
	}
}

/**
 * @brief noise add noise to the measurements.
 *
 * @param[in] stddev the standard deviation used the add the noise{double}.
 *
 * @param[in] seedNum {long long}.
 * 
 */
double Highway::noise(double stddev, long long seedNum)
{
	std::mt19937::result_type seed = seedNum;
	auto dist = std::bind(std::normal_distribution<double>{0, stddev}, std::mt19937(seed));
	return dist();
}

/**
 * @brief lidarSense Extract the lidar data from scene and send the measuremnts to ukf and ground truth
 *        to the tools.
 *
 * @param[in] car the standard deviation used the add the noise{Car}.
 *
 * @param[in] xy_position a storage to record lidar data {vector<MeasurementPackage>}.
 * 
 * @param[in] nis a storage to record The normalised innovation squared for each measurement {std::vector<double>}.
 * 
 * @param[in] timestamp the recorded time for each step in micro-second{long long}.
 * 
 * @param[in] visualize is boolean to display/un-display the lidar data {bool}.
 * 
 * @param[in] marker moves the vehicle based on lidar scene{lmarker}.
 * 
 */
lmarker Highway::lidarSense(Car& car, pcl::visualization::PCLVisualizer::Ptr& viewer,
	                        std::vector<MeasurementPackage> &xy_position,
							std::vector<double>&nis, long long timestamp, bool visualize)
{
	MeasurementPackage meas_package;
	meas_package.sensor_type_ = MeasurementPackage::LASER;
  	meas_package.raw_measurements_ = Eigen::VectorXd(2);

	lmarker marker = lmarker(car.position.x + noise(0.15,timestamp), car.position.y + noise(0.15,timestamp+1));
	if(visualize)
		viewer->addSphere(pcl::PointXYZ(marker.x,marker.y,3.0),0.5, 1, 0, 0,car.name+"_lmarker");

    meas_package.raw_measurements_ << marker.x, marker.y;
    meas_package.timestamp_ = timestamp;
	xy_position.push_back(meas_package);

    car.ukf.ProcessMeasurement(meas_package);
	nis.push_back(car.ukf.kd_.nis);

    return marker;
}


/**
 * @brief radarSense Extract the radar data from scene and send the measuremnts to ukf and ground truth
 *        to the tools.
 *
 * @param[in] car the handle of all objects in the scene{Car}.
 *
 * @param[in] xy_position a storage to record lidar data {vector<MeasurementPackage>}.
 * 
 * @param[in] nis a storage to record The normalised innovation squared for each measurement {std::vector<double>}.
 * 
 * @param[in] timestamp the recorded time for each step in micro-second{long long}.
 * 
 * @param[in] visualize is boolean to display/un-display the radar data {bool}.
 * 
 * @param[in] marker moves the vehicle based on lidar scene{lmarker}.
 * 
 */
rmarker Highway::radarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer,
	                        std::vector<MeasurementPackage> &xy_position,
							std::vector<double>&nis, long long timestamp, bool visualize)
{
	double rho = sqrt((car.position.x-ego.position.x)*(car.position.x-ego.position.x)+(car.position.y-ego.position.y)*(car.position.y-ego.position.y));
	double phi = atan2(car.position.y-ego.position.y,car.position.x-ego.position.x);
	double rho_dot = (car.velocity*cos(car.angle)*rho*cos(phi) + car.velocity*sin(car.angle)*rho*sin(phi))/rho;

	rmarker marker = rmarker(rho+noise(0.3,timestamp+2), phi+noise(0.03,timestamp+3), rho_dot+noise(0.3,timestamp+4));
	if(visualize)
	{
		viewer->addLine(pcl::PointXYZ(ego.position.x, ego.position.y, 3.0), pcl::PointXYZ(ego.position.x+marker.rho*cos(marker.phi), ego.position.y+marker.rho*sin(marker.phi), 3.0), 1, 0, 1, car.name+"_rho");
		viewer->addArrow(pcl::PointXYZ(ego.position.x+marker.rho*cos(marker.phi), ego.position.y+marker.rho*sin(marker.phi), 3.0), pcl::PointXYZ(ego.position.x+marker.rho*cos(marker.phi)+marker.rho_dot*cos(marker.phi), ego.position.y+marker.rho*sin(marker.phi)+marker.rho_dot*sin(marker.phi), 3.0), 1, 0, 1, car.name+"_rho_dot");
	}
	
	MeasurementPackage meas_package;
	meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = Eigen::VectorXd(3);
    meas_package.raw_measurements_ << marker.rho, marker.phi, marker.rho_dot;
    meas_package.timestamp_ = timestamp;
	xy_position.push_back(meas_package);

    car.ukf.ProcessMeasurement(meas_package);
	nis.push_back(car.ukf.kd_.nis);

    return marker;
}

// Show UKF tracking and also allow showing predicted future path
// double time:: time ahead in the future to predict
// int steps:: how many steps to show between present and time and future time
/**
 * @brief ukfResults Show UKF tracking and also allow showing predicted future path.
 *
 * @param[in] car the handle of all objects in the scene{Car}.
 * 
 * @param[in] time  time ahead in the future to predict{double}.
 * 
 * @param[in] steps moves the vehicle based on lidar scene{int}.
 * 
 */
void Highway::ukfResults(Car car, pcl::visualization::PCLVisualizer::Ptr& viewer, double time, int steps)
{
	UKF ukf = car.ukf;
	viewer->addSphere(pcl::PointXYZ(ukf.kd_.x[0],ukf.kd_.x[1],3.5), 0.5, 0, 1, 0,car.name+"_ukf");
	viewer->addArrow(pcl::PointXYZ(ukf.kd_.x[0], ukf.kd_.x[1],3.5), 
	                 pcl::PointXYZ(ukf.kd_.x[0]+ukf.kd_.x[2]*cos(ukf.kd_.x[3]),
					 ukf.kd_.x[1]+ukf.kd_.x[2]*sin(ukf.kd_.x[3]),3.5), 0, 1, 0, car.name+"_ukf_vel");
	if(time > 0)
	{
		double dt = time/steps;
		double ct = dt;
		while(ct <= time)
		{
			ukf.Prediction(dt);
			viewer->addSphere(pcl::PointXYZ(ukf.kd_.x[0],ukf.kd_.x[1],3.5),
			                  0.5, 0, 1, 0,car.name+"_ukf"+std::to_string(ct));
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-0.8*(ct/time),
			                                    car.name+"_ukf"+std::to_string(ct));
			//viewer->addArrow(pcl::PointXYZ(ukf.x_[0], ukf.x_[1],3.5), pcl::PointXYZ(ukf.x_[0]+ukf.x_[2]*cos(ukf.x_[3]),ukf.x_[1]+ukf.x_[2]*sin(ukf.x_[3]),3.5), 0, 1, 0, car.name+"_ukf_vel"+std::to_string(ct));
			//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-0.8*(ct/time), car.name+"_ukf_vel"+std::to_string(ct));
			ct += dt;
		}
	}
}
/**
 *  @}
 */