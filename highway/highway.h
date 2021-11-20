/* \author Aaron Brown */
// Handle logic for creating traffic on highway and animating it

/** @file highway.h
 *  @ingroup highway
 *  @brief highway class
 */

/**
 *  @addtogroup highway
 *  @{
 */

#ifndef HIGHWAY_PACKAGE_H_
#define HIGHWAY_PACKAGE_H_

#include "render/render.h"
#include "sensors/lidar.h"
#include "measurement_package.h"
#include <random>
#include <pcl/io/pcd_io.h>
#include "tools.h"
struct lmarker
{
	double x, y;
	lmarker(double setX, double setY)
		: x(setX), y(setY)
	{}

};

struct rmarker
{
	double rho, phi, rho_dot;
	rmarker(double setRho, double setPhi, double setRhoDot)
		: rho(setRho), phi(setPhi), rho_dot(setRhoDot)
	{}

};

class Highway
{
	public:
	std::vector<Car> traffic;
	Car egoCar;
	bool pass = true;
	std::vector<double> rmseThreshold = {0.30,0.16,0.95,0.70};
	std::vector<double> rmseFailLog = {0.0,0.0,0.0,0.0};
	Lidar* lidar;
	
	// Parameters 
	// --------------------------------
	// Set which cars to track with UKF
	std::vector<bool> trackCars = {true,true,true};
	// Visualize sensor measurements
	bool visualize_lidar = true;
	bool visualize_radar = true;
	bool visualize_pcd = false;
	// Predict path in the future using UKF
	double projectedTime = 0;
	int projectedSteps = 0;
	// --------------------------------
	double noise(double stddev, long long seedNum);

	lmarker lidarSense(Car& car, pcl::visualization::PCLVisualizer::Ptr& viewer,
	                   std::vector<MeasurementPackage> &xy_position,
					   std::vector<double>&nis, long long timestamp, bool visualize);

	rmarker radarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer,
	                   std::vector<MeasurementPackage> &xy_position,
					   std::vector<double>&nis, long long timestamp, bool visualize);

	void ukfResults(Car car, pcl::visualization::PCLVisualizer::Ptr& viewer, double time, int steps);

	Highway(pcl::visualization::PCLVisualizer::Ptr& viewer);
	
	void stepHighway(double egoVelocity, long long timestamp, int frame_per_sec,
	                 pcl::visualization::PCLVisualizer::Ptr& viewer , Tools &tools);

};
#endif /* HIGHWAY_PACKAGE_H_*/
/**
 *  @}
 */