/*
 * tools.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file tools.h
 *  @ingroup Tool
 *  @brief Tools class
 */

/**
 *  @addtogroup Tool
 *  @{
 */

#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "KalmanConfig.h"

#if (defined USE_TEST) || (defined USE_HIGHWAY)
#include "chebyshev.h"
#include "eggplot.h"
#endif

#ifdef USE_HIGHWAY
#include <pcl/io/pcd_io.h>
#endif

class Tools 
{
	public:
	/**
	* Constructor.
	*/
	Tools();
	
	/**
	* Destructor.
	*/
	virtual ~Tools();
	
	// Members
	std::vector<MeasurementPackage> measurement_pack_list;
	std::vector<Eigen::VectorXd> ground_truth;
	std::vector<Eigen::VectorXd> estimations;
	std::vector<double> nis_radar;
	std::vector<double> nis_lidar;
	std::string name;
	/**
	* A helper method to calculate RMSE.
	*/
#ifdef USE_HIGHWAY
	// savePcd save the pcd format of pcl.
	void savePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file);

	// loadPcd load the pcd format of pcl.
	pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcd(std::string file);
#endif
 
	//CalculateRMSE Calculates the root mean square error
	Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                  const std::vector<Eigen::VectorXd> &ground_truth);

#if (defined USE_TEST) || (defined USE_HIGHWAY)
	//CalculateRMSE Calculates the root mean square error
	void plotData(std::string op);

	//CalculateRMSE Calculates the root mean square error
	void check_arguments(int argc, char* argv[]);

	//CalculateRMSE Calculates the root mean square error
	void check_files(std::ifstream& in_file, std::string& in_name);

	//CalculateRMSE Calculates the root mean square error
	void readData(int argc, char* argv[]);
#endif
	
};

#endif /* TOOLS_H_ */
/**
 *  @}
 */
