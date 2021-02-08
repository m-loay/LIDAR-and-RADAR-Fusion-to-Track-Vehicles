/*
 * tools.cpp
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file tools.cpp
 *  @ingroup Tool
 *  @brief Tools class
 */

/**
 *  @addtogroup Tool
 *  @{
 */

#include <iostream>
#include <random>
#include "tools.h"

/**
 * @brief kfApp The constructor for Tools.
 *
 */
Tools::Tools() {}

/**
 * @brief Tools The destructor for Tools.
 *
 */
Tools::~Tools() {}

#ifdef USE_HIGHWAY
/**
 * @brief savePcd Saves the output of simulation of PCL using .pcd format.
 *
 * @param[in] cloud which is a point cloud data object {PointCloud<pcl::PointXYZ>::Ptr}.
 *
 */
void Tools::savePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file)
{
  pcl::io::savePCDFileASCII (file, *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

/**
 * @brief loadPcd loads the output of simulation of PCL using .pcd format.
 *
 * @param[in] file The pcd file name {string}.
 *
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Tools::loadPcd(std::string file)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }
  //std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

  return cloud;
}
#endif

/**
 * @brief CalculateRMSE Calculated the Root Mean Square.
 *
 * @param[in] estimations The output of the filter {VectorXd}.
 * 
 * @param[in] ground_truth The ground truth corresponding to filter's output {VectorXd}.
 * 
 * @param[out]  Vector contains the RMSE for each state {VectorXd}.
 *
 */
Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                     const std::vector<Eigen::VectorXd> &ground_truth) 
{
  
    Eigen::VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0)
	{
		std::cout << "Invalid estimation or ground_truth data" << std::endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i)
	{

		Eigen::VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();


	//return the result
	return rmse;
}

#if (defined USE_TEST) || (defined USE_HIGHWAY)

/**
 * @brief plotData extract the data in class members estimations,ground_truth and measurement_pack_list
 *        then plot the data using gnuplot library.
 *
 */
void Tools::plotData(std::string type)
{
   /*******************************************************************************
   *  extract data from (Estimations,gt,measurement)                              *
   *******************************************************************************/
  //
  std::string opName = type+"_"+name;
  int N =  ground_truth.size();;
  // create variables for plotting
  std::vector<double> x(N),y(N),vx(N),vy(N),x_meas(N),y_meas(N),x_gt(N),y_gt(N)
                     ,vx_gt(N),vy_gt(N),v(N),v_gt(N),time(N);

   for(int k=0; k<N ; k++)
   {
		// 1.output the estimation
		Eigen::VectorXd estimate = estimations[k];
		x.at(k)  = estimate(0); // pos1 - est
		y.at(k)  = estimate(1); // pos2 - est
		vx.at(k) = estimate(2); // vx - calculated from v & theta
		vy.at(k) = estimate(3); // vy - calculated from v & theta
		v.at(k)  = sqrt(pow(vx.at(k),2)*pow(vy.at(k),2)); // v -est

	   // 2.output the measurements
	   if(measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER)
	   {
			// output the estimation in the cartesian coordinates
			x_meas.at(k) = measurement_pack_list[k].raw_measurements_(0);// pos1 - meas
			y_meas.at(k) = measurement_pack_list[k].raw_measurements_(1);// pos2 - meas
	   }
	   if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR)
	   {
			// output the estimation in the cartesian coordinates
			float ro     = measurement_pack_list[k].raw_measurements_(0);// ro - meas
			float phi    = measurement_pack_list[k].raw_measurements_(1);// phi - meas
			x_meas.at(k) = measurement_pack_list[k].raw_measurements_(0)* cos(phi);
			y_meas.at(k) = measurement_pack_list[k].raw_measurements_(1)* sin(phi);
	   }

	    // 3.output the ground truth packages
		Eigen::VectorXd gt = ground_truth[k];
		x_gt.at(k)  = gt(0);// pos1 - gt
		y_gt.at(k)  = gt(1);// pos2 - gt
		vx_gt.at(k) = gt(2);// vx - calculated from v & theta
		vy_gt.at(k) = gt(3);// vy - calculated from v & theta
		v_gt.at(k)  = sqrt(pow(vx_gt.at(k),2)*pow(vy_gt.at(k),2));

		// 4.output the ground truth packages
		time.at(k) = k;
    }
	std::cout<<"finish data" <<std::endl;

	/*******************************************************************************
	 *  plot the results                                                           *
	 *******************************************************************************/
	// Show on screen and
	// export to .png, .eps, .pdf, .html, .svg files
	eggp::Eggplot plt(eggp::PNG);
	
	// Plot line from given x and y data. Color is selected automatically.
	plt.title(opName+"--"+"gt VS Meas VS Estimated");
	plt.plot({x,y,x_gt, y_gt,x_meas,y_meas});
	plt.xlabel("pos-x (m)");
	plt.ylabel("pos-y (m)");
	plt.linespec(1, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "*"}});
	plt.linespec(2, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "."}});
	plt.linespec(3, {{eggp::Marker, "*"}, {eggp::LineStyle, "none"}});
	plt.linespec(3, {{eggp::MarkerSize, "0.3"}});
	plt.linespec(1, eggp::Color, "b");
	plt.linespec(2, eggp::Color, "g");
	plt.linespec(3, eggp::Color, "r");
	plt.legend({"es", "gt","meas"});
	plt.grid(true);
	plt.print(opName+"--"+"Output estimation");
	plt.exec();

	plt.title(opName+"--"+"Vxgt VS VxEstimated");
	plt.plot({time,vx,time,vx_gt});
	plt.xlabel("steps");
	plt.ylabel("velocity (m/s)");
	plt.linespec(1, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "*"}});
	plt.linespec(2, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "."}});
	plt.linespec(1, eggp::Color, "b");
	plt.linespec(2, eggp::Color, "g");
	plt.legend({"es", "gt"});
	plt.grid(true);
	plt.print(opName+"--"+"Output Vx");
	plt.exec();

	plt.title(opName+"--"+"Vygt VS VyEstimated");
	plt.plot({time,vy,time,vy_gt});
	plt.xlabel("steps");
	plt.ylabel("velocity (m/s)");
	plt.linespec(1, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "*"}});
	plt.linespec(2, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "."}});
	plt.linespec(1, eggp::Color, "b");
	plt.linespec(2, eggp::Color, "g");
	plt.legend({"es", "gt"});
	plt.grid(true);
	plt.print(opName+"--"+"Output Vy");
	plt.exec();

	plt.title(opName+"--"+"Vgt VS VEstimated");
	plt.plot({time,v,time,v_gt});
	plt.xlabel("steps");
	plt.ylabel("velocity (m/s)");
	plt.linespec(1, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "*"}});
	plt.linespec(2, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "."}});
	plt.linespec(1, eggp::Color, "b");
	plt.linespec(2, eggp::Color, "g");
	plt.legend({"es", "gt"});
	plt.grid(true);
	plt.print(opName+"--"+"Output V");
	plt.exec();

	int nis_size = nis_lidar.size();
	std::vector<double> ref_nis(nis_size,5.991);
	std::vector<double> time_nis;
	time_nis.assign(time.begin(),time.begin()+nis_size);

	plt.title(opName+"--"+"NIS LAser");
	plt.plot({time_nis,nis_lidar,time_nis,ref_nis});
	plt.xlabel("steps");
	plt.ylabel("chi^2");
	plt.linespec(1, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "*"}});
	plt.linespec(2, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "."}});
	plt.linespec(1, eggp::Color, "b");
	plt.linespec(2, eggp::Color, "g");
	plt.legend({"es", "gt"});
	plt.grid(true);
	plt.print(opName+"--"+"NIS_laser");
	plt.exec();

	nis_size = nis_radar.size();
	std::fill(nis_radar.begin(),nis_radar.begin()+3,0.0);
	ref_nis.clear();
	ref_nis.assign(nis_size,7.815);
	time_nis.clear();
	time_nis.assign(time.begin(),time.begin()+nis_size);

	plt.title(opName+"--"+"NIS Radar");
	plt.plot({time_nis,nis_radar,time_nis,ref_nis});
	plt.xlabel("steps");
	plt.ylabel("chi^2");
	plt.linespec(1, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "*"}});
	plt.linespec(2, {{eggp::MarkerSize, "0.5"}, {eggp::Marker, "."}});
	plt.linespec(1, eggp::Color, "b");
	plt.linespec(2, eggp::Color, "g");
	plt.legend({"es", "gt"});
	plt.grid(true);
	plt.print(opName+"--"+"NIS Radar");
	plt.exec();
	std::cout<<"finish plot" <<std::endl;

}

/**
 * @brief check_arguments Checks if the sent arguments are valid.
 *
 * @param[in] argc which is number of passed arguments {int}.
 * 
 * @param[in] argv which contains the external arguments {char*}.
 *
 */
void Tools::check_arguments(int argc, char* argv[])
{
  std::string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1)
  {
    std::cerr << usage_instructions << std::endl;
  }
  else if(argc == 2)
  {
    has_valid_args = true;
  }
  else if (argc > 2)
  {
    std::cerr << "Too many arguments.\n" << usage_instructions << std::endl;
  }
  else
  {

  }

  if (!has_valid_args)
  {
    exit(EXIT_FAILURE);
  }	
}

/**
 * @brief check_files check if file is exist and it can be opened.
 *
 * @param[in] in_file Handle object to file for(read/write) operations {ifstream}.
 * 
 * @param[in] in_name file name {string}.
 *
 */
void Tools::check_files (std::ifstream& in_file, std::string& in_name)
{
  if (!in_file.is_open())
  {
    std::cerr << "Cannot open input file: " << in_name << std::endl;
    exit(EXIT_FAILURE);
  }
}

/**
 * @brief readData Read the data from .txt file and parse all data to class member.
 *
 * @param[in] argc which is number of passed arguments {int}.
 * 
 * @param[in] argv which contains the external arguments {char*}.
 *
 */
void Tools::readData(int argc, char* argv[])
{
  /*******************************************************************************
   *  Parse input file                                                         *
   *******************************************************************************/
  check_arguments(argc, argv);

  std::string in_file_name_ = argv[1];
  std::ifstream in_file_(in_file_name_.c_str(), std::ifstream::in);
  std::size_t found = in_file_name_.find_last_of("/\\");
  std::size_t found2 = in_file_name_.find_last_of(".");

  name = in_file_name_.substr(found+1, (found2 - found - 1));


  check_files(in_file_, in_file_name_);

  /**********************************************
   *  Set Measurements & grountruth             *
   **********************************************/
  // prep the measurement packages (each line represents a measurement at a timestamp)
  std::string line;

  while (getline(in_file_, line))
  {
    MeasurementPackage meas_package;
    std::string sensor_type;
    std::istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0)
    {
      // laser measurement
      // read measurements at this timestamp

      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = Eigen::VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }
    else if (sensor_type.compare("R") == 0)
    {
    //   radar measurement
    //   read measurements at this timestamp

      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = Eigen::VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    Eigen::VectorXd gt = Eigen::VectorXd(4);
    gt << x_gt, y_gt, vx_gt, vy_gt;
    ground_truth.push_back(gt);
  }

  if (in_file_.is_open())
  {
  	in_file_.close();
  }
}
#endif
/**
 *  @}
 */