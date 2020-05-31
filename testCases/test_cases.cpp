/*
 * TestCase.cpp
 *
 *  Created on: Apr 24, 2019
 *      Author: mloay
 */

/** @file test_cases.cpp
 *  @ingroup testCases
 *  @brief test cases
 */

/**
 *  @addtogroup testCases
 *  @{
 */

#define _USE_MATH_DEFINES

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include"test_cases.h"

Eigen::VectorXd PredictionModel (const Eigen::VectorXd &sig_pred_ , const void *args);
Eigen::VectorXd calc_covar (const Eigen::VectorXd &sig_pred , const void *args);
Eigen::VectorXd PredictionModelMeasurement (const Eigen::VectorXd &sig_pred_ , const void *args);
Eigen::VectorXd calc_covar_measurement (const Eigen::VectorXd &sig_pred , const void *args);

//Test cases


/**
 * @brief Test case 1 linearFilter
 *        Test The linear update step.
 *
 */
bool linearFilter(void)
{
  /*******************************************************************************
   *  Initialization                                                             *
   *******************************************************************************/
	//set state dimension
	int n_x = 2;

	//set measurement dimension
	int n_z = 1;

	//create kalman data object and configure it
	KalmanData kd;
	kd.m_numState = n_x;

	//create example std::vector for predicted state mean.
	kd.x = Eigen::VectorXd(n_x);
	kd.x << 0, 0;

	//create example matrix for predicted state covariance.
	kd.P = Eigen::MatrixXd(n_x,n_x);
	kd.P.fill(0.0);
	kd.P << 1000, 0, 0, 1000;

	//create state transition matrix for predicted state covariance.
	kd.F = Eigen::MatrixXd(n_x,n_x);
	kd.F<< 1, 1, 0, 1;
	
	//output matrix
	kd.H = Eigen::MatrixXd (n_z, n_x);
	kd.H<< 1, 0;

	//Sensor Noise Covariance matrix
	kd.R = Eigen::MatrixXd(n_z, n_z);
	kd.R.diagonal()<<1.0;

	//precoess noise covariance matrix Q
	kd.Q = Eigen::MatrixXd (n_x, n_x);
	kd.Q.fill(0.0);

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer
	//x-state
	Eigen::VectorXd x_corr = Eigen::VectorXd(n_x);
	std::vector<Eigen::VectorXd> x_corr_list;
	x_corr << 0.999001,0.0;
	x_corr_list.push_back(x_corr);
	x_corr << 2.998,0.999002;
	x_corr_list.push_back(x_corr);
	x_corr << 3.99967,1.0;
	x_corr_list.push_back(x_corr);

	//P-state
	Eigen::MatrixXd p_corr = Eigen::MatrixXd(n_x,n_x);
	std::vector<Eigen::MatrixXd> p_corr_list;
	p_corr << 1001, 1000, 1000, 1000;
	p_corr_list.push_back(p_corr);
	p_corr << 4.99002, 2.99302, 2.99302, 1.99501;
	p_corr_list.push_back(p_corr);
	p_corr << 2.33189, 0.999168, 0.999168, 0.499501;
	p_corr_list.push_back(p_corr);

  /*******************************************************************************
   *  Set Measurement Input                                                      *
   *******************************************************************************/
	//set the measurement
	std::vector<MeasurementPackage> measurement_pack_list; 
	MeasurementPackage meas_package;

	meas_package.raw_measurements_ = Eigen::VectorXd(1);
	meas_package.raw_measurements_ << 1.0;
	measurement_pack_list.push_back(meas_package);
	meas_package.raw_measurements_ << 2.0;
	measurement_pack_list.push_back(meas_package);
	meas_package.raw_measurements_ << 3.0;
	measurement_pack_list.push_back(meas_package);

  /*******************************************************************************
   *  Run Main Algorithm Loop                                                    *
   *******************************************************************************/
  	std::vector<Eigen::VectorXd> x_rest_list;
	std::vector<Eigen::MatrixXd> p_rest_list;
	KfPredict* kfPred = new KfPredictLinear();
	KfUpdate *kfUpdate = new KfUpdateLinear();
	for (unsigned int n = 0; n < measurement_pack_list.size(); ++n)
	{
		// //perform update step
		kd.zpred = kd.H * kd.x;
		kfUpdate->update(kd,measurement_pack_list[n]);

		//perform predict step
		kfPred->predict(kd);

		// //collect result
		x_rest_list.push_back(kd.x);
		p_rest_list.push_back(kd.P);
	}

  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	for (unsigned int n = 0; n < measurement_pack_list.size(); ++n)
	{
		r = r && ((x_rest_list[n] - x_corr_list[n]).norm() < 0.01);
		r = r && ((p_rest_list[n] - p_corr_list[n]).norm() < 0.001);
	}
	return r;
}

/**
 * @brief Test case 2 trackLinearFilter
 *        Test both linear update step and linear prediction step .
 *
 */
bool trackLinearFilter(void)
{
  /*******************************************************************************
   *  Parse input file                                                         *
   *******************************************************************************/
	// hardcoded input file with laser and radar measurements
	std::string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
	std::ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

	if (!in_file.is_open()) 
	{
		std::cout << "Cannot open input file: " << in_file_name_ << std::endl;
	}

  /**********************************************
   *  Set Measurements                          *
   **********************************************/
  // prep the measurement packages (each line represents a measurement at a timestamp)
  std::vector<MeasurementPackage> measurement_pack_list;
  std::string line;
  int i=0;
  while (getline(in_file, line)&& (i<=3))
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
      // radar measurement
      // read measurements at this timestamp
	  continue;
    }
	i++;
  }

  /*******************************************************************************
   *  Run Kalman Filter and save the output                                    *
   *******************************************************************************/
  	std::vector<Eigen::VectorXd> x_rest_list;
	std::vector<Eigen::MatrixXd> p_rest_list;
	// Create a KF instance
	kfApp tracking(4);
	tracking.kd_.x.fill(0);
	tracking.kd_.P = Eigen::MatrixXd(4, 4);
	tracking.kd_.P << 1, 0, 0, 0,
					  0, 1, 0, 0,
					  0, 0, 1000, 0,
					  0, 0, 0, 1000;
	
	tracking.kd_.R = Eigen::MatrixXd(2, 2);
	tracking.kd_.R.fill(0);
	tracking.kd_.R.diagonal()<<2.0,2.0;

	tracking.kd_.H = Eigen::MatrixXd(2, 4);
	tracking.kd_.H << 1, 0, 0, 0,
                      0, 1, 0, 0;
	
	tracking.kd_.F = Eigen::MatrixXd(4, 4);
	tracking.kd_.F << 1, 0, 1, 0,
				      0, 1, 0, 1,
				      0, 0, 1, 0,
				      0, 0, 0, 1;

	tracking.noise_ax = 5;
	tracking.noise_ay = 5;
	tracking.std_laspx_ = 0.15;
	tracking.std_laspy_ = 0.15;

	for (unsigned int n = 0; n < measurement_pack_list.size(); ++n)
	{
		// Call the KF-based fusion
		tracking.ProcessMeasurement(measurement_pack_list[n]);

		//collect result
		if(n>0)
		{
			x_rest_list.push_back(tracking.kd_.x);
			p_rest_list.push_back(tracking.kd_.P);
		}
	}
  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer
	//x-state
	Eigen::VectorXd x_corr = Eigen::VectorXd(4);
	std::vector<Eigen::VectorXd> x_corr_list;
	x_corr << 0.96749,0.405862,4.58427,-1.83232;
	x_corr_list.push_back(x_corr);
	x_corr << 0.958365,0.627631,0.110368, 2.04304;
	x_corr_list.push_back(x_corr);
	x_corr << 1.34291,0.364408, 2.32002,-0.722813;
	x_corr_list.push_back(x_corr);

	//P-state
	Eigen::MatrixXd p_corr = Eigen::MatrixXd(4,4);
	std::vector<Eigen::MatrixXd> p_corr_list;

	p_corr << 0.0224541, 0, 0.204131, 0,
              0, 0.0224541, 0, 0.204131,
              0.204131, 0, 92.7797, 0,
              0, 0.204131, 0, 92.7797;
	p_corr_list.push_back(p_corr);

	p_corr << 0.0220006, 0, 0.210519, 0,
              0, 0.0220006, 0, 0.210519,
              0.210519, 0, 4.08801, 0,
              0, 0.210519, 0, 4.08801;
	p_corr_list.push_back(p_corr);

	p_corr << 0.0185328, 0, 0.109639, 0,
              0, 0.0185328, 0, 0.109639,
              0.109639, 0, 1.10798, 0,
              0, 0.109639, 0, 1.10798;
	p_corr_list.push_back(p_corr);

  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	
	for (unsigned int n = 0; n < measurement_pack_list.size()-1; ++n)
	{
		r = r && ((x_rest_list[n] - x_corr_list[n]).norm() < 0.01);
		r = r && ((p_rest_list[n] - p_corr_list[n]).norm() < 0.01);
	}
	return r;
}

/**
 * @brief Test case 3 CalculateJacobian
 *        Test the jacobian Calculation.
 *
 */
bool CalculateJacobian(void)
{
  /**********************************************
   *  Set jacobia Inputs                        *
   **********************************************/
	// Create a KF instance
	kfApp tracking(4);
	tracking.kd_.x<< 1, 2, 0.2, 0.4;

  /*******************************************************************************
   *  Calculate the Jacobian                                                     *
   *******************************************************************************/
   tracking.kd_.H = tracking.CalculateJacobianH_(tracking.kd_.x);

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer
	//P-state
	Eigen::MatrixXd Hj_corr = Eigen::MatrixXd(3,4);
	Hj_corr << 0.447214, 0.894427, 0, 0,
              -0.4, 0.2, 0, 0,
               0, 0, 0.447214, 0.894427;

  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((tracking.kd_.H -Hj_corr).norm() < 0.01);

	return r;
}

/**
 * @brief Test case 4 calculateRMSE
 *        Test the RMSE Calculation.
 *
 */
bool calculateRMSE(void)
{
  /**********************************************
   *  Set RMSE Inputs                        *
   **********************************************/
	std::vector<Eigen::VectorXd> estimations;
	std::vector<Eigen::VectorXd> ground_truth;
	Tools tools;

	// the input list of estimations
	Eigen::VectorXd e(4);
	e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
	estimations.push_back(e);

	// the corresponding list of ground truth values
	Eigen::VectorXd g(4);
	g << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(g);
	g << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(g);
	g << 3.1, 3.1, 0.5, 0.4;
	ground_truth.push_back(g);

  /*******************************************************************************
   *  Calculate the RMSE                                                    *
   *******************************************************************************/
  	Eigen::VectorXd rmse(4);
	rmse = tools.CalculateRMSE(estimations, ground_truth);

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer
	//rmse correct
	Eigen::VectorXd rmse_correct(4);
	rmse_correct << 0.1, 0.1, 0.1, 0.1;

  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((rmse - rmse_correct).norm() < 0.01);

	return r;
}

/**
 * @brief Test case 5 CalculateSigmaPoints
 *        Test the calculation of sigma points in UT.
 *
 */
bool CalculateSigmaPoints(void)
{
  /**********************************************
   *  Set jacobia Inputs                        *
   **********************************************/
	// Create a kalman data instance
	int nx = 5;
	int sigma = 2*nx+1;
	KalmanData kd;
	kd.setKalmanData(5);
	kd.m_numAddedAug = 0;

	// set example state
	kd.x << 5.7441,
         	1.3800,
			2.2049,
			0.5015,
			0.3528;

	// set example covariance matrix
	kd.P <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
			-0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
			0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
			-0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
			-0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  /*******************************************************************************
   *  Calculate the sigma points                                                 *
   *******************************************************************************/
	// Create a kalman data instance
	KfUt kft;
	kft.setKalmanData(kd,false,false,NULL,NULL,NULL);
	kft.CalculateSigmaPoints();

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer
	Eigen::MatrixXd Xsig = Eigen::MatrixXd(nx,sigma);
	Xsig <<
       5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441,
       1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38,
       2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049,
       0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015,
       0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879;


  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((Xsig -kft.m_sig).norm() < 0.01);

	return r;
}

/**
 * @brief Test case 6 CalculateSigmaPointsAug
 *        Test the calculation of sigma points using Augmented states in UT.
 *
 */
bool CalculateSigmaPointsAug(void)
{
  /**********************************************
   *  Set jacobia Inputs                        *
   **********************************************/
	// Create a kalman data instance
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;
	// Process noise standard deviation longitudinal acceleration in m/s^2
	double std_a = 0.2;

	// Process noise standard deviation yaw acceleration in rad/s^2
	double std_yawdd = 0.2;

	KalmanData kd;

	kd.setKalmanData(nx,na_g);

	// set example state
	kd.x << 5.7441,
         	1.3800,
			2.2049,
			0.5015,
			0.3528;

	// set example covariance matrix
	kd.P <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
			-0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
			0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
			-0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
			-0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

	//set example covariance matrix
	kd.Q = Eigen::MatrixXd(na_g,na_g);
	kd.Q.fill(0.0);
	kd.Q.diagonal()<<pow(std_a, 2),pow(std_yawdd, 2);

  /*******************************************************************************
   *  Calculate the sigma points                                                 *
   *******************************************************************************/
	// Create a kalman data instance
	KfUt kft;
	kft.setKalmanData(kd,false,false,NULL,NULL,NULL);
	kft.CalculateSigmaPoints();

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer
	Eigen::MatrixXd Xsig = Eigen::MatrixXd(ng,sigma);
	Xsig <<
 			5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,  5.7441,   5.7441 ,  5.7441,   5.7441,   5.7441,
 			  1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,  1.41434,  1.23194,    1.38,     1.38 ,    1.38,     1.38,     1.38,
 			2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,  2.12566,  2.16423, 2.11398,   2.2049 ,  2.2049,   2.2049,   2.2049,
 			0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,  0.55961, 0.371114,0.486077, 0.407773 ,  0.5015,   0.5015,   0.5015,
 			0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528, 0.405627, 0.243477,0.329261,  0.22143 ,0.286879,   0.3528,   0.3528,
 			     0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,       0,        0 ,       0, -0.34641,        0,
 			     0,        0,        0,        0,        0,        0,        0,  0.34641,        0,        0,       0,        0 ,       0,        0, -0.34641;



  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((Xsig -kft.m_sig).norm() < 0.01);

	return r;
}

/**
 * @brief Test case 7 CalculateSigmaPointsAugPred
 *        Test the prediction through model of sigma points using Augmented states in UT.
 *
 */
bool CalculateSigmaPointsAugPred(void)
{
  /*******************************************************************************
   *  Set to calcualte tranformation sigma points Inputs                        *
   *******************************************************************************/
	// Create a kalman data instance
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;

	//set sample time
	double dt = 0.1;

	KalmanData kd;
	kd.setKalmanData(nx,na_g);

	// Create a kalmanUT data instance
	KfUt kft;
	kft.m_numUTState = nx;
	kft.m_numAug = ng;
	kft.m_lambda = 3 - ng;
	kft.m_sigmaPoints = sigma;
    kft.m_Func = PredictionModel;
    kft.m_HelperFunc = NULL;
    kft.m_args = &dt;
	kft.m_sig = Eigen::MatrixXd(ng,sigma);
	kft.m_sig <<
			5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
			1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
			2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
			0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
			0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
				0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
				0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  /*******************************************************************************
   *  Calculate the sigma points                                                 *
   *******************************************************************************/
  	kft.PredictSigmaPoints();

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer
	Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(nx, sigma);
	Xsig_pred <<
			5.93553,  6.06251 , 5.92217 ,  5.9415  ,   5.92361 , 5.93516  , 5.93705 , 5.93553  , 5.80832  ,5.94481  ,5.92935  ,5.94553  ,5.93589  ,5.93401 , 5.93553  ,
			1.48939,  1.44673 , 1.66484 ,  1.49719 ,   1.508   , 1.49001  , 1.49022 , 1.48939  , 1.5308   ,1.31287  ,1.48182  ,1.46967  ,1.48876  ,1.48855 , 1.48939  ,
			2.2049 ,  2.28414 , 2.24557 ,  2.29582 ,   2.2049  , 2.2049   , 2.23954 , 2.2049   , 2.12566  ,2.16423  ,2.11398  ,2.2049   ,2.2049   ,2.17026 , 2.2049   ,
			0.53678,  0.473387, 0.678098,  0.554557,   0.643644, 0.543372 , 0.53678 , 0.538512 , 0.600173 ,0.395462 ,0.519003 ,0.429916 ,0.530188 ,0.53678 , 0.535048 ,
			0.3528 ,  0.299973, 0.462123,  0.376339,   0.48417 , 0.418721 , 0.3528  , 0.387441 , 0.405627 ,0.243477 ,0.329261 ,0.22143  ,0.286879 ,0.3528  , 0.318159 ;

  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((Xsig_pred -kft.m_sig_pred).norm() < 0.01);

	return r;
}

/**
 * @brief Test case 8 CalculateSigmaPointsMeanCovar
 *        Test the calculation of mean and covariance of predicted sigma points with Augmented states in UT.
 *
 */
bool CalculateSigmaPointsMeanCovar(void)
{
  /*******************************************************************************
   *  Set to calcualte tranformation sigma points Inputs                        *
   *******************************************************************************/
	// Create a kalman data instance
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;

	KalmanData kd;
	kd.setKalmanData(nx,na_g);

	//create example std::vector for predicted state mean.
	kd.x = Eigen::VectorXd(nx);
	kd.x.fill(0);

	//create example matrix for predicted state covariance.
	kd.P = Eigen::MatrixXd(nx,nx);
	kd.P.fill(0.0);

	// Create a kalmanUT data instance
	KfUt kft;
	kft.m_numUTState = nx;
	kft.m_numAug = ng;
	kft.m_lambda = 3 - ng;
	kft.m_sigmaPoints = sigma;
    kft.m_Func = PredictionModel;
    kft.m_HelperFunc = calc_covar;
	kft.m_sig_pred = Eigen::MatrixXd(nx,sigma);
  	kft.m_sig_pred <<
			5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
	
	///* Weights of sigma points
    kft.m_weights = Eigen::VectorXd(sigma);
    kft.m_weights.fill(1 / float(2 *(kft.m_lambda +kft.m_numAug)));
    kft.m_weights(0) = kft.m_lambda /float(kft.m_lambda + kft.m_numAug);

	///* Initialize mean and covar matrix
	kft.mean = Eigen::VectorXd(nx);
	kft.mean.fill(0);
	kft.covar= Eigen::MatrixXd(nx,nx);
	kft.covar.fill(0);

  /*******************************************************************************
   *  Calculate the sigma points                                                 *
   *******************************************************************************/
  	kft.CalculateMean();
	kd.x = kft.mean;
	kft.CalculateCovariance();
	kd.P = kft.covar;

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer

	//x-state
	Eigen::VectorXd x_corr = Eigen::VectorXd(nx);
	x_corr << 5.93637,1.49035,2.20528,0.536853,0.353577;

	//P-state
	Eigen::MatrixXd p_corr = Eigen::MatrixXd(nx,nx);
	std::vector<Eigen::MatrixXd> p_corr_list;

	p_corr << 0.00543425, -0.0024053  ,0.00341576 , -0.00348196 ,-0.00299378  ,
			 -0.0024053 ,  0.010845   ,0.0014923  , 0.00980182  , 0.00791091  ,
			  0.00341576,  0.0014923  ,0.00580129 , 0.000778632 , 0.000792973 ,
			 -0.00348196,  0.00980182 ,0.000778632, 0.0119238   , 0.0112491   ,
			 -0.00299378,  0.00791091 ,0.000792973, 0.0112491   , 0.0126972   ;
  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((x_corr - kd.x).norm() < 0.01);
	r = r && ((p_corr - kd.P).norm() < 0.01);

	return r;
}

/**
 * @brief Test case 9 CalculateSigmaPointsMeanCovar2
 *        Test the calculation of mean and covariance of predicted sigma points with Augmented states in UT.
 *
 */
bool CalculateSigmaPointsMeanCovar2(void)
{
  /*******************************************************************************
   *  Set to calcualte tranformation sigma points Inputs                        *
   *******************************************************************************/
	// Create a kalman data instance
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;

	KalmanData kd;
	kd.setKalmanData(nx,na_g);
	//set example covariance matrix
	kd.Q = Eigen::MatrixXd(na_g,na_g);
	kd.Q.fill(0.0);

	// Create a kalmanUT data instance
	KfUt kft;
	kft.setKalmanData(kd,false,false,PredictionModel,calc_covar,NULL);
	kft.m_sig_pred = Eigen::MatrixXd(nx,sigma);
  	kft.m_sig_pred <<
			5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

	///* Initialize mean and covar matrix
	kft.mean = Eigen::VectorXd(nx);
	kft.mean.fill(0);
	kft.covar= Eigen::MatrixXd(nx,nx);
	kft.covar.fill(0);

  /*******************************************************************************
   *  Calculate the sigma points                                                 *
   *******************************************************************************/
  	kft.CalculateMean();
	kd.x = kft.mean;
	kft.CalculateCovariance();
	kd.P = kft.covar;

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer

	//x-state
	Eigen::VectorXd x_corr = Eigen::VectorXd(nx);
	x_corr << 5.93637,1.49035,2.20528,0.536853,0.353577;

	//P-state
	Eigen::MatrixXd p_corr = Eigen::MatrixXd(nx,nx);
	std::vector<Eigen::MatrixXd> p_corr_list;

	p_corr << 0.00543425, -0.0024053  ,0.00341576 , -0.00348196 ,-0.00299378  ,
			 -0.0024053 ,  0.010845   ,0.0014923  , 0.00980182  , 0.00791091  ,
			  0.00341576,  0.0014923  ,0.00580129 , 0.000778632 , 0.000792973 ,
			 -0.00348196,  0.00980182 ,0.000778632, 0.0119238   , 0.0112491   ,
			 -0.00299378,  0.00791091 ,0.000792973, 0.0112491   , 0.0126972   ;
  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((x_corr - kd.x).norm() < 0.01);
	r = r && ((p_corr - kd.P).norm() < 0.01);

	return r;
}

/**
 * @brief Test case 10 CalculateSigmaPointsMeanCovar
 *        Test the calculation of mean and covariance of predicted sigma
 *        of measurements points with Augmented states in UT.
 *
 */
bool CalculateMeasSigmaPointsMeanCovar(void)
{
  /*******************************************************************************
   *  Set to calcualte tranformation sigma points Inputs                        *
   *******************************************************************************/
	// set kalman parameters
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;
	int nz = 3;
	int lambda = 3- ng;

	// Create a kalman data instance
	KalmanData kd;
	kd.setKalmanData(nx,na_g,nz);

	//set process noise covariance
	kd.Q = Eigen::MatrixXd(na_g,na_g);
	kd.Q.fill(0.0);

	// radar measurement noise standard deviation radius in m
	double std_radr = 0.3;

	// radar measurement noise standard deviation angle in rad
	double std_radphi = 0.0175;

	// radar measurement noise standard deviation radius change in m/s
	double std_radrd = 0.1;

	kd.R = Eigen::MatrixXd(nz,nz);
	kd.R.fill(0.0);
	kd.R.diagonal()<<pow(std_radr,2.0),pow(std_radphi,2.0),pow(std_radrd,2.0);

	// Create a kalmanUT data instance
	KfUt kft;
	kft.m_sig_pred = Eigen::MatrixXd(nx,sigma);
  	kft.m_sig_pred <<
			5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

	///* Weights of sigma points
    kft.m_weights = Eigen::VectorXd(sigma);
    kft.m_weights.fill(1 / float(2 *(lambda +ng)));
    kft.m_weights(0) = lambda /float(lambda + ng);

	kft.setKalmanData(kd,true,true,PredictionModelMeasurement,calc_covar_measurement,NULL);

	///* Initialize mean and covar matrix
	kft.mean = Eigen::VectorXd(nz);
	kft.mean.fill(0);
	kft.covar= Eigen::MatrixXd(nz,nz);
	kft.covar.fill(0);

  /*******************************************************************************
   *  Calculate the sigma points                                                 *
   *******************************************************************************/
    kft.PredictSigmaPoints();
    kft.CalculateMean();
	kft.CalculateCovariance();
	kft.covar = kft.covar + kd.R;

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer

	//z-state
	Eigen::VectorXd z_corr = Eigen::VectorXd(nz);
	z_corr << 6.12155, 0.245993, 2.10313;

	//S-Matrix
	Eigen::MatrixXd s_corr = Eigen::MatrixXd(nz,nz);

	s_corr << 0.0946171, -0.000139448, 0.00407016,
             -0.000139448, 0.000617548, -0.000770652,
              0.00407016, -0.000770652, 0.0180917;
			  
  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((z_corr - kft.mean).norm() < 0.01);
	r = r && ((s_corr - kft.covar).norm() < 0.01);

	return r;
}

/**
 * @brief Test case 11 CalculateMeanCovarUT1
 *        Test the compelete UT with mean and covariance calculations.
 *
 */
bool CalculateMeanCovarUT1(void)
{
  /**********************************************
   *  Set jacobia Inputs                        *
   **********************************************/
	// Create a kalman data instance
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;

	//set sample time
	double dt = 0.1;
	// Process noise standard deviation longitudinal acceleration in m/s^2
	double std_a = 0.2;

	// Process noise standard deviation yaw acceleration in rad/s^2
	double std_yawdd = 0.2;

	KalmanData kd;

	kd.setKalmanData(nx,na_g);

	// set example state
	kd.x << 5.7441,
         	1.3800,
			2.2049,
			0.5015,
			0.3528;

	// set example covariance matrix
	kd.P <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
			-0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
			0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
			-0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
			-0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

	//set example covariance matrix
	kd.Q = Eigen::MatrixXd(na_g,na_g);
	kd.Q.fill(0.0);
	kd.Q.diagonal()<<pow(std_a, 2),pow(std_yawdd, 2);

  /*******************************************************************************
   *  Calculate Mean & covar of UT                                              *
   *******************************************************************************/
	// Create a kalman data instance
	KfUt kft;
	kft.setKalmanData(kd,false,false,PredictionModel,calc_covar,&dt);
	kft.UT();

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer

	//x-state
	Eigen::VectorXd x_corr = Eigen::VectorXd(nx);
	x_corr << 5.93445, 1.48885, 2.2049, 0.53678, 0.3528;

	//P-state
	Eigen::MatrixXd p_corr = Eigen::MatrixXd(nx,nx);
	std::vector<Eigen::MatrixXd> p_corr_list;

	p_corr <<   0.0054808 , -0.00249899,  0.00340521 , -0.0035741  , -0.00309082  ,
			   -0.00249899,  0.0110551 ,  0.00151803 ,  0.00990779 ,  0.00806653  ,
				0.00340521,  0.00151803,  0.0057998  ,  0.000780142,  0.000800107 ,
			   -0.0035741 ,  0.00990779,  0.000780142,  0.0119239  ,  0.01125     ,
			   -0.00309082,  0.00806653,  0.000800107,  0.01125    ,  0.0127      ;
  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((x_corr - kft.mean).norm() < 0.01);
	r = r && ((p_corr - kft.covar).norm() < 0.01);
}

/**
 * @brief Test case 12 CalculateMeanCovarUT2
 *        Test the compelete UT with mean and covariance calculations.
 *
 */
bool CalculateMeanCovarUT2(void)
{
  /*******************************************************************************
   *  Set to calcualte tranformation sigma points Inputs                        *
   *******************************************************************************/
	// set kalman parameters
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;
	int nz = 3;
	int lambda = 3- ng;

	// Create a kalman data instance
	KalmanData kd;
	kd.setKalmanData(nx,na_g,nz);

	//set process noise covariance
	kd.Q = Eigen::MatrixXd(na_g,na_g);
	kd.Q.fill(0.0);

	// radar measurement noise standard deviation radius in m
	double std_radr = 0.3;

	// radar measurement noise standard deviation angle in rad
	double std_radphi = 0.0175;

	// radar measurement noise standard deviation radius change in m/s
	double std_radrd = 0.1;

	kd.R = Eigen::MatrixXd(nz,nz);
	kd.R.fill(0.0);
	kd.R.diagonal()<<pow(std_radr,2.0),pow(std_radphi,2.0),pow(std_radrd,2.0);

	// Create a kalmanUT data instance
	KfUt kft;
	kft.m_sig_pred = Eigen::MatrixXd(nx,sigma);
  	kft.m_sig_pred <<
			5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

	///* Weights of sigma points
    kft.m_weights = Eigen::VectorXd(sigma);
    kft.m_weights.fill(1 / float(2 *(lambda +ng)));
    kft.m_weights(0) = lambda /float(lambda + ng);

  /*******************************************************************************
   *  Calculate the sigma points                                                 *
   *******************************************************************************/
  	kft.setKalmanData(kd,true,true,PredictionModelMeasurement,calc_covar_measurement,NULL);
	kft.UT();
	kft.covar = kft.covar + kd.R;

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer

	//z-state
	Eigen::VectorXd z_corr = Eigen::VectorXd(nz);
	z_corr << 6.12155, 0.245993, 2.10313;

	//S-Matrix
	Eigen::MatrixXd s_corr = Eigen::MatrixXd(nz,nz);

	s_corr << 0.0946171, -0.000139448, 0.00407016,
             -0.000139448, 0.000617548, -0.000770652,
              0.00407016, -0.000770652, 0.0180917;
			  
  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((z_corr - kft.mean).norm() < 0.01);
	r = r && ((s_corr - kft.covar).norm() < 0.01);

	return r;
}

/**
 * @brief Test case 13 PredictionUT
 *        Test the compelete UT Prediction step.
 *
 */
bool PredictionUT(void)
{
  /**********************************************
   *  Set jacobia Inputs                        *
   **********************************************/
	// Create a kalman data instance
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;

	//set sample time
	double dt = 0.1;
	// Process noise standard deviation longitudinal acceleration in m/s^2
	double std_a = 0.2;

	// Process noise standard deviation yaw acceleration in rad/s^2
	double std_yawdd = 0.2;

	KalmanData kd;

	kd.setKalmanData(nx,na_g);

	// set example state
	kd.x << 5.7441,
         	1.3800,
			2.2049,
			0.5015,
			0.3528;

	// set example covariance matrix
	kd.P <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
			-0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
			0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
			-0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
			-0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

	//set example covariance matrix
	kd.Q = Eigen::MatrixXd(na_g,na_g);
	kd.Q.fill(0.0);
	kd.Q.diagonal()<<pow(std_a, 2),pow(std_yawdd, 2);

  /*******************************************************************************
   *  Calculate Mean & covar of UT                                              *
   *******************************************************************************/
	// Create a kalman data instance
	KfPredict *kfptrdUT = new KfPredictUT();
	KfUt *p_kfut = new KfUt();
	kfptrdUT ->predict(kd, p_kfut, PredictionModel, calc_covar, &dt);

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer

	//x-state
	Eigen::VectorXd x_corr = Eigen::VectorXd(nx);
	x_corr << 5.93445, 1.48885, 2.2049, 0.53678, 0.3528;

	//P-state
	Eigen::MatrixXd p_corr = Eigen::MatrixXd(nx,nx);
	std::vector<Eigen::MatrixXd> p_corr_list;

	p_corr <<   0.0054808 , -0.00249899,  0.00340521 , -0.0035741  , -0.00309082  ,
			   -0.00249899,  0.0110551 ,  0.00151803 ,  0.00990779 ,  0.00806653  ,
				0.00340521,  0.00151803,  0.0057998  ,  0.000780142,  0.000800107 ,
			   -0.0035741 ,  0.00990779,  0.000780142,  0.0119239  ,  0.01125     ,
			   -0.00309082,  0.00806653,  0.000800107,  0.01125    ,  0.0127      ;
  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((x_corr - kd.x).norm() < 0.01);
	r = r && ((p_corr - kd.P).norm() < 0.01);
}

/**
 * @brief Test case 14 PredictionUTKFLIB
 *        Test the compelete UT Prediction step using kflib.
 *
 */
bool PredictionUTKFLIB(void)
{
  /**********************************************
   *  Set jacobia Inputs                        *
   **********************************************/
	// Create a kalman data instance
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;

	//set sample time
	double dt = 0.1;
	// Process noise standard deviation longitudinal acceleration in m/s^2
	double std_a = 0.2;

	// Process noise standard deviation yaw acceleration in rad/s^2
	double std_yawdd = 0.2;

	KalmanData kd;

	kd.setKalmanData(nx,na_g);

	// set example state
	kd.x << 5.7441,
         	1.3800,
			2.2049,
			0.5015,
			0.3528;

	// set example covariance matrix
	kd.P <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
			-0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
			0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
			-0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
			-0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

	//set example covariance matrix
	kd.Q = Eigen::MatrixXd(na_g,na_g);
	kd.Q.fill(0.0);
	kd.Q.diagonal()<<pow(std_a, 2),pow(std_yawdd, 2);

  /*******************************************************************************
   *  Calculate Mean & covar of UT                                              *
   *******************************************************************************/
	///* instance of kalman helper
	KfLib kf_helper_;
	kf_helper_.setMode(new KfPredictUT(), new KfUpdateLinear(), new KfUt);
	kf_helper_.predict(kd, PredictionModel, calc_covar, &dt) ;

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer

	//x-state
	Eigen::VectorXd x_corr = Eigen::VectorXd(nx);
	x_corr << 5.93445, 1.48885, 2.2049, 0.53678, 0.3528;

	//P-state
	Eigen::MatrixXd p_corr = Eigen::MatrixXd(nx,nx);
	std::vector<Eigen::MatrixXd> p_corr_list;

	p_corr <<   0.0054808 , -0.00249899,  0.00340521 , -0.0035741  , -0.00309082  ,
			   -0.00249899,  0.0110551 ,  0.00151803 ,  0.00990779 ,  0.00806653  ,
				0.00340521,  0.00151803,  0.0057998  ,  0.000780142,  0.000800107 ,
			   -0.0035741 ,  0.00990779,  0.000780142,  0.0119239  ,  0.01125     ,
			   -0.00309082,  0.00806653,  0.000800107,  0.01125    ,  0.0127      ;
  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((x_corr - kd.x).norm() < 0.01);
	r = r && ((p_corr - kd.P).norm() < 0.01);
}

/**
 * @brief Test case 15 MeasPredictionUT
 *        Test the compelete UT Prediction step in measurement space using kflib.
 *
 */
bool MeasPredictionUT(void)
{
  /*******************************************************************************
   *  Set to calcualte tranformation sigma points Inputs                        *
   *******************************************************************************/
	// set kalman parameters
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;
	int nz = 3;
	int lambda = 3- ng;

	// Create a kalman data instance
	KalmanData kd;
	kd.setKalmanData(nx,na_g,nz);

	//set process noise covariance
	kd.Q = Eigen::MatrixXd(na_g,na_g);
	kd.Q.fill(0.0);

	// radar measurement noise standard deviation radius in m
	double std_radr = 0.3;

	// radar measurement noise standard deviation angle in rad
	double std_radphi = 0.0175;

	// radar measurement noise standard deviation radius change in m/s
	double std_radrd = 0.1;

	kd.R = Eigen::MatrixXd(nz,nz);
	kd.R.fill(0.0);
	kd.R.diagonal()<<pow(std_radr,2.0),pow(std_radphi,2.0),pow(std_radrd,2.0);

	// Create a kalmanUT data instance
	KfUt kft;
	kft.m_sig_pred = Eigen::MatrixXd(nx,sigma);
  	kft.m_sig_pred <<
			5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

	///* Weights of sigma points
    kft.m_weights = Eigen::VectorXd(sigma);
    kft.m_weights.fill(1 / float(2 *(lambda +ng)));
    kft.m_weights(0) = lambda /float(lambda + ng);

  /*******************************************************************************
   *  Calculate the sigma points                                                 *
   *******************************************************************************/
	// Create a kalman data instance
	KfUpdateUT *kfptrdUT = new KfUpdateUT();
	MeasurementPackage meas_package;
	meas_package.raw_measurements_ = Eigen::VectorXd(nz);
	meas_package.raw_measurements_ << 5.9214, 0.2187, 2.0062;   
	kfptrdUT ->update(kd, meas_package, &kft, PredictionModelMeasurement, calc_covar_measurement,NULL ,NULL);
	kft.covar = kft.covar + kd.R;

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer

	//z-state
	Eigen::VectorXd z_corr = Eigen::VectorXd(nz);
	z_corr << 6.12155, 0.245993, 2.10313;

	//S-Matrix
	Eigen::MatrixXd s_corr = Eigen::MatrixXd(nz,nz);

	s_corr << 0.0946171, -0.000139448, 0.00407016,
             -0.000139448, 0.000617548, -0.000770652,
              0.00407016, -0.000770652, 0.0180917;
			  
  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
  	bool r= true;
	r = r && ((z_corr - kft.mean).norm() < 0.01);
	r = r && ((s_corr - kft.covar).norm() < 0.01);

	return r;
}

/**
 * @brief Test case 16 PredictionMeasUTKFLIB
 *        Test the compelete UT Prediction step in measurement space using kflib.
 *
 */
bool PredictionMeasUTKFLIB(void)
{
  /*******************************************************************************
   *  Set to calcualte tranformation sigma points Inputs                        *
   *******************************************************************************/
	// set kalman parameters
	int nx = 5;
	int na_g = 2;
	int ng = nx + na_g;
	int sigma = 2*(ng)+1;
	int nz = 3;
	int lambda = 3- ng;

	// Create a kalman data instance
	KalmanData kd;
	kd.setKalmanData(nx,na_g,nz);

    // create example std::vector for predicted state mean
	kd.x <<5.93637,1.49035,2.20528,0.536853,0.353577;

	// create example matrix for predicted state covariance
	kd.P <<
		 0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
		-0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
		 0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
		-0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
		-0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

	//set process noise covariance
	kd.Q = Eigen::MatrixXd(na_g,na_g);
	kd.Q.fill(0.0);

	// radar measurement noise standard deviation radius in m
	double std_radr = 0.3;

	// radar measurement noise standard deviation angle in rad
	double std_radphi = 0.0175;

	// radar measurement noise standard deviation radius change in m/s
	double std_radrd = 0.1;

	kd.R = Eigen::MatrixXd(nz,nz);
	kd.R.fill(0.0);
	kd.R.diagonal()<<pow(std_radr,2.0),pow(std_radphi,2.0),pow(std_radrd,2.0);

	// Create a kalmanUT data instance
	KfUt kft;
	kft.m_sig_pred = Eigen::MatrixXd(nx,sigma);
  	kft.m_sig_pred <<
			5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

	///* Weights of sigma points
    kft.m_weights = Eigen::VectorXd(sigma);
    kft.m_weights.fill(1 / float(2 *(lambda +ng)));
    kft.m_weights(0) = lambda /float(lambda + ng);

  /*******************************************************************************
   *  Calculate the sigma points                                                 *
   *******************************************************************************/
	// Create a kalman data instance
		///* instance of kalman helper
	MeasurementPackage meas_package;
	meas_package.raw_measurements_ = Eigen::VectorXd(nz);
	meas_package.raw_measurements_ << 5.9214, 0.2187, 2.0062;   

	KfLib kf_helper_;
	kf_helper_.setMode(new KfPredictUT(), new KfUpdateUT(), &kft);
	kf_helper_.update(kd, meas_package, PredictionModelMeasurement, calc_covar_measurement, calc_covar,NULL);
	kft.covar = kft.covar + kd.R;

  /*******************************************************************************
   *  Set correct Answer                                                         *
   *******************************************************************************/
	//Correct Answer

	//z-state
	Eigen::VectorXd z_corr = Eigen::VectorXd(nz);
	z_corr << 6.12155, 0.245993, 2.10313;

	//S-Matrix
	Eigen::MatrixXd s_corr = Eigen::MatrixXd(nz,nz);

	s_corr << 0.0946171, -0.000139448, 0.00407016,
             -0.000139448, 0.000617548, -0.000770652,
              0.00407016, -0.000770652, 0.0180917;

	//x-state
	Eigen::VectorXd x_corr = Eigen::VectorXd(nx);
	x_corr  << 5.92276,1.41823,2.15593,0.489274,	0.321338;

	//P-Matrix
	Eigen::MatrixXd p_corr = Eigen::MatrixXd(nx,nx);
	p_corr <<
		0.00361579  ,-0.000357881,  0.00208316,  -0.000937196,  0.00071727,
	   -0.000357881 , 0.00539867 ,  0.00156846,   0.00455342 ,  0.00358885,
		0.00208316  , 0.00156846 ,  0.00410651,   0.00160333 ,  0.00171811,
	   -0.000937196 , 0.00455342 ,  0.00160333,   0.00652634 ,  0.00669436,
	   -0.00071719  , 0.00358884 ,  0.00171811,   0.00669426 ,  0.00881797;

  /*******************************************************************************
   *  Evaluation                                                    *
   *******************************************************************************/
	bool r= true;
	r = r && ((z_corr - kft.mean).norm() < 0.01);
	r = r && abs(((s_corr - kft.covar).norm()) < 0.01);
	r = r && ((x_corr - kd.x).norm() < 0.01);
	r = r && ((p_corr - kd.P).norm() < 0.01);

	return r;
}


Eigen::VectorXd PredictionModel (const Eigen::VectorXd &sig_pred_ , const void *args)
{
	Eigen::VectorXd Xsig_pred = Eigen::VectorXd(5);
	Xsig_pred.fill(0.0);
	double dt = *(double*)args;
	// Extract values for readability
	double p_x      = sig_pred_(0);
	double p_y      = sig_pred_(1);
	double v        = sig_pred_(2);
	double yaw      = sig_pred_(3);
	double yawd     = sig_pred_(4);
	double nu_a     = sig_pred_(5);
	double nu_yawdd = sig_pred_(6);

	// predicted state values
	double px_p, py_p;

	// avoid divison ny zero
	if (fabs(yawd) > 0.001)
	{
		// General equations
		px_p = p_x + v / yawd * ( sin(yaw + yawd * dt) - sin(yaw));
		py_p = p_y + v / yawd * (-cos(yaw + yawd * dt) + cos(yaw));
	}
	else
	{
		// Special case
		px_p = p_x + v * dt * cos(yaw);
		py_p = p_y + v * dt * sin(yaw);
	}

	double v_p = v;
	double yaw_p = yaw + yawd * dt;
	double yawd_p = yawd;

	// add noise
	double dt2 = dt * dt;
	px_p = px_p + 0.5 * nu_a * dt2 * cos(yaw);
	py_p = py_p + 0.5 * nu_a * dt2 * sin(yaw);
	v_p = v_p + nu_a * dt;

	yaw_p  = yaw_p + 0.5 * nu_yawdd * dt2;
	yawd_p = yawd_p + nu_yawdd * dt;

	// write predicted sigma point into right column
	Xsig_pred(0) = px_p;
	Xsig_pred(1) = py_p;
	Xsig_pred(2) = v_p;
	Xsig_pred(3) = yaw_p;
	Xsig_pred(4) = yawd_p;

	return Xsig_pred;
}

/**
 * \brief brief PredictionModelMeasurement, Propagates the sigma points into Measurement model.
 *
 * \param[in]  sig_pred_
 *  The Sigma points std::vector {Eigen::VectorXd&}.
 *
 * \param[in]  dt
 *  The difference time stamp {double}.
 *
 *  \return  x_diff {Eigen::VectorXd}.
 */
Eigen::VectorXd PredictionModelMeasurement (const Eigen::VectorXd &sig_pred_ , const void *args)
{
	Eigen::VectorXd Xsig_pred = Eigen::VectorXd(3);
	Xsig_pred.fill(0.0);
	// extract values for better readability
	double p_x = sig_pred_(0);
	double p_y = sig_pred_(1);
	double v   = sig_pred_(2);
	double yaw = sig_pred_(3);

	double vx = v * cos(yaw);
	double vy = v * sin(yaw);

    // Avoid division by zero
    if(fabs(p_x) <= 0.0001)
	{
        p_x = 0.0001;
    }

    if(fabs(p_y) <= 0.0001)
	{
        p_y = 0.0001;
    }

	// measurement model
	double p_x2 = p_x * p_x;
	double p_y2 = p_y * p_y;

	double r     = sqrt(p_x2 + p_y2);
	double phi   = atan2(p_y, p_x);
	double r_dot =  (p_x * vx + p_y * vy) / r;


	// write predicted sigma point into right column
	Xsig_pred(0) = r;
	Xsig_pred(1) = phi;
	Xsig_pred(2) = r_dot;


	return Xsig_pred;
}

/**
 * \brief calc_covar,Helper function fix rounding issues in calculating covariance.
 *
 * \param[in]  sig_pred_
 *  The Sigma points std::vector {Eigen::VectorXd&}.
 *
 * \param[in]  dt
 *  The difference time stamp {double}.
 *
 *  \return  x_diff {Eigen::VectorXd}.
 */
Eigen::VectorXd calc_covar (const Eigen::VectorXd &sig_pred , const void *args)
{
	Eigen::VectorXd x_diff;
	x_diff = sig_pred;
	// angle normalization
	while (x_diff(3) > M_PI)
		x_diff(3) -= 2.0 * M_PI;
	while (x_diff(3) < -M_PI)
		x_diff(3) += 2.0 * M_PI;
	return x_diff;
}

/**
 * \brief calc_covar Helper function fix rounding issues in calculating covariance.
 *
 * \param[in]  sig_pred the Sigma points std::vector {Eigen::VectorXd&}.
 *
 * \param[in]  dt the difference time stamp {double}.
 *
 *  \return x_diff {Eigen::VectorXd} .
 */
Eigen::VectorXd calc_covar_measurement (const Eigen::VectorXd &sig_pred , const void *args)
{
	Eigen::VectorXd x_diff;
	x_diff = sig_pred;
	//angle normalization
	while (x_diff(1) > M_PI)
		x_diff(1) -= 2.0 * M_PI;
	while (x_diff(1) < -M_PI)
		x_diff(1) += 2.0 * M_PI;
	return x_diff;
}
/**
 *  @}
 */
