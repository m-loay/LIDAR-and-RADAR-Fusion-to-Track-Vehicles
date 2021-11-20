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
#include "kalmanFilter.h"
#include "UT.h"

Eigen::VectorXd PredictionModel (const Eigen::Ref<const Eigen::VectorXd> &,const void *p=NULL);
Eigen::VectorXd calc_covar (const Eigen::Ref<const Eigen::VectorXd>&);
Eigen::VectorXd PredictionModelMeasurement (const Eigen::Ref<const Eigen::VectorXd>&, const void* p = NULL);
Eigen::VectorXd calc_covar_measurement (const Eigen::Ref<const Eigen::VectorXd>&);
Eigen::VectorXd g_t1 (const Eigen::Ref<const Eigen::VectorXd>& mean, const void *p_args=NULL);
Eigen::MatrixXd g_t1_prime (const Eigen::Ref<const Eigen::VectorXd>& mean, const void *p_args=NULL);

//Test cases


/**
 * @brief 
 * 
 * @return true 
 * @return false 
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

    //create example std::vector for predicted state mean.
    Eigen::VectorXd x = Eigen::VectorXd(n_x);
    x << 0, 0;

    //create example matrix for predicted state covariance.
    Eigen::MatrixXd P = Eigen::MatrixXd(n_x,n_x);
    P.fill(0.0);
    P << 1000, 0, 0, 1000;

    //output matrix
    Eigen::MatrixXd H = Eigen::MatrixXd (n_z, n_x);
    H<< 1, 0;

    //Sensor Noise Covariance matrix
    Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
    R.fill(0.0);
    R.diagonal()<<1.0;

    //precoess noise covariance matrix Q
    Eigen::MatrixXd Q = Eigen::MatrixXd (n_x, n_x);
    Q<< 0.0, 0.0, 0.0, 0.0;

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
    for (unsigned int n = 0; n < measurement_pack_list.size(); ++n)
    {
        // Calculate Innovation
        Eigen::VectorXd zpred = H * x;
        Eigen::VectorXd z_meas = measurement_pack_list[n].raw_measurements_;
        Eigen::VectorXd Y = z_meas - zpred;

        //Calculate Kalman Gain
        Eigen::MatrixXd K = kalmanFilter::CalculateKalmanGain(P, H, R);

        //perform Update step
        kalmanFilter::update(x, P, Y, H, K);

        //perform Predict step
        kalmanFilter::predict(x, P, Q, g_t1, g_t1_prime);

        // //collect result
        x_rest_list.push_back(x);
        p_rest_list.push_back(P);
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
    Eigen::MatrixXd H = tracking.h_prime_(tracking.kd_.x);

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
    r = r && ((H -Hj_corr).norm() < 0.01);

    return r;
}

/**
 * @brief Test case 4 trackEKF
 *        Test both linear/NoneLinear update step and linear prediction step .
 *
 */
bool trackEKF(void)
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
    while (getline(in_file, line)&& (i<=5))
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
            // read measurements at this 
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
    x_corr << 0.722628,0.567796,3.70663,-0.56481;
    x_corr_list.push_back(x_corr);
    x_corr << 0.969665,0.413432,5.6934,-2.08598;
    x_corr_list.push_back(x_corr);
    x_corr << 0.984782,0.681457, 2.3165,0.760458;
    x_corr_list.push_back(x_corr);
    x_corr <<  1.03169,0.698205,2.11118,0.916794;
    x_corr_list.push_back(x_corr);
    x_corr << 1.21554,0.670913,2.35642,0.325553;
    x_corr_list.push_back(x_corr);

    //P-state
    Eigen::MatrixXd p_corr = Eigen::MatrixXd(4,4);
    std::vector<Eigen::MatrixXd> p_corr_list;

    p_corr <<   0.0744763,   0.0957463,   0.0140901,  -0.0088403,
                0.0957463,    0.127007, -0.00884025,  0.00923985,
                0.0140901, -0.00884025,     180.933,    -137.793,
                -0.0088403,  0.00923985,    -137.793,     105.334;
    p_corr_list.push_back(p_corr);

    p_corr <<    0.0212348, -0.000763264,     0.275495,    -0.208923,
                -0.000763264,     0.020816,    -0.208923,      0.16087,
                0.275495,    -0.208923,      5.94417,      -4.3339,
                -0.208923,      0.16087,      -4.3339,      3.56638;
    p_corr_list.push_back(p_corr);

    p_corr <<     0.012367,   0.00418933,    0.0424686,   -0.0499424,
                0.00418933,   0.00439293,   0.00839503, -0.000486848,
                0.0424686,   0.00839503,     0.265165  ,   -0.19538,
                -0.0499424, -0.000486848,     -0.19538,     0.490509;
    p_corr_list.push_back(p_corr);

    p_corr <<    0.00974513, 0.000737499,   0.0318128,  -0.0346475,
                0.000737499,  0.00442744, -0.00294044,   0.0215166,
                0.0318128, -0.00294044,    0.198251,   -0.107771,
                -0.0346475,   0.0215166,   -0.107771,    0.387774;
    p_corr_list.push_back(p_corr);

    p_corr << 0.00769929, 0.00194051,  0.0192605, -0.0125547,
                0.00194051, 0.00382965, 0.00150358,   0.011961,
                0.0192605, 0.00150358,   0.109024, -0.0288252,
                -0.0125547,   0.011961, -0.0288252,   0.165914;
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
 * @brief Test case 5 calculateRMSE
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
 * @brief Test case 6 CalculateSigmaPointsNoAugmentation
 *        Test the calculation of sigma points in UT.
 *
 */
bool CalculateSigmaPointsNoAugmentation(void)
{
    /**********************************************
     *  Initialization                         *
     **********************************************/
    //number of states
    int nx = 5;

    // set example state
    Eigen::VectorXd mean = Eigen::VectorXd(nx);
    mean << 5.7441,
                1.3800,
            2.2049,
            0.5015,
            0.3528;

    // set example covariance matrix
    Eigen::MatrixXd covariance = Eigen::MatrixXd(nx,nx);
    covariance <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
            -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
            0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
            -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
            -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    /*******************************************************************************
     *  Calculate the sigma points                                                 *
     *******************************************************************************/
    Eigen::MatrixXd sigmaPoints = UT::CalculateSigmaPoints(mean, covariance);

    /*******************************************************************************
     *  Set correct Answer                                                         *
     *******************************************************************************/
    //Correct Answer
    Eigen::MatrixXd Xsig = Eigen::MatrixXd(sigmaPoints.rows(),sigmaPoints.cols());
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
    r = r && ((Xsig - sigmaPoints).norm() < 0.01);

    return r;
}

/**
 * @brief Test case 7 CalculateSigmaPointsWithAugmentation
 *        Test the calculation of sigma points in UT.
 *
 */
bool CalculateSigmaPointsWithAugmentation(void)
{
    /**********************************************
     *  Set jacobia Inputs                        *
     **********************************************/
    //number of states
    int nx = 5;
    int na_aug = 2;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a = 0.2;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd = 0.2;

    //create process noise covariance matrix
    Eigen::MatrixXd Q = Eigen::MatrixXd(na_aug, na_aug);
    Q.fill(0.0);
    Q.diagonal()<<pow(std_a, 2),pow(std_yawdd, 2);

    // set example state
    Eigen::VectorXd mean = Eigen::VectorXd(nx);
    mean << 5.7441,
                1.3800,
            2.2049,
            0.5015,
            0.3528;

    // set example covariance matrix
    Eigen::MatrixXd covariance = Eigen::MatrixXd(nx,nx);
    covariance <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
            -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
            0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
            -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
            -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    /*******************************************************************************
     *  Calculate the sigma points                                                 *
     *******************************************************************************/
    // Create a kalman data instance
    Eigen::MatrixXd sigmaPoints = UT::CalculateSigmaPoints(mean, covariance, Q);

    /*******************************************************************************
     *  Set correct Answer                                                         *
     *******************************************************************************/
    //Correct Answer
    Eigen::MatrixXd Xsig = Eigen::MatrixXd(sigmaPoints.rows(),sigmaPoints.cols());
    Xsig <<
                5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441 ,  5.7441 ,  5.7441,
                1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38 ,    1.38 ,    1.38,
                2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049 ,  2.2049 ,  2.2049,
                0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015 ,  0.5015 ,  0.5015,
                0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879 ,  0.3528 ,  0.3528,
                    0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0,        0 ,-0.34641 ,       0,
                    0,        0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0 ,       0 ,-0.34641;


    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    bool r= true;
    r = r && ((Xsig - sigmaPoints).norm() < 0.01);

    return r;
}

/**
 * @brief Test case 8 CalculateSigmaPointsAugPred
 *        Test the prediction through model of sigma points using Augmented states in UT.
 *
 */
bool CalculateSigmaPointsAugPred(void)
{
    /*******************************************************************************
     *  Set to calcualte tranformation sigma points Inputs                        *
     *******************************************************************************/
    //number of states
    int nx = 5;
    int na_aug = 2;
    int numAugState = nx + na_aug;
    int numSigmaPoints = 2*numAugState +1;

    //set sample time
    double dt = 0.1;

    // Create a sigma points
    Eigen::MatrixXd sig = Eigen::MatrixXd(numAugState, numSigmaPoints);
    sig <<
            5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
            1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
            2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
            0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
            0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
                0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
                0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

    /*******************************************************************************
     *  Predict sigma points                                                 *
     *******************************************************************************/
    Eigen::MatrixXd pred_sig = UT::PredictSigmaPoints(sig, PredictionModel, &dt, na_aug);

    /*******************************************************************************
     *  Set correct Answer                                                         *
     *******************************************************************************/
    //Correct Answer
    Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(nx,numSigmaPoints);
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
    r = r && ((Xsig_pred - pred_sig).norm() < 0.01);

    return r;
}

/**
 * @brief Test case 9 CalculateSigmaPointsMeanCovar
 *        Test the calculation of mean and covariance of predicted sigma points with Augmented states in UT.
 *
 */
bool CalculateSigmaPointsMeanCovar(void)
{
    /*******************************************************************************
     *  Set to calcualte tranformation sigma points Inputs                        *
     *******************************************************************************/
    //number of states
    int nx = 5;
    int na_aug = 2;
    int numAugState = nx + na_aug;
    int numSigmaPoints = 2*numAugState +1;


    // Create a kalmanUT data instance
    Eigen::MatrixXd sig_pred = Eigen::MatrixXd(nx,numSigmaPoints);
    sig_pred <<
            5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    /*******************************************************************************
     *  Calculate Mean and Covriance                                                 *
     *******************************************************************************/
    //calculate the weights
    Eigen::VectorXd weights = UT::CalculateWeigts(numSigmaPoints, numAugState);

    // Predict Sigma Points
    Eigen::VectorXd mean = UT::PredictMean(sig_pred, weights);

    // Calculate Covariance Sigma Points
    Eigen::MatrixXd covariance = UT::PredictCovariance(mean, sig_pred, weights, calc_covar);

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
    r = r && ((x_corr - mean).norm() < 0.01);
    r = r && ((p_corr - covariance).norm() < 0.01);

    return r;
}

/**
 * @brief Test case 10 PredictUT
 *        Test the compelete UT with mean and covariance calculations.
 *
 */
bool PredictUT(void)
{
    /**********************************************
     *  Set Kalman Data                       *
     **********************************************/
    //number of states
    int nx = 5;
    int na_aug = 2;
    int numAugState = nx + na_aug;
    int numSigmaPoints = 2*numAugState +1;
    double dt = 0.1;

    // set example state
    Eigen::VectorXd mean = Eigen::VectorXd(nx);
    mean << 5.7441,
                1.3800,
            2.2049,
            0.5015,
            0.3528;

    // set example covariance matrix
    Eigen::MatrixXd covariance = Eigen::MatrixXd(nx,nx);
    covariance <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
            -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
            0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
            -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
            -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    //precoess noise covariance matrix Q
    Eigen::MatrixXd Q = Eigen::MatrixXd (na_aug, na_aug);
    Q<< 0.0, 0.0, 0.0, 0.0;

    /*******************************************************************************
     *  Calculate Mean & covar of UT                                              *
     *******************************************************************************/
    // Calculate Sigma Points
    Eigen::MatrixXd sigmaPoints = UT::CalculateSigmaPoints(mean, covariance, Q);

    // Predict Sigma Points
    Eigen::MatrixXd pred_sig = UT::PredictSigmaPoints(sigmaPoints, PredictionModel, &dt, na_aug);

    //calculate the weights
    Eigen::VectorXd weights = UT::CalculateWeigts(numSigmaPoints, numAugState);

    // Calculate mean of Sigma Points
    mean = UT::PredictMean(pred_sig, weights);

    // Calculate Covariance Sigma Points
    covariance = UT::PredictCovariance(mean, pred_sig, weights, calc_covar);

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

    p_corr << 0.00543425, -0.0024053  ,0.00341576 , -0.00348196 ,-0.00299378  ,
                -0.0024053 ,  0.010845   ,0.0014923  , 0.00980182  , 0.00791091  ,
                0.00341576,  0.0014923  ,0.00580129 , 0.000778632 , 0.000792973 ,
                -0.00348196,  0.00980182 ,0.000778632, 0.0119238   , 0.0112491   ,
                -0.00299378,  0.00791091 ,0.000792973, 0.0112491   , 0.0126972   ;
    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    bool r= true;
    r = r && ((x_corr - mean).norm() < 0.01);
    r = r && ((p_corr - covariance).norm() < 0.01);
    return r;
}

/**
 * @brief Test case 11 UpdateUT
 *        Test the compelete UT update with mean and covariance calculations.
 *
 */
bool CalculateMeasurementsMeanCovar(void)
{
    /*******************************************************************************
     *  Set kalman data Inputs                        *
     *******************************************************************************/
    // set kalman parameters
    int nx = 5;
    int na_g = 2;
    int nz = 3;
    int numAugState = nx + na_g;
    int numSigmaPoints = 2*numAugState +1;

    // radar measurement noise standard deviation radius in m
    double std_radr = 0.3;

    // radar measurement noise standard deviation angle in rad
    double std_radphi = 0.0175;

    // radar measurement noise standard deviation radius change in m/s
    double std_radrd = 0.1;

    Eigen::MatrixXd R = Eigen::MatrixXd(nz,nz);
    R.fill(0.0);
    R.diagonal()<<pow(std_radr,2.0),pow(std_radphi,2.0),pow(std_radrd,2.0);

    // Create a kalmanUT data instance
    Eigen::MatrixXd sig_pred = Eigen::MatrixXd(nx,numSigmaPoints);
    sig_pred <<
            5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    /*******************************************************************************
     *  Calculate Measurement covraiance                                                 *
     *******************************************************************************/
    //Transform predicted sigma points from state space to measurement space
    Eigen::MatrixXd tSig_pred = UT::TransformPredictedSigmaToMeasurement(sig_pred, nz, PredictionModelMeasurement);

    //calculate the weights
    Eigen::VectorXd weights = UT::CalculateWeigts(numSigmaPoints, numAugState);

    // Calculate mean of Sigma Points
    Eigen::VectorXd zpred = UT::PredictMean(tSig_pred, weights);

    // Calculate Covariance Sigma Points
    Eigen::MatrixXd S = UT::PredictCovariance(zpred, tSig_pred, weights, calc_covar_measurement);
    S += R;

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
    r = r && ((z_corr - zpred).norm() < 0.01);
    r = r && ((s_corr - S).norm() < 0.01);

    return r;
}

/**
 * @brief Test case 12 UpdateUT
 *        Test the compelete UT update with mean and covariance calculations.
 *
 */
bool UpdateUT(void)
{
    /*******************************************************************************
     *  Set kalman data Inputs                        *
     *******************************************************************************/
    // set kalman parameters
    int nx = 5;
    int na_g = 2;
    int nz = 3;
    int numAugState = nx + na_g;
    int numSigmaPoints = 2*numAugState +1;

    // create example vector for predicted state mean
    Eigen::VectorXd x = Eigen::VectorXd(nx);
    x <<5.93637,1.49035,2.20528,0.536853,0.353577;

    // create example matrix for predicted state covariance
    Eigen::MatrixXd P = Eigen::MatrixXd(nx,nx);
    P <<
    0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
    -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
    0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
    -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
    -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

    // create example matrix with predicted sigma points in state space
    Eigen::MatrixXd sig_pred = Eigen::MatrixXd(nx,numSigmaPoints);
    sig_pred <<
            5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    // create measurement Vector
    Eigen::VectorXd z = Eigen::VectorXd(nz);
    z << 5.9214, 0.2187, 2.0062;

    // create example vector for mean predicted measurement
    Eigen::VectorXd z_pred = Eigen::VectorXd(nz);
    z_pred <<6.12155,0.245993,2.10313;

    // create example matrix for predicted measurement covariance
    Eigen::MatrixXd S = Eigen::MatrixXd(nz,nz);
    S <<
        0.0946171, -0.000139448,   0.00407016,
        -0.000139448,  0.000617548, -0.000770652,
        0.00407016, -0.000770652,    0.0180917;

    // create example matrix with sigma points in measurement space
    Eigen::MatrixXd Zsig = Eigen::MatrixXd(nz, numSigmaPoints);
    Zsig <<
    6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
    0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
    2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;

    /*******************************************************************************
     * update cycle for measumnt                                                 *
     *******************************************************************************/
    //calculate the weights
    Eigen::VectorXd weights = UT::CalculateWeigts(numSigmaPoints, numAugState);

    //calculate the kalman Gain
    Eigen::MatrixXd K = UT::CalculateKalmanGainUT(x, z_pred, weights, sig_pred, Zsig, S, calc_covar, calc_covar_measurement);

    //update
    Eigen::VectorXd Y = z - z_pred;
    UT::updateUT(x,P,Y,S,K);

    /*******************************************************************************
     *  Set correct Answer                                                         *
     *******************************************************************************/
    //Correct Answer

    //x-state
    Eigen::VectorXd x_corr = Eigen::VectorXd(nx);
    x_corr << 5.92276, 1.41823, 2.15593, 0.489274,  0.321338;

    //P-Matrix
    Eigen::MatrixXd p_corr = Eigen::MatrixXd(nx,nx);

    p_corr << 0.00361579 ,-0.000357881 ,  0.00208316 ,-0.000937196 , -0.00071727,
                -0.000357881,   0.00539867,   0.00156846,   0.00455342,   0.00358885,
                0.00208316,   0.00156846,   0.00410651,   0.00160333,   0.00171811,
                -0.000937196,   0.00455342,   0.00160333,   0.00652634,   0.00669436,
                -0.00071719,   0.00358884,   0.00171811,   0.00669426,   0.00881797;
                
    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    bool r= true;
    r = r && ((x_corr - x).norm() < 0.01);
    r = r && ((p_corr - P).norm() < 0.01);

    return r;
}

/**
 * @brief 
 * 
 * @param col 
 * @param p_args 
 * @return Eigen::VectorXd 
 */
Eigen::VectorXd PredictionModel (const Eigen::Ref<const Eigen::VectorXd> &col,const void *pArgs)
{
    Eigen::VectorXd Xsig_pred = Eigen::VectorXd(5);
    Xsig_pred.fill(0.0);
    double dt = *(double*)pArgs;
    // Extract values for readability
    double p_x      =col(0);
    double p_y      =col(1);
    double v        =col(2);
    double yaw      =col(3);
    double yawd     =col(4);
    double nu_a     =col(5);
    double nu_yawdd =col(6);

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
 * @brief PredictionModelMeasurement used to propagate the sigma points throgh the measurement model.
 * 
 * @param[in] sig_pred_
 *  The sigma points calculated by the unscented transform {VectorXd} .
 * 
 * @param[in] p_args
 *  Extra arguments {const void *}.
 * 
 * @return Xsig_pred 
 * The propagated sigma points{VectorXd}.
 *
 */
Eigen::VectorXd PredictionModelMeasurement (const Eigen::Ref<const Eigen::VectorXd>&sig_pred_ , const void* p_args)
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
 * @brief calc_covar Helper function fix rounding issues in calculating covariance in prediction step.
 * 
 * @param[in] sig_pred_
 *  The state vector during the calculation of covarince {VectorXd} .
 * 
 * @return x_diff
 *  The state vector after applying the proper rounding to angles{VectorXd}.
 *
 */
Eigen::VectorXd calc_covar (const Eigen::Ref<const Eigen::VectorXd> &sig_pred)
{
    Eigen::VectorXd x_diff;
    x_diff = sig_pred;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    return x_diff;
}

/**
 * @brief calc_covar_measurement Helper function fix rounding issues in calculating covariance in update step.
 * 
 * @param[in] sig_pred_
 *  The state vector during the calculation of covarince {VectorXd} .
 * 
 * @return x_diff 
 * The state vector after applying the proper rounding to angles{VectorXd}.
 *
 */
Eigen::VectorXd calc_covar_measurement (const Eigen::Ref<const Eigen::VectorXd> &sig_pred)
{
    Eigen::VectorXd x_diff;
    x_diff = sig_pred;
    //angle normalization
    while (x_diff(1) > M_PI) x_diff(1) -= 2.0 * M_PI;
    while (x_diff(1) < -M_PI) x_diff(1) += 2.0 * M_PI;
    return x_diff;
}

/**
 * @brief g_t1 Function 
 *  which calculates the mean state vector based dynamic model.
 *
 * @param[in] mean
 *  the state vector {VectorXd&}.
 * 
 * @param[in] p_args
 *  Extra arguments {const void *}.
 * 
 * @return F.x
 *  the mean state vector {{VectorXd}}.
 */
Eigen::VectorXd g_t1 (const Eigen::Ref<const Eigen::VectorXd>& mean, const void *p_args)
{
    //create state transition matrix for predicted state covariance.
    Eigen::MatrixXd F = Eigen::MatrixXd(2,2);
    F<< 1, 1, 0, 1;
    return F* mean;

}

/**
 * @brief g_prime the derivative of g_function.
 *  In linear case it shall return the state transition Matrix.
 *  In non-linear it shall return the jacobians. 
 *
 * @param[in] mean
 *  the state vector {VectorXd&}.
 * 
 * @param[in] p_args
 *  Extra arguments {const void *}.
 * 
 * @return F 
 *  the state transition matrix {MatrixXd}.
 */
Eigen::MatrixXd g_t1_prime (const Eigen::Ref<const Eigen::VectorXd>& mean, const void *p_args)
{
    //create state transition matrix for predicted state covariance.
    Eigen::MatrixXd F = Eigen::MatrixXd(2,2);
    F<< 1, 1, 0, 1;
    return F;
}
/**
 *  @}
 */
