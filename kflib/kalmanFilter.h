/*
 * kf_lib.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */
/** @file kf_lib.h
 *  @ingroup kf
 *  @brief kf class
 */

/**
 *  @addtogroup kf
 *  @{
 */

#ifndef KALMAN_FILER_H_
#define KALMAN_FILER_H_
#include <Eigen/Dense>
#include <vector>
#include "KalmanConfig.h"
#include"kalman_data.h"

class kalmanFilter
{
public:
    /**
     * @brief predict
     * Perform the Prediction step in kalman filter.
     *
     * @param[in,out] kd 
     * is kalman data object  {KalmanData}.
     * 
     * @param[in] Q 
     * the process noise matrix  {MatrixXd}.
     * 
     * @param[in] g 
     * calculates the mean state vector based dynamic model{function}.
     * 
     * @param[in] g_prime 
     * calculates the mean state vector based dynamic model{function}.
     *
     */
    static void predict(KalmanData &kd,
                        const Eigen::MatrixXd &Q,
                        std::function<Eigen::VectorXd(KalmanData &)>g,
                        std::function<Eigen::MatrixXd(KalmanData &)>g_prime)
    {
        kd.x = g(kd);
        Eigen::MatrixXd G = g_prime(kd);
        kd.P = (G* kd.P * G.transpose()) + Q;
    }

    /**
     * @brief CalculateKalmanGain
     * It calculates kalman gain.
     *
     * @param[in,out] kd 
     * is kalman data object  {KalmanData}.
     * 
     * @param[in] H 
     * the measurement matrix  {MatrixXd}.
     * 
     * @param[in] R 
     * the noise covariance measurement matrix  {MatrixXd}.
     *
     */
    static Eigen::MatrixXd CalculateKalmanGain(KalmanData &kd,
                                            const Eigen::MatrixXd &H,
                                            const Eigen::MatrixXd &R)
    {
        Eigen::MatrixXd Ht = H.transpose();
        Eigen::MatrixXd S = (H * kd.P * Ht) + R;
        Eigen::MatrixXd K = kd.P *Ht * S.inverse();
        return K;
    }

    /**
     * @brief update
     * Perform the update step.
     *
     * @param[in,out] kd 
     * is kalman data object  {KalmanData}.
     * 
     * @param[in] Y 
     * the innovation vector  {VectorXd}.
     * 
     * @param[in] H 
     * the measurement matrix  {MatrixXd}.
     * 
     * @param[in] k 
     * the kalman gain matrix  {MatrixXd}.
     *
     */
    static void update(KalmanData &kd,
                       const Eigen::VectorXd &Y,
                       const Eigen::MatrixXd &H,
                       const Eigen::MatrixXd &K)
    {
        kd.x = kd.x + (K * Y);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(kd.x.size(), kd.x.size());
        kd.P = (I - K * H) * kd.P;
    }

};

#endif /* KALMAN_FILER_H_ */
/**
 *  @}
 */
