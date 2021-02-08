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
     * @param[in,out] x 
     * The mean of stat vector  {VectorXd &}.
     * 
     * @param[in,out] P
     * The mean of stat vector  {VectorXd &}.
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
     * @param[in] p_args
     *  Extra arguments {const void *}.
     *
     */
    static void predict(Eigen::VectorXd & x,
                        Eigen::MatrixXd & P,
                        const Eigen::Ref<const Eigen::MatrixXd> &Q,
                        std::function<Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd> &, const void *)>g,
                        std::function<Eigen::MatrixXd(const Eigen::Ref<const Eigen::VectorXd> &, const void *)>g_prime,
                        const void *p_args = NULL)
    {
        
        x = g(x, p_args);
        Eigen::MatrixXd G = g_prime(x, p_args);
        P = (G* P * G.transpose()) + Q;
    }

    /**
     * @brief CalculateKalmanGain
     * It calculates kalman gain.
     *
     * @param[in] P
     * The mean of stat vector  {VectorXd &}.
     * 
     * @param[in] H 
     * the measurement matrix  {MatrixXd}.
     * 
     * @param[in] R 
     * the noise covariance measurement matrix  {MatrixXd}.
     * 
     * @return K 
     * the kalman gain matrix  {MatrixXd}.
     *
     */
    static Eigen::MatrixXd CalculateKalmanGain(const Eigen::Ref<const Eigen::MatrixXd> &P,
                                               const Eigen::Ref<const Eigen::MatrixXd> &H,
                                               const Eigen::Ref<const Eigen::MatrixXd> &R)
    {
        Eigen::MatrixXd Ht = H.transpose();
        Eigen::MatrixXd S = (H * P * Ht) + R;
        Eigen::MatrixXd K = P *Ht * S.inverse();
        return K;
    }

    /**
     * @brief update
     * Perform the update step.
     *
     * @param[in,out] x 
     * The mean of stat vector  {VectorXd &}.
     * 
     * @param[in,out] P
     * The mean of stat vector  {VectorXd &}.
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
    static void update(Eigen::VectorXd & x,
                       Eigen::MatrixXd & P,
                       const Eigen::Ref<const Eigen::VectorXd> &Y,
                       const Eigen::Ref<const Eigen::MatrixXd> &H,
                       const Eigen::Ref<const Eigen::MatrixXd> &K)
    {
        x = x + (K * Y);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x.size(), x.size());
        P = (I - K * H) * P;
    }

};

#endif /* KALMAN_FILER_H_ */
/**
 *  @}
 */
