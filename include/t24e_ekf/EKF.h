/*
MIT License

Copyright (c) 2024 Associação Académica de Desportos Motorizados de Leiria (AADML)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef T24E_EKF_EKF_H_
#define T24E_EKF_EKF_H_

#include <eigen3/Eigen/Dense>
#include <utility>
#include "lart_common/lart_common.h"

/*! \brief Extended Kalman Filter class. */
class EKF {
    public:
        /*! \brief Constructor of the EKF class. */
        EKF();

    private:
        /*! \brief State vector. */
        Eigen::VectorXd state_;

        /*! \brief Covariance matrix. */
        Eigen::MatrixXd sigma_;

        /*! \brief State transition matrix (Jacobian of the motion model). */
        Eigen::MatrixXd F_;

        /*! \brief Covariance matrix for the variables of the motion model (2x2 velocity and st. angle). */
        Eigen::MatrixXd sigma_noise_;

        /*! \brief Motion noise covariance matrix. */
        Eigen::MatrixXd R_;

        /*! \brief Measurement noise covariance matrix. */
        Eigen::MatrixXd Q_;

        /*! \brief Measurement matrix (Jacobian of the measurement model). */
        Eigen::MatrixXd H_;

        /*! \brief Kalman gain. */
        Eigen::MatrixXd K_;

        /*! \brief Predict the state.
         * \param u Control input vector.
         * \return Pair containing the predicted state vector and covariance matrix.
         */
        std::pair<Eigen::VectorXd,Eigen::MatrixXd> predict(Eigen::VectorXd u);

        /*! \brief Update the state. 
         * \param z Measurement vector.
         * \return Pair containing the updated state vector and covariance matrix.
         */
        std::pair<Eigen::VectorXd,Eigen::MatrixXd> update(Eigen::VectorXd z);

};

#endif // T24E_EKF_EKF_H_