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
#include <cmath>
#include <iostream>
#include <chrono>
#include "lart_common/lart_common.h"

/*! \brief Standard deviation of the initial position. */
#define INIT_POS_STD 30.0

/*! \brief Variation of the initial position. */
#define INIT_POS_VAR pow(INIT_POS_STD, 2)

/*! \brief Standard deviation of the initial theta. */
#define INIT_THETA_STD 360.0

/*! \brief Variation of the initial theta. */
#define INIT_THETA_VAR pow(INIT_THETA_STD, 2)

/*! \brief Standard deviation of the initial velocity. */
#define INIT_VEL_STD 10.0

/*! \brief Variation of the initial velocity. */
#define INIT_VEL_VAR pow(INIT_VEL_STD, 2)

/*! \brief Standard deviation of the velocity measurement noise. */
#define VELOCITY_MEASUREMENT_MOTOR_STD 0.2

/*! \brief Standard deviation of the angle measurement noise. */
#define ANGLE_MEASUREMENT_STD 0.0000467

/*! \brief Standard deviation of the position measurement noise. */
#define POSITION_MEASUREMENT_STD 0.05

/*! \brief Variance of the position measurement noise. */
#define POSITION_MEASUREMENT_VAR pow(POSITION_MEASUREMENT_STD, 2)

/*! \brief Standard deviation of the heading measurement noise. */
#define HEADING_MEASUREMENT_STD 0.5

/*! \brief Variance of the heading measurement noise. */
#define HEADING_MEASUREMENT_VAR pow(HEADING_MEASUREMENT_STD, 2)

/*! \brief Standard deviation of the velocity measurement noise. */
#define VELOCITY_MEASUREMENT_STD 0.05

/*! \brief Variance of the velocity measurement noise. */
#define VELOCITY_MEASUREMENT_VAR pow(VELOCITY_MEASUREMENT_STD, 2)

/*! \brief Macro to compute the secant of an angle. */
#define SEC2(ANGLE) 1 / (pow(cos(ANGLE), 2))


/*! \brief Extended Kalman Filter class. */
class EKF {
    public:
        /*! \brief Constructor of the EKF class. */
        EKF();

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

        /*! \brief Get the state vector. 
         * \return State vector.
         */
        Eigen::VectorXd get_state() const;

        /*! \brief Get the covariance matrix. 
         * \return Covariance matrix.
         */
        Eigen::MatrixXd get_sigma() const;

    private:

        /*! \brief Time step. */
        double delta_t_;

        /*! \brief Last time the filter was updated. Used to compute the time step. */
        std::chrono::high_resolution_clock::time_point last_time_;

        /*! \brief State vector. Is a 4-dimensional vector (x, y, theta, velocity)*/
        Eigen::VectorXd state_;

        /*! \brief Covariance matrix. */
        Eigen::MatrixXd sigma_;

        /*! \brief State transition matrix (Jacobian of the motion model). */
        Eigen::MatrixXd F_;

        /*! \brief Covariance matrix for the variables of the motion model (2x2 velocity and st. angle). */
        Eigen::MatrixXd sigma_noise_;

        /*! \brief Jacobian of the motion model with respect to the noise. */
        Eigen::MatrixXd G_;

        /*! \brief Motion noise covariance matrix. */
        Eigen::MatrixXd R_;

        /*! \brief Measurement noise covariance matrix. */
        Eigen::MatrixXd Q_;

        /*! \brief Measurement matrix (Jacobian of the measurement model). */
        Eigen::MatrixXd H_;

        /*! \brief Kalman gain. */
        Eigen::MatrixXd K_;

        /*! \brief Compute the motion model. 
         * \param u Control vector.
         * \return State vector.
         */
        Eigen::VectorXd f(Eigen::VectorXd u);

        /*! \brief Compute the measurement model. 
         * \return Expected measurement vector.
         */
        Eigen::VectorXd h();

        /*! \brief Compute the state transition matrix from the control vector.
         * \param u Control vector.
         * \return State transition matrix.
         */
        Eigen::MatrixXd compute_F(Eigen::VectorXd u);

        /*! \brief Compute the Jacobian of the motion model with respect to the noise. 
         * \param u Control vector.
         * \return Jacobian of the motion model.
         */
        Eigen::MatrixXd compute_G(Eigen::VectorXd u);

        /*! \brief Compute the measurement matrix. Maps the current state to the expected measurement.
         * \return Measurement matrix.
         */
        Eigen::MatrixXd compute_H();
};

#endif // T24E_EKF_EKF_H_