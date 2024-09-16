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
#include "t24e_ekf/EKF.h"

EKF::EKF() {

    // initialize the state vector
    // x, y, theta, v
    this->state_ = Eigen::VectorXd(4);
    this->state_ << 0, 0, 0, 0;

    // initialize the covariance matrix
    this->sigma_ = Eigen::MatrixXd(4, 4);
    this->sigma_ << 0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0.1, 0,
                    0, 0, 0, 0.1;

    // initialize the state transition matrix
    this->F_ = Eigen::MatrixXd(4, 4);

    // initialize the motion model variables covariance noise
    this->sigma_noise_ = Eigen::MatrixXd(2, 2);
    this->sigma_noise_ << pow(VELOCITY_MEASUREMENT_STD, 2), 0,
                          0, pow(ANGLE_MEASUREMENT_STD, 2);

    // initialize the motion noise covariance matrix
    this->R_ = Eigen::MatrixXd(4, 4);

    // initialize the measurement noise covariance matrix
    this->Q_ = Eigen::MatrixXd(5, 5);
    this->Q_ = POSITION_MEASUREMENT_VAR, 0, 0, 0, 0,
               0, POSITION_MEASUREMENT_VAR, 0, 0, 0,
               0, 0, HEADING_MEASUREMENT_VAR, 0, 0,
               0, 0, 0, VELOCITY_MEASUREMENT_VAR, 0,
               0, 0, 0, 0, VELOCITY_MEASUREMENT_VAR;

    // initialize the measurement model matrix
    this->H_ = Eigen::MatrixXd(5, 4);


}

Eigen::MatrixXd EKF::compute_F(Eigen::VectorXd u) {

    // compute the state transition matrix
    this->F_ = 1, 0, -u(0) * sin(this->state_(2)) * this->delta_t_, cos(this->state_(2)) * this->delta_t_,
               0, 1, u(0) * cos(this->state_(2)) * this->delta_t_, sin(this->state_(2)) * this->delta_t_,
               0, 0, 1, (tan(u(1)) / WHEELBASE_M) * this->delta_t_,
               0, 0, 0, 1;

    return this->F_;
}

Eigen::MatrixXd EKF::compute_H(Eigen::VectorXd z) {

    // compute the linear velocity
    double v = sqrt(pow(this->state_(3), 2) + pow(this->state_(4), 2));

    // compute the measurement model matrix
    this->H_ = 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, -v * sin(this->state_(2)), cos(this->state_(2)),
                0, 0, v * cos(this->state_(2)), sin(this->state_(2));

    return this->H_;
}




