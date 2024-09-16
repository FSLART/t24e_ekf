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


}

Eigen::MatrixXd EKF::compute_F(Eigen::VectorXd u) {

    // compute the state transition matrix
    this->F_ = 1, 0, -u(0) * sin(this->state_(2)) * this->delta_t_, cos(this->state_(2)) * this->delta_t_,
               0, 1, u(0) * cos(this->state_(2)) * this->delta_t_, sin(this->state_(2)) * this->delta_t_,
               0, 0, 1, (tan(u(1)) / WHEELBASE_M) * this->delta_t_,
               0, 0, 0, 1;

    return this->F_;
}
