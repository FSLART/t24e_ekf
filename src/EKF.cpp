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

EKF::EKF(double delta_t) {

    // assign the time step
    this->delta_t_ = delta_t;

    // initialize the state vector
    // x, y, theta, v
    this->state_ = Eigen::VectorXd(4);
    this->state_ << 0.0, 0.0, 0.0, 0.0;

    // initialize the covariance matrix
    this->sigma_ = Eigen::MatrixXd(4, 4);
    this->sigma_ << 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.1, 0.0,
                    0.0, 0.0, 0.0, 0.1;

    // initialize the state transition matrix
    this->F_ = Eigen::MatrixXd(4, 4);

    // initialize the motion model variables covariance noise
    this->sigma_noise_ = Eigen::MatrixXd(2, 2);
    this->sigma_noise_ << pow(VELOCITY_MEASUREMENT_MOTOR_STD, 2), 0.0,
                          0.0, pow(ANGLE_MEASUREMENT_STD, 2);

    // initialize the Jacobian of the motion model with respect to the noise
    this->G_ = Eigen::MatrixXd(4, 2);

    // initialize the motion noise covariance matrix
    this->R_ = Eigen::MatrixXd(4, 4);

    // initialize the measurement noise covariance matrix
    this->Q_ = Eigen::MatrixXd(5, 5);
    this->Q_ << POSITION_MEASUREMENT_VAR, 0.0, 0.0, 0.0, 0.0,
               0.0, POSITION_MEASUREMENT_VAR, 0.0, 0.0, 0.0,
               0.0, 0.0, HEADING_MEASUREMENT_VAR, 0.0, 0.0,
               0.0, 0.0, 0.0, VELOCITY_MEASUREMENT_VAR, 0.0,
               0.0, 0.0, 0.0, 0.0, VELOCITY_MEASUREMENT_VAR;

    // initialize the measurement model matrix
    this->H_ = Eigen::MatrixXd(5, 4);

    // initialize the Kalman gain
    this->K_ = Eigen::MatrixXd(4, 5);
}

Eigen::MatrixXd EKF::compute_F(Eigen::VectorXd u) {

    // compute the state transition matrix
    this->F_ << 1.0, 0.0, -u(0) * sin(this->state_(2)) * this->delta_t_, cos(this->state_(2)) * this->delta_t_,
               0.0, 1.0, u(0) * cos(this->state_(2)) * this->delta_t_, sin(this->state_(2)) * this->delta_t_,
               0.0, 0.0, 1.0, (tan(u(1)) / WHEELBASE_M) * this->delta_t_,
               0.0, 0.0, 0.0, 1.0;

    return this->F_;
}

Eigen::MatrixXd EKF::compute_G(Eigen::VectorXd u) {

    // compute the Jacobian of the motion model with respect to the noise
    this->G_ << cos(this->state_(2)) * this->delta_t_, 0.0,
               sin(this->state_(2)) * this->delta_t_, 0.0,
               (tan(u(1)) / WHEELBASE_M) * this->delta_t_, u(0) * (SEC2(u(1)) / WHEELBASE_M) * this->delta_t_,
               1.0, 0.0;

    return this->G_;
}

Eigen::MatrixXd EKF::compute_H() {

    // compute the measurement model matrix
    this->H_ << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, -this->state_(3) * sin(this->state_(2)), cos(this->state_(2)),
                0.0, 0.0, this->state_(3) * cos(this->state_(2)), sin(this->state_(2));

    return this->H_;
}

std::pair<Eigen::VectorXd,Eigen::MatrixXd> EKF::predict(Eigen::VectorXd u) {

    // compute the state transition matrix
    this->F_ = compute_F(u);

    // predict the state
    this->state_ = this->F_ * this->state_;

    // compute the Jacobian of the motion model with respect to the noise
    this->G_ = this->compute_G(u);

    // compute the motion noise covariance matrix
    this->R_ = this->G_ * this->sigma_noise_ * this->G_.transpose();

    // predict the covariance matrix
    this->sigma_ = this->F_ * this->sigma_ * this->F_.transpose() + this->R_;

    return std::make_pair(this->state_, this->sigma_);
}

std::pair<Eigen::VectorXd,Eigen::MatrixXd> EKF::update(Eigen::VectorXd z) {

    // compute the measurement model matrix
    this->H_ = compute_H();

    // compute the Kalman gain
    this->K_ = this->sigma_ * this->H_.transpose() * (this->H_ * this->sigma_ * this->H_.transpose() + this->Q_).inverse();

    // update the state
    this->state_ = this->state_ + this->K_ * (z - this->H_ * this->state_);

    // update the covariance matrix
    this->sigma_ = (Eigen::MatrixXd::Identity(4, 4) - this->K_ * this->H_) * this->sigma_;

    return std::make_pair(this->state_, this->sigma_);
}

Eigen::VectorXd EKF::get_state() const {
    return this->state_;
}

Eigen::MatrixXd EKF::get_sigma() const {
    return this->sigma_;
}
