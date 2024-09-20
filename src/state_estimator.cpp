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

#include "t24e_ekf/state_estimator.h"

StateEstimator::StateEstimator() : Node("state_estimator") {

    // initialize the subscriber for the dynamics message
    this->dynamics_sub_ = this->create_subscription<lart_msgs::msg::DynamicsCMD>(
        "dynamics", 10, std::bind(&StateEstimator::dynamics_callback, this, std::placeholders::_1));

    // initialize the subscriber for the GNSS/INS message
    this->gnss_sub_ = this->create_subscription<lart_msgs::msg::GNSSINS>(
        "gnss_ins", 10, std::bind(&StateEstimator::gnss_callback, this, std::placeholders::_1));

    // initialize the publisher for the state estimate
    this->state_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "state_estimate", 10);

    // initialize the transform broadcaster
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // initialize the EKF object
    this->ekf_ = std::make_unique<EKF>();

}

void StateEstimator::dynamics_callback(const lart_msgs::msg::DynamicsCMD& msg) {

    // create the control vector
    Eigen::VectorXd u(2);
    u << RPM_TO_MS(msg.rpm), msg.steering_angle;

    // predict the EKF
    std::pair<Eigen::VectorXd,Eigen::MatrixXd> state = this->ekf_->predict(u);

    // create the state estimate message
    geometry_msgs::msg::PoseWithCovarianceStamped state_msg = this->create_state_message(state.first, state.second);

    // publish the state estimate
    this->state_pub_->publish(state_msg);

    // create the transform message
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = state.first(0);
    transform.transform.translation.y = state.first(1);
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = sin(state.first(2) / 2);
    transform.transform.rotation.w = cos(state.first(2) / 2);

    // broadcast the transform
    this->tf_broadcaster_->sendTransform(transform);
}

void StateEstimator::gnss_callback(const lart_msgs::msg::GNSSINS& msg) {

    // create the measurement vector (x, y, theta, vx, vy)
    Eigen::VectorXd z(5);
    z << msg.position.x, msg.position.y, msg.heading, msg.velocity.x, msg.velocity.y;

    // update the EKF
    std::pair<Eigen::VectorXd,Eigen::MatrixXd> state = this->ekf_->update(z);

    // create the state estimate message
    geometry_msgs::msg::PoseWithCovarianceStamped state_msg = this->create_state_message(state.first, state.second);

    // publish the state estimate
    this->state_pub_->publish(state_msg);

    // create the transform message
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = state.first(0);
    transform.transform.translation.y = state.first(1);
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = sin(state.first(2) / 2);
    transform.transform.rotation.w = cos(state.first(2) / 2);

    // broadcast the transform
    this->tf_broadcaster_->sendTransform(transform);
}

geometry_msgs::msg::PoseWithCovarianceStamped StateEstimator::create_state_message(Eigen::VectorXd state, Eigen::MatrixXd covariance) {

    geometry_msgs::msg::PoseWithCovarianceStamped state_msg;
    state_msg.header.stamp = this->now();
    state_msg.pose.pose.position.x = state(0);
    state_msg.pose.pose.position.y = state(1);
    // quaternion conversion as per https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Intuition
    state_msg.pose.pose.orientation.x = 0.0;
    state_msg.pose.pose.orientation.y = 0.0;
    state_msg.pose.pose.orientation.z = sin(state(2) / 2);
    state_msg.pose.pose.orientation.w = cos(state(2) / 2);
    // fill the covariance (6x6 matrix: x, y, z, rx, ry, rz)
    for(short i = 0; i < 6; i++) {
        for(short j = 0; j < 6; j++) {
            state_msg.pose.covariance[i] = 0.0;
        }
    }
    // fill the xyz covariances
    for(short i = 0; i < 3; i++) {
        for(short j = 0; j < 3; j++) {
            state_msg.pose.covariance[i*6+j] = covariance.coeff(i,j);
        }
    }
    // fill the rz covariances
    state_msg.pose.covariance[5] = covariance.coeff(0,3);
    state_msg.pose.covariance[11] = covariance.coeff(1,3);
    state_msg.pose.covariance[17] = covariance.coeff(2,3);
    state_msg.pose.covariance[35] = covariance.coeff(3,3);
    state_msg.pose.covariance[30] = covariance.coeff(3,0);
    state_msg.pose.covariance[31] = covariance.coeff(3,1);
    state_msg.pose.covariance[32] = covariance.coeff(3,2);

    return state_msg;
}

int main(int argc, char *argv[]) {

    // initialize ROS2
    rclcpp::init(argc, argv);

    // create a node instance and spin
    rclcpp::spin(std::make_shared<StateEstimator>());

    // shutdown ROS after the node is destroyed
    rclcpp::shutdown();

    return 0;
}
