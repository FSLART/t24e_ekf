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
#ifndef T24E_EKF_STATE_ESTIMATOR_H_
#define T24E_EKF_STATE_ESTIMATOR_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "lart_msgs/msg/gnssins.hpp"
#include "lart_msgs/msg/dynamics.hpp"

/*! \brief Simple subscriber class. Subscribes a string message. */
class StateEstimator : public rclcpp::Node {

    public:
        /*! \brief Constructor of the state estimator class. */
        StateEstimator();

    private:
        /*! \brief Subscriber for the GNSS/INS message (used for the measurement model). */
        rclcpp::Subscription<lart_msgs::msg::GNSSINS>::SharedPtr gnss_sub_;

        /*! \brief Subscriber for the dynamics message (used for the motion model). */
        rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr dynamics_sub_;

        /*! \brief Callback function for the GNSS/INS message. */
        void gnss_callback(const lart_msgs::msg::GNSSINS::SharedPtr msg);

        /*! \brief Callback function for the dynamics message. */
        void dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);

        /*! \brief Publisher for the state estimate. */
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr state_pub_;

        /*! \brief Current state vector (x, y, theta, v). */
        Eigen::VectorXd state_;

};

#endif // T24E_EKF_STATE_ESTIMATOR_H_
