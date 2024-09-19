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
#include <gtest/gtest.h>
#include <memory>
#include "t24e_ekf/EKF.h"

/*! \brief Test fixture for the EKF methods. */
class EKFTest : public ::testing::Test {
    protected:
        EKFTest() {
            // initialize the EKF object
            this->ekf = std::make_unique<EKF>(0.1);
        }

        /*! \brief Unique pointer to EKF object. */
        std::unique_ptr<EKF> ekf;
};

TEST_F(EKFTest, InitEKFShapes)
{
    // get the initial state and covariance
    Eigen::VectorXd state = this->ekf->get_state();
    Eigen::MatrixXd sigma = this->ekf->get_sigma();

    // assert the shape of the state
    ASSERT_EQ(state.size(), 4);

    // assert the shape of the covariance matrix
    ASSERT_EQ(sigma.rows(), 4);
    ASSERT_EQ(sigma.cols(), 4);
}

TEST_F(EKFTest, PredictAndMeasure)
{
    // create the control vector
    Eigen::VectorXd u(2);
    u << 1.0, 0.1;

    // predict the state
    std::pair<Eigen::VectorXd,Eigen::MatrixXd> prediction = this->ekf->predict(u);

    // get the predicted state and covariance
    Eigen::VectorXd state = prediction.first;
    Eigen::MatrixXd sigma = prediction.second;

    // assert x to be 0.1
    ASSERT_NEAR(state(0), 0.1, 1e-6);

    // assert y to be 0.0
    ASSERT_NEAR(state(1), 0.0, 1e-6);

    // assert theta to be 0.0064732
    ASSERT_NEAR(state(2), 0.0064732, 1e-6);

    // assert velocity to be 1.0
    ASSERT_NEAR(state(3), 1.0, 1e-6);

    // create the measurement vector
    Eigen::VectorXd z(5);
    z << 0.1, 0.0, 0.0064732, 0.99, 0.02;

    // update the state
    std::pair<Eigen::VectorXd,Eigen::MatrixXd> update = this->ekf->update(z);

    // get the updated state and covariance
    state = update.first;
    sigma = update.second;

    // assert x to be 0.1
    ASSERT_NEAR(state(0), 0.1, 1e-2);

    // assert y to be 0.0
    ASSERT_NEAR(state(1), 0.0, 1e-2);

    // assert theta to be 0.0064732
    ASSERT_NEAR(state(2), 0.0064732, 1e-2);

    // assert velocity to be 1
    ASSERT_NEAR(state(3), 1.0, 1e-1);

}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
