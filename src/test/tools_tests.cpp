#include <iostream>
#include "gtest/gtest.h"

#include  "tools.h"

#define NEAR_ZERO 10e-8

TEST(Normalize, EdgeCases){
    double angle = M_PI;
    Tools::NormalizeAngle(angle);
    ASSERT_EQ(angle, M_PI);

    angle = 0.0F;
    Tools::NormalizeAngle(angle);
    ASSERT_EQ(angle, 0.0F);

    angle = -M_PI;
    Tools::NormalizeAngle(angle);
    ASSERT_EQ(angle, M_PI);

    angle = - ( 100 * M_PI );
    Tools::NormalizeAngle(angle);
    ASSERT_NEAR(angle, 0.0F, NEAR_ZERO);
}

TEST(NIS, NIS_ZeroVectors){
    Eigen::VectorXd z_pred = Eigen::VectorXd(3);
    z_pred << 0.0,
              0.0,
              0.0;

    Eigen::VectorXd z_meas = Eigen::VectorXd(3);
    z_meas << 0.0,
              0.0,
              0.0;

    Eigen::MatrixXd S_pred = Eigen::MatrixXd(3, 3);
    S_pred << 9.0, 2.0, 3.0,
                3.0, 1.0, 1.0,
                2.0, 3.0, -3.0;

    double result = Tools::NIS(z_pred, z_meas, S_pred);
    ASSERT_EQ(result, 0.0);
}

TEST(NIS, NIS_ProperFunctionning){
    Eigen::VectorXd z_pred = Eigen::VectorXd(3);
    z_pred << 0.0,
              0.0,
              0.0;

    Eigen::VectorXd z_meas = Eigen::VectorXd(3);
    z_meas << 1.0,
              1.0,
              1.0;

    Eigen::MatrixXd S_pred = Eigen::MatrixXd(3, 3);
    S_pred << 1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0;

    double result = Tools::NIS(z_pred, z_meas, S_pred);
    ASSERT_EQ(result, 3.0);
}

TEST(isNISAboveLimit, DF_2_Below){
    double epsilon = 5.0F;
    int df = 2;
    bool result = Tools::isNISAboveLimit(epsilon, df);
    ASSERT_FALSE(result);
}

TEST(isNISAboveLimit, DF_2_Above){
    double epsilon = 6.0F;
    bool result = Tools::isNISAboveLimit(epsilon, 2);
    ASSERT_TRUE(result);
}

TEST(isNISAboveLimit, DF_3_Below){
    double epsilon = 7.0F;
    bool result = Tools::isNISAboveLimit(epsilon, 3);
    ASSERT_FALSE(result);
}

TEST(isNISAboveLimit, DF_3_Above){
    double epsilon = 8.0F;
    bool result = Tools::isNISAboveLimit(epsilon, 3);
    ASSERT_TRUE(result);
}
