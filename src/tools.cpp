#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
     TODO:
     * Calculate the RMSE here.
     */
    return Eigen::VectorXd();
}

void Tools::NormalizeAngle(double &angle){
    while ( angle > M_PI ){
        angle -= ( 2.0F * M_PI );
    }
    while ( angle < -M_PI ) {
        angle += ( 2.0F * M_PI );
    }
}
