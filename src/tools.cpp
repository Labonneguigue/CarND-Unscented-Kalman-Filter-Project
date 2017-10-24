#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools()
: mOverallSquaredSum(Eigen::VectorXd(4)){}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    int vectorSize = estimations.size();

    if ( ( vectorSize == 0 ) ||
        ( vectorSize != ground_truth.size() ) ||
        ( estimations[vectorSize - 1].size() != ground_truth[vectorSize - 1].size() ) )
    {
        std::cout << " ERROR \n";
        VectorXd rmse(4);
        rmse << 0,0,0,0;
        return rmse;
    }
    else
    {
        mOverallSquaredSum = mOverallSquaredSum.array() * mOverallSquaredSum.array();
        mOverallSquaredSum *= (vectorSize - 1);

        Eigen::VectorXd difference = estimations[vectorSize -1] - ground_truth[vectorSize -1];

        difference = difference.array() * difference.array();
        // I keep the overall sum of squared differences
        mOverallSquaredSum += difference;

        // I compute the result once the latest estimations has been added to the rolling sum
        mOverallSquaredSum = mOverallSquaredSum / vectorSize;
        mOverallSquaredSum = mOverallSquaredSum.array().sqrt();
        return mOverallSquaredSum;
    }
}

void Tools::NormalizeAngle(double &angle){
    while ( angle > M_PI ){
        angle -= ( 2.0F * M_PI );
    }
    while ( angle <= -M_PI ) {
        angle += ( 2.0F * M_PI );
    }
}

bool Tools::isNISAboveLimit(double epsilon, int df){
    double df2_95 = 5.991;
    double df3_95 = 7.815;
    switch (df)
    {
        case 2:
            return ( epsilon > df2_95 ? true : false );
            break;
        case 3:
            return ( epsilon > df3_95 ? true : false );
            break;
        default:
            std::cout << "\nError in isNISAboveLimit() function ! \n";
            return true;
    }
}
