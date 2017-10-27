#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
    /**
     * Constructor.
     */
    Tools();
    
    /**
     * Destructor.
     */
    virtual ~Tools();
    
    /**
     * A helper method to calculate RMSE.
     */
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

    /**
     Normalize an angle within ]-PI;PI]

     @param[in|out] angle to be normalized
     */
    static void NormalizeAngle(double &angle);

    /**
     Return the Normalized Innovation Squared (NIS) for the current measurement.

     @param z_pred  Predicted measurement mean
     @param z_meas  Actual measurement
     @param S       Predicted measurement covariance matrix
     @return Epsilon    Scalar number 

     */
    static inline double NIS(const Eigen::VectorXd &z_pred, const Eigen::VectorXd &z_meas, const Eigen::MatrixXd &S){
        return ((z_meas - z_pred).transpose() * S.inverse() * (z_meas - z_pred));
    }

    /**
     Returns whether the NIS calculated (epsilon) is above
     the expected maximum value for 95% of all cases relative
     to the specified degree of freedom.

     @param epsilon NIS value
     @param df degree of freedom
     @return Whether the measurement is too far of the prediction
     
     @note epsilon follows a chi-squared distribution dependent on
     the degree of freedom.

     Probability less than the critical value
     ν           0.90      0.95     0.975      0.99     0.999

     1          2.706     3.841     5.024     6.635    10.828
     2          4.605     5.991     7.378     9.210    13.816
     3          6.251     7.815     9.348    11.345    16.266
     4          7.779     9.488    11.143    13.277    18.467
     5          9.236    11.070    12.833    15.086    20.515
     6         10.645    12.592    14.449    16.812    22.458
     */
    static bool isNISAboveLimit(double epsilon, int df);


private:

    Eigen::VectorXd mOverallSquaredSum;

};

#endif /* TOOLS_H_ */
