#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
    
    /**
     * Constructor
     */
    UKF();
    
    /**
     * Destructor
     */
    virtual ~UKF();
    
    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);
    
    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);
    
    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);
    
    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);
    
    /**
     * Generates 2 * size(x_aug_) + 1 sigma points
     * and store them in the provided matrix.
     * @param Xsig_out Matrix containing the sigma points
     */
    void GenerateAugmentedSigmaPoints(MatrixXd& Xsig_out);
    
    /**
     * Predict the position of the sigma points using the
     * non-linear process function f()
     * @param[in|out] Xsig The generated sigma points as input
     *                and the predicted ones as output done in-place
     */
    void SigmaPointPrediction(MatrixXd& Xsig);
    
    
    /**
     Compute the mean and covariance matrix of the predicted sigma
     points.

     @param[out] x_out Predicted state vector
     @param[out] P_out Predicted covariance matrix
     */
    void PredictMeanAndCovariance(VectorXd& x_out, MatrixXd& P_out);
    
protected:
    
    ///* State dimension
    int n_x_;
    
    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;
    
    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;
    
    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;
    
    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;
    
    ///* time when the state is true, in us
    long long time_us_;
    
    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;
    
    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;
    
    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;
    
    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;
    
    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;
    
    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;
    
    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_ ;
    
    ///* Weights of sigma points
    VectorXd weights_;
    
    ///* Augmented state dimension
    int n_aug_;
    
    ///* Sigma point spreading parameter
    double lambda_;
    
public:
    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;
    
    ///* state covariance matrix
    MatrixXd P_;
};

#endif /* UKF_H */
