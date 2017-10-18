#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
: n_x_(5)            // State dimension
, x_(VectorXd(n_x_)) // initial state vector
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;
    
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;
    
    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;
    
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;
    
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;
    
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    
    /**
     TODO:
     
     Complete the initialization. See ukf.h for other member properties.
     
     Hint: one or more values initialized above might be wildly off...
     */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
     TODO:
     
     Complete this function! Make sure you switch between lidar and radar
     measurements.
     */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
     TODO:
     
     Complete this function! Estimate the object's location. Modify the state
     vector, x_. Predict sigma points, the state, and the state covariance matrix.
     */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     TODO:
     
     Complete this function! Use lidar data to update the belief about the object's
     position. Modify the state vector, x_, and covariance, P_.
     
     You'll also need to calculate the lidar NIS.
     */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     TODO:
     
     Complete this function! Use radar data to update the belief about the object's
     position. Modify the state vector, x_, and covariance, P_.
     
     You'll also need to calculate the radar NIS.
     */
}

void UKF::GenerateSigmaPoints(MatrixXd& Xsig_out) {
    
    //calculate square root of P
    MatrixXd A = P_.llt().matrixL();
    
    //set first column of sigma point matrix
    Xsig_out.col(0)  = x_;
    
    //set remaining sigma points
    for (int i = 0; i < n_x_; i++)
    {
        Xsig_out.col(i+1)     = x_ + sqrt(lambda_+n_x_) * A.col(i);
        Xsig_out.col(i+1+n_x_) = x_ - sqrt(lambda_+n_x_) * A.col(i);
    }
    
}
