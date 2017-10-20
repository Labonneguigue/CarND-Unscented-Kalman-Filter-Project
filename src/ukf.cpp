#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define CLOSE_TO_ZERO 0.001

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
    P_ = MatrixXd(n_x_, n_x_);
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.2; // was 30
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.2; // was 30
    
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
    
    n_aug_ = 7;
    
    lambda_ = 3 - n_aug_;
    
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

void UKF::GenerateAugmentedSigmaPoints(MatrixXd& Xsig_out) {
    
    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);
    
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);
    
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;
    
    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++)
    {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }
    
    //write result
    Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd& Xsig) {
   
    //Prepare [out] matrix
    Eigen::MatrixXd Xsig_out(n_x_, 2 * n_aug_ + 1);
    double delta_t = 0.1; //time diff in sec
     //predict sigma points
    for (int i = 0; i< (( 2* n_aug_) +1); i++)
    {
        //extract values for better readability
        double p_x = Xsig(0,i);
        double p_y = Xsig(1,i);
        double v = Xsig(2,i);
        double yaw = Xsig(3,i);
        double yawd = Xsig(4,i);
        double nu_a = Xsig(5,i);
        double nu_yawdd = Xsig(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > CLOSE_TO_ZERO) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;
        
        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;
        
        //write predicted sigma point into right column
        Xsig_out(0,i) = px_p;
        Xsig_out(1,i) = py_p;
        Xsig_out(2,i) = v_p;
        Xsig_out(3,i) = yaw_p;
        Xsig_out(4,i) = yawd_p;
    }
    
    Xsig.resize(n_x_, 2 * n_aug_ + 1);
    Xsig = Xsig_out;
}

void UKF::PredictMeanAndCovariance(VectorXd& x_out, MatrixXd& P_out){
    
}
