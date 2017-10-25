#include "ukf.h"
#include "tools.h"
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
{
    n_x_ = 5;      // State dimension

    n_aug_ = 7;

    lambda_ = 3 - n_aug_;

    previous_timestamp_ = 0.0F;

    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = false;
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initialization of the state vector
    x_ = Eigen::VectorXd(n_x_);
    x_.fill(0.0F);

    // initialization of the covariance matrix
    P_ = MatrixXd(n_x_, n_x_);
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.2; // was 30
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.2; // was 30

    // Lidar measurement dimension is x and y position -> 2
    n_z_laser_ = 2;
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;
    // Laser measurement noise covariance matrix
    R_laser_ = MatrixXd(n_z_laser_, n_z_laser_);
    R_laser_ << std_laspx_ * std_laspx_, 0,
                0, std_laspy_ * std_laspy_;
    // NIS counter
    nisCount_laser_ = 0;


    n_z_radar_ = 3; //set measurement dimension, radar can measure r, phi, and r_dot
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.0175; // was 0.03;
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.1; // was 0.3;

    R_radar_ = MatrixXd(n_z_radar_,n_z_radar_);
    R_radar_ <<    std_radr_*std_radr_, 0, 0,
                   0, std_radphi_*std_radphi_, 0,
                   0, 0,std_radrd_*std_radrd_;
    // NIS counter
    nisCount_radar_ = 0;


    //set vector for weights
    weights_ = VectorXd(2*n_aug_+1);
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights_(0) = weight_0;
    for ( int i=1 ; i < (2*n_aug_) + 1 ; i++) {
        weights_(i) = 0.5 / (n_aug_+lambda_);
    }

    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack)
{

    if (!is_initialized_)
    {
        std::cout << "Initialization.\n";
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            assert( measurement_pack.raw_measurements_.size() == 3 );
            float rho = measurement_pack.raw_measurements_[0]; // Distance
            float phi = measurement_pack.raw_measurements_[1]; // Bearing
            x_(0) = rho * cos(phi);
            x_(1) = rho * sin(phi);
            x_(2) = 0.0F;
            x_(3) = 0.0F;
            x_(4) = 0.0F;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            /**
             Initialize state vector from the measurement data.
             No assumptions are made on the speed and rotation speed.
             */
            assert( measurement_pack.raw_measurements_.size() == 2 );
            x_(0) = measurement_pack.raw_measurements_[0];
            x_(1) = measurement_pack.raw_measurements_[1];
            x_(2) = 0.0F;
            x_(3) = 0.0F;
            x_(4) = 0.0F;
        }

        /* Store the timestamp to be able to calculate dt when
         the next measurement comes.
         */
        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    else
    {
        float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
        previous_timestamp_ = measurement_pack.timestamp_;

        if ( dt > 0.001F){
            // Prediction only if the car has moved significantly after the last prediction step
            Prediction(static_cast<double>(dt));
        }

        if (use_laser_ && measurement_pack.sensor_type_ == MeasurementPackage::LASER){
            UpdateLidar(measurement_pack);
        }
        else if (use_radar_ && measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            UpdateRadar(measurement_pack);
        }
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(float delta_t)
{
    /**
     TODO:
     
     Complete this function! Estimate the object's location. Modify the state
     vector, x_. Predict sigma points, the state, and the state covariance matrix.
     */

    std::cout << "Prediction.\n";

    GenerateAugmentedSigmaPoints(Xsig_aug_);
    SigmaPointPrediction(static_cast<double>(delta_t));
    PredictMeanAndCovariance();
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

    Eigen::VectorXd z_pred(n_z_radar_);
    Eigen::MatrixXd S_pred(n_z_radar_, n_z_radar_);
    MatrixXd Zsig(n_z_radar_, 2 * n_aug_ + 1);

    PredictRadarMeasurement(z_pred, S_pred, Zsig);
    UpdateState(Xsig_pred_, x_, P_, Zsig, z_pred, S_pred, meas_package.raw_measurements_, x_, P_);
}

void UKF::GenerateSigmaPoints(MatrixXd& Xsig_out){

    assert(Xsig_out.rows() == n_x_);
    assert(Xsig_out.cols() == 2 * n_x_ + 1);
    Xsig_out.fill(0.0F);

    //calculate square root of P
    MatrixXd A = P_.llt().matrixL();

    //set first column of sigma point matrix
    Xsig_out.col(0)  = x_;

    //set remaining sigma points
    for (int i = 0; i < n_x_; i++)
    {
        Xsig_out.col(i+1)      = x_ + sqrt(lambda_+n_x_) * A.col(i);
        Xsig_out.col(i+1+n_x_) = x_ - sqrt(lambda_+n_x_) * A.col(i);
    }
}

void UKF::GenerateAugmentedSigmaPoints(MatrixXd& Xsig_out) {
    std::cout << "GenerateAugmentedSigmaPoints.\n";
    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);
    
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);
    
    // check size of provided Xsig_out matrix
    assert(Xsig_out.cols() == (2 * n_aug_ + 1));
    assert(Xsig_out.rows() == n_aug_);
    Xsig_out.fill(0.0F);

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
    Xsig_out.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++)
    {
        Xsig_out.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_out.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }
}

void UKF::SigmaPointPrediction(double dt)
{
    std::cout << "SigmaPointsPrediction.\n";

     //predict sigma points
    for (int i = 0; i< (( 2* n_aug_) +1); i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug_(0,i);
        double p_y = Xsig_aug_(1,i);
        double v = Xsig_aug_(2,i);
        double yaw = Xsig_aug_(3,i);
        double yawd = Xsig_aug_(4,i);
        double nu_a = Xsig_aug_(5,i);
        double nu_yawdd = Xsig_aug_(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > CLOSE_TO_ZERO) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
        }
        else {
            px_p = p_x + v*dt*cos(yaw);
            py_p = p_y + v*dt*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*dt;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5*nu_a*dt*dt * cos(yaw);
        py_p = py_p + 0.5*nu_a*dt*dt * sin(yaw);
        v_p = v_p + (nu_a * dt);
        
        yaw_p = yaw_p + 0.5*nu_yawdd*dt*dt;
        yawd_p = yawd_p + nu_yawdd*dt;
        
        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }
}

void UKF::PredictMeanAndCovariance(){
    std::cout << "PredictMeanAndCovariance.\n";
    // check size of predicted state x
    assert(x_.size() == n_x_);
    
    // check that P is square and of the right size
    assert(P_.cols() == n_x_);
    assert(P_.rows() == n_x_);
    
    //predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }
    
    //predicted state covariance matrix
    P_.fill(0.0);
    //iterate over sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        Tools::NormalizeAngle(x_diff(3));

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}

void UKF::PredictRadarMeasurement(VectorXd& z_out, MatrixXd& S_out, Eigen::MatrixXd& Zsig) {

    assert(Xsig_pred_.rows() == n_x_);
    assert(Xsig_pred_.cols() == 2 * n_aug_ + 1);

    //create matrix for sigma points in measurement space
    assert(Zsig.rows() == n_z_radar_);
    assert(Zsig.cols() == 2 * n_aug_ + 1);
       
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);
        
        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;
        
        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }
    
    //mean predicted measurement
    assert(z_out.size() == n_z_radar_);
    
    z_out.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_out = z_out + weights_(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    assert(S_out.cols() == n_z_radar_);
    assert(S_out.rows() == n_z_radar_);
    
    S_out.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_out;
        
        //angle normalization
        Tools::NormalizeAngle(z_diff(1));
        
        S_out = S_out + weights_(i) * z_diff * z_diff.transpose();
    }

    S_out = S_out + R_radar_;
}

void UKF::UpdateState(MatrixXd &Xsig_pred,
                      VectorXd &x_pred_mean,
                      MatrixXd &P_pred_covs,
                      MatrixXd &Zsig,
                      VectorXd &z_pred,
                      MatrixXd &S,
                      VectorXd &z,
                      VectorXd &x_out,
                      MatrixXd &P_out) {
    
    // check Xsig_pred size
    assert(Xsig_pred.rows() == n_x_);
    assert(Xsig_pred.cols() == 2 * n_aug_ + 1);
    
    //check x_pred_mean vector size (predicted state mean)
    assert(x_pred_mean.size() == n_x_);
    
    assert(P_pred_covs.cols() == n_x_);
    assert(P_pred_covs.rows() == n_x_);
    
    assert(Zsig.rows() == z_pred.size());
    
    assert(S.rows() == z.size());
    assert(z.size() == S.cols());
    
    //create matrix for cross correlation Tc
    //
    MatrixXd Tc = MatrixXd(n_x_, S.cols());

    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        Tools::NormalizeAngle(z_diff(1));
        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x_pred_mean;
        //angle normalization
        Tools::NormalizeAngle(x_diff(3));
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    Tools::NormalizeAngle(z_diff(1));
    
    //update state mean and covariance matrix
    x_out = x_pred_mean + K * z_diff;
    P_out = P_pred_covs - K*S*K.transpose();
}



