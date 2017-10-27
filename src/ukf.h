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
     * @param measurement_pack The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(const MeasurementPackage& measurement_pack);
    
    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(const float delta_t);
    
    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(const MeasurementPackage& meas_package);
    
    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(const MeasurementPackage& meas_package);

    /**
     * Generates 2 * size(x_aug_) + 1 sigma points
     * and store them in the provided matrix.
     *
     * @param[out] Xsig_out Matrix containing the sigma points
     */
    void GenerateAugmentedSigmaPoints(MatrixXd& Xsig_out);
    
    /**
     * @copydoc GenerateAugmentedSigmaPoints(MatrixXd&)
     *
     * @param[in] std_a  Acceleration process noise
     * @param[in] std_yawdd Yaw acceleration process noise
     */
    void GenerateAugmentedSigmaPoints(MatrixXd& Xsig_out, const float std_a, const float std_yawdd);


    /**
     * Predict the position of the sigma points using the
     * non-linear process function f()
     * @note The generated sigma points as input and the predicted
     *       ones as output done on member variables Xsig_aug_ & Xsig_pred_
     * @param[in] dt  Time deelta between this measurement and the previous one
     */
    void SigmaPointPrediction(const double dt = 0.1F);

    /**
     * Compute the mean and covariance matrix of the predicted sigma
     * points.
     *
     * @note Here are the following class member modified by this method:
     *      [in]  Xsig_pred_  Predicted sigma points
     *      [out] x_ Predicted state vector
     *      [out] P_ Predicted covariance matrix
     */
    void PredictMeanAndCovariance();

    /**
     Performs the prediction of the radar measurement.

     @param[out] z_out     Predicted vector z (mean of the predicted measurement)
     @param[out] S_out     Matrix S (measurement noise covariance)
     @param[out] Zsig      Matrix Zsig sigma points in measurement space
     */
    void PredictRadarMeasurement(VectorXd& z_out, MatrixXd& S_out, MatrixXd& Zsig);

    /**
     @copydoc UKF::PredictRadarMeasurement(VectorXd, MatrixXd, MatrixXd)

     @param[in] R   Measurement noise matrix
     */
    void PredictRadarMeasurement(VectorXd& z_out, MatrixXd& S_out, Eigen::MatrixXd& Zsig, MatrixXd& R);

    /**
     Maps the generated sigma points onto the Laser measurement space
     and predict the

     @param z_out Predicted measurement mean
     @param S_out Measurement covariance
     @param Zsig  Sigma points in measurement space
     */
    void PredictLaserMeasurement(VectorXd& z_out, MatrixXd& S_out, Eigen::MatrixXd& Zsig);

    /**
     Update the State vector and covariance of the Kalman Filter

     @param Zsig Sigma points in measurement space
     @param z_pred Mean predicted measurement
     @param S Covariance predicted measurement
     @param z Incoming radar measurement

     @note The matrix Tc is the matrix of cross correlation
           K is the Kalman Gain

     */
    void UpdateState(MatrixXd &Zsig,
                     VectorXd &z_pred,
                     MatrixXd &S,
                     const VectorXd &z);

    /**
     Get the state vector

     @return x_
     */
    inline const Eigen::VectorXd& stateVector(){
        return x_;
    }

    /**
     * Set the state vector
     *
     * @param[in] x
     *
     * @return bool Returns true if the size was correct
     */
    inline bool stateVector(Eigen::VectorXd &x){
        if (x.size() == n_x_){
            x_ = x;
            return true;
        }
        return false;
    }

    /*
     * Get the state covariance matrix
     *
     * @return MatrixXd P_
     */
    inline const Eigen::MatrixXd& covarianceMatrix(){
        return P_;
    }

    /**
     * Set the state covariance matrix
     *
     * @param[in] P
     *
     * @return bool Returns true if the size was correct
     */
    inline bool covarianceMatrix(Eigen::MatrixXd &P){
        if (P.rows() == n_x_ && P.cols() == n_x_){
            P_ = P;
            return true;
        }
        return false;
    }

    /**
     * Get the predicted sigma points matrix
     *
     * @return MatrixXd Xsig_pred_
     */
    inline Eigen::MatrixXd& predictedSigmaPoints(){
        return Xsig_pred_;
    }

    /**
     * Set the predicted sigma points matrix
     *
     * @param[in] Xsig_pred
     *
     * @return bool Returns true if the size was correct
     */
    inline bool predictedSigmaPoints(Eigen::MatrixXd &Xsig_pred){
        if ( (Xsig_pred.rows() == n_x_) && (Xsig_pred.cols() == (n_sig_)) ){
            Xsig_pred_ = Xsig_pred;
            return true;
        }
        return false;
    }

    /**
     * Get the augmented sigma points matrix
     *
     * @return MatrixXd Xsig_aug_
     */
    inline Eigen::MatrixXd& augmentedSigmaPoints(){
        return Xsig_aug_;
    }

    /**
     * Set the augmented sigma points matrix
     *
     * @param[in] Xsig_aug
     *
     * @return bool Returns true if the size was correct
     */
    inline bool augmentedSigmaPoints(Eigen::MatrixXd &Xsig_aug){
        if ( (Xsig_aug.rows() == n_aug_) && (Xsig_aug.cols() == (n_sig_)) ){
            Xsig_aug_ = Xsig_aug;
            return true;
        }
        return false;
    }

private:

    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Number of augmented sigma points
    int n_sig_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;


    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;
    
    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    ///* Augmented sigma points matrix
    MatrixXd Xsig_aug_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    ///* Lidar measurement dimension
    int n_z_laser_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;
    
    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Laser measurement noise covariance matrix
    MatrixXd R_laser_;

    ///* Number of laser measurements
    uint16_t nbLaserMeasurements_;

    ///* Number of laser measurement below the expected 95% limit
    uint16_t laserNISBelowLimit_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;
    
    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;
    
    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    ///* Radar measurement noise covariance matrix
    MatrixXd R_radar_;

    ///* Radar measurement dimension
    int n_z_radar_;

    ///* Number of laser measurements
    uint16_t nbRadarMeasurements_;

    ///* Number of laser measurement below the expected 95% limit
    uint16_t radarNISBelowLimit_;


    ///* Weights of sigma points
    VectorXd weights_;

    ///* Sigma point spreading parameter
    double lambda_;


    ///* Previous timestamp to compute dt (time bwt measurement)
    long long previous_timestamp_;

};

#endif /* UKF_H */
