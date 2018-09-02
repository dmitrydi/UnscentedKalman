#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  //
  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  n_meas_ = 0;
  
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1,0,0,0,0,
        0,1,0,0,0,
        0,0,1,0,0,
        0,0,0,1,0,
        0,0,0,0,1;

  // time_us_ is initialized during first call of ProcessMeasurement()

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.9;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  // state dimentionality [px, py, v, psi, psi_dot]
  n_x_ = 5;

  // augmented dimentionality
  n_aug_ = 7;

  // sigma points generation parameter
  lambda_ = 3 - n_x_;

  // augmanted sigma points storage
  Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_ + 1);

  // sigma points storage
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);

  // storages for predicted sigma points in measurement space
  Zsig_pred_rad_ = MatrixXd(3, 2*n_aug_ + 1);
  Zsig_pred_lidar_ = MatrixXd(2, 2*n_aug_ + 1);

  // predicted measurement for radar
  zpred_rad_ = VectorXd(3);

  // predicted measurement for lidar
  zpred_lidar_ = VectorXd(2);

  // Radar covariance matrix
  S_rad_ = MatrixXd(3,3);

  // Measurement covariance for radar
  R_rad_ = MatrixXd(3,3);
  R_rad_ << std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0, std_radrd_*std_radrd_;

  // Lidar covariance matrix
  S_lid_ = MatrixXd(2,2);

  R_lid_ = MatrixXd(2,2);
  R_lid_ << std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;

  // weight for sigma points
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  
    weights_(i) = 0.5/(n_aug_+lambda_);
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} measurement_pack The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  const double time_conv = 1000000.;
  n_meas_ += 1;
  cout<<"Processing measurement "<<n_meas_<<endl;
  
  if (!is_initialized_) {
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho, phi, px, py;
      rho = measurement_pack.raw_measurements_[0];
      phi = measurement_pack.raw_measurements_[1];
      px = rho*cos(phi);
      py = rho*sin(phi);
      x_ << px, py, 0, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0;
    }
    time_us_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  double delta_t = (measurement_pack.timestamp_ - time_us_)/time_conv;
  UKF::Prediction(delta_t);
  cout<<"Prediction at step "<<n_meas_<<" done. Proceed with update"<<endl;
  cout<<"Sensor type: "<<measurement_pack.sensor_type_<<"("<<MeasurementPackage::LASER<<" - Laser, "<<MeasurementPackage::RADAR<<" - radar)"<<endl;

  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    UKF::UpdateLidar(measurement_pack);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    UKF::UpdateRadar(measurement_pack);
  }
  time_us_ = measurement_pack.timestamp_;

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
  UKF::AugmentedSigmaPoints(&Xsig_aug_);
  UKF::SigmaPointPrediction(&Xsig_pred_, delta_t);
  UKF::PredictMeanAndCovariance(&x_, &P_);
}

void UKF::PredictRadar(VectorXd* zpred_out, MatrixXd* S_out, MatrixXd* Zsig_rad_out) {
  // dimentionality of radar measurements
  int n_z = 3;
  // sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
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
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_ + 1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  S += R_rad_;
  *zpred_out = z_pred;
  *S_out = S;
  *Zsig_rad_out = Zsig;
  cout<<"Predicted sigma points for radar at step Zsig_rad_out "<<n_meas_<<endl;
  cout<<Zsig<<endl;
  cout<<"Predicted radar measurement at step z+pred"<<n_meas_<<endl;
  cout<<z_pred<<endl;
  cout<<"Predicted radar delta matrix at step "<<n_meas_<<endl;
  cout<<S<<endl;
}

void UKF::PredictLidar(VectorXd* zpred_out, MatrixXd* S_out, MatrixXd* Zsig_lidar_out) {
  // dimentionality of lidar measurements
  int n_z = 2;
  // sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  //transform sigma points into measurement space
  Zsig = Xsig_pred_.topLeftCorner(n_z, 2*n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_ + 1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  S += R_lid_;
  *zpred_out = z_pred;
  *S_out = S;
  *Zsig_lidar_out = Zsig;
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
  PredictLidar(&zpred_lidar_, &S_lid_, &Zsig_pred_lidar_);
  VectorXd z = VectorXd(2);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, 2);
  Tc.fill(0.0);
  //calculate cross correlation matrix
  for (int i=0; i<(2 * n_aug_ + 1); i++) {
      Tc += weights_(i)*(Xsig_pred_.col(i)-x_)*(Zsig_pred_lidar_.col(i)-zpred_lidar_).transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc*S_lid_.inverse();
  //update state mean and covariance matrix
  x_ += K*(z - zpred_lidar_);
  P_ -= K*S_lid_*K.transpose();
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
  PredictRadar(&zpred_rad_, &S_rad_, &Zsig_pred_rad_);
  VectorXd z = VectorXd(3);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
  cout<<"Radar measurement for step "<<n_meas_<<endl;
  cout<<z<<endl;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, 3);
  Tc.fill(0.0);
  //calculate cross correlation matrix
  cout<<"Values of variables before cross correlation calc:"<<endl;
  cout<<"weights:"<<endl;
  cout<<weights_<<endl;
  cout<<"Xsig_pred_"<<endl;
  cout<<Xsig_pred_<<endl;
  cout<<"x_ "<<endl;
  cout<<x_<<endl;
  cout<<"Zsig_pred_rad_ "<<endl;
  cout<<Zsig_pred_rad_<<endl;
  cout<<"zpred_rad_"<<endl;
  cout<<zpred_rad_<<endl;
  for (int i=0; i<(2 * n_aug_ + 1); i++) {
      Tc += weights_(i)*(Xsig_pred_.col(i)-x_)*(Zsig_pred_rad_.col(i)-zpred_rad_).transpose();
  }
  cout<<"Tc matrix for step "<<n_meas_<<endl;
  cout<<Tc<<endl;
 
  //calculate Kalman gain K;
  MatrixXd K = Tc*S_rad_.inverse();
  //update state mean and covariance matrix
  x_ += K*(z - zpred_rad_);
  P_ -= K*S_rad_*K.transpose();
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_aug_out) {
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5)= 0.;
  x_aug(6) = 0.;
  //create augmented covariance matrix
  P_aug = MatrixXd::Constant(n_aug_,n_aug_,0.);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  //create square root matrix
  MatrixXd P_sqrt = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for(int i=0; i<n_aug_; i++) {
      Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*P_sqrt.col(i);
      Xsig_aug.col(i+n_aug_+1) = x_aug - sqrt(lambda_+n_aug_)*P_sqrt.col(i);
  }
  
  *Xsig_aug_out = Xsig_aug;
  cout<<"Generated sigma points as step "<<n_meas_<<endl;
  cout<<Xsig_aug<<endl;
}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out, double delta_t) {
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
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
    if (fabs(yawd) > 0.001) {
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
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

  *Xsig_out = Xsig_pred;
  cout<<"Predicted sigma points at step "<<n_meas_<<endl;
  cout<<Xsig_pred<<endl;
}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {
  //const double pi = 3.1415926535897932384626433832795;
  VectorXd x;
  MatrixXd P;
  x = VectorXd(n_x_);
  P = MatrixXd(n_x_, n_x_);
  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x+ weights_(i) * Xsig_pred_.col(i);
  }
  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  *x_out = x;
  *P_out = P;
  cout<<"Predicted mean at step "<<n_meas_<<endl;
  cout<<x<<endl;
  cout<<"Predicted covariance at step "<<n_meas_<<endl;
  cout<<P<<endl;
}
