#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;
  
  MatrixXd Zsig_pred_rad_;
  MatrixXd Zsig_pred_lidar_;
  VectorXd zpred_rad_;
  VectorXd zpred_lidar_;
  MatrixXd S_rad_;
  MatrixXd R_rad_;
  MatrixXd S_lid_;
  MatrixXd R_lid_;
  
  
  

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* augmented sigma points matrix
  MatrixXd Xsig_aug_;

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

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;


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

  void PredictRadar(VectorXd* zpred_out, MatrixXd* S_out, MatrixXd* Zsig_rad_out);

  void PredictLidar(VectorXd* zpred_out, MatrixXd* S_out, MatrixXd* Zsig_lidar_out);

  void UpdateLidar(MeasurementPackage meas_package);

  void UpdateRadar(MeasurementPackage meas_package);

  void AugmentedSigmaPoints(MatrixXd* Xsig_aug_out);

  void SigmaPointPrediction(MatrixXd* Xsig_out, double delta_t);

  void PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out);
};

#endif /* UKF_H */
