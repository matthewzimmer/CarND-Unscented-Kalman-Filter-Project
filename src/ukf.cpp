#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.0;

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

  NIS_radar_ = 0;

  NIS_laser_ = 0;

  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  n_z_ = 3;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //create matrix for sigma points in measurement space
  Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  z_pred_ = VectorXd(n_z_);

  //measurement covariance matrix S
  S_ = MatrixXd(n_z_, n_z_);

  // measurement covariance noise matrix R
  R_ = MatrixXd(n_z_, n_z_);
  R_ << std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0, std_radrd_ * std_radrd_;

  //create matrix with predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); // 5 x 15

  //create matrix with augmented sigma points
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1); // 7 x 15

  //create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    double weight = 0.5 / (n_aug_ + lambda_);
//    double weight = 1 / (2 * (lambda_ + n_aug_));
    weights_(i) = weight;
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} measurement_pack The latest measurement data of
 * either radar or laser.
 */
bool UKF::ProcessMeasurement(MeasurementPackage &measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  Eigen::VectorXd z = measurement_pack.raw_measurements_;

  // prevent devision by zero
  double p_x = z(0);
  double p_y = z(1);
  if (p_x == 0 || p_y == 0) {
    return false;
  }

  if (!is_initialized_) { // first measurement
    /**
      * Initialize the state ekf_.x_ with the first measurement.
    */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state
      // output the estimation in the cartesian coordinates. We do the conversion to
      // cartesian because we always want x_ in cartesian coordinates.
      double rho = z(0);
      double phi = z(1);
      x_ << rho * cos(phi), rho * sin(phi), 1.0, 0, 0;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_ << p_x, p_y, 1.0, 0, 0;
    }

    // The first measurement is reliable so use the
    // identity matrix as the initial covariance matrix
//    P_ << MatrixXd::Identity(n_x_, n_x_);
    P_ = x_ * x_.transpose();
    cout << "P = " << endl << P_ << endl;

    time_us_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return true;
  }

  if ((use_laser_ && measurement_pack.sensor_type_ == MeasurementPackage::LASER) ||
      (use_radar_ && measurement_pack.sensor_type_ == MeasurementPackage::RADAR)) {
//    cout << "x = " << endl << x_ << endl;

    //compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - time_us_) / 1000000.0;  //dt - expressed in seconds
    time_us_ = measurement_pack.timestamp_;
//    cout << "dt= " << dt << endl << endl;

    /**
      * PREDICT
      *
      *   1. Generate sigma points
      *   2. Predict augmented sigma points
      *   3. Predict mean and covariance
      *
      *
      * UPDATE
      *
      *   1. Predict Measurement
      *   2. Update State
      *
      * TIPS
      *
      *  * Process noise vector "new_K" has a non-linear effect:
      *
      *     vk = [v_a,k, v_psi_dd]^T
      */
    Prediction(dt);
    measurement_pack.Update(*this);

    return true;
  }

  return false;
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
//  GenerateSigmaPoints();
  AugmentedSigmaPoints();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();
}

void UKF::AugmentedSigmaPoints() {
  // classroom video used to implemented this algorithm:
  //
  // https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/daf3dee8-7117-48e8-a27a-fc4769d2b954/concepts/684424d0-4d25-4aab-8d1c-cf4869fe5f6e

  double sqr_lambda = sqrt(lambda_ + n_aug_);

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
//  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  // add std_a_ and std_yawdd_
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  MatrixXd Q = MatrixXd(2, 2);
  Q << std_a_ * std_a_, 0.0,
          0.0, std_yawdd_ * std_yawdd_;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q;

  //create square root matrix
  //calculate square root of P_
  MatrixXd A = P_aug.llt().matrixL();
  MatrixXd B = sqr_lambda * A;

  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug;
  for (int i = 1; i <= n_aug_; i++) {
    Xsig_aug_.col(i) = x_aug + B.col(i - 1);
    Xsig_aug_.col(i + n_aug_) = x_aug - B.col(i - 1);
  }

  //print result
//  std::cout << "Xsig_aug = " << std::endl << Xsig_aug_ << std::endl;
}

void UKF::SigmaPointPrediction(double delta_t) {
//  double delta_t = time_us_; //time diff in sec

  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  VectorXd x;
  size_t N = 2 * n_aug_ + 1;
  for (int i = 0; i < N; i++) {
    x = Xsig_aug_.col(i);

    double p_x = x(0);
    double p_y = x(1);
    double v = x(2);
    double yaw = x(3);
    double yaw_dot = x(4);
    double nu_a = x(5);
    double nu_yawdd = x(6);

    VectorXd a = VectorXd(5);
    if (fabs(yaw_dot) > 0.001) {
      a << (v / yaw_dot) * (sin(yaw + yaw_dot * delta_t) - sin(yaw)),
              (v / yaw_dot) * (cos(yaw) - cos(yaw + yaw_dot * delta_t)),
              0,
              yaw_dot * delta_t,
              0;
    } else { // since yaw_dot is zero, do linear approximation (avoiding division by zero, too)
      a << v * cos(yaw) * delta_t,
              v * sin(yaw) * delta_t,
              0,
              0, // yaw_dot * dt,  <--- actual formula but yaw_dot is zero in this case so not calculating
              0;
    }

    VectorXd b = VectorXd(5);
    b << 0.5 * (delta_t * delta_t) * cos(yaw) * nu_a,
            0.5 * (delta_t * delta_t) * sin(yaw) * nu_a,
            delta_t * nu_a,
            0.5 * (delta_t * delta_t) * nu_yawdd,
            delta_t * nu_yawdd;
    VectorXd x_seg = x.segment(0, n_x_);
    Xsig_pred_.col(i) = x_seg + a + b;
  }

  //print result
//  std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;
}

void UKF::PredictMeanAndCovariance() {

  size_t N = 2 * n_aug_ + 1;
  //predict state mean
  for (int i = 0; i < N; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix
  VectorXd x_diff;
  for (int i = 0; i < N; i++) {
    x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    /**
     * When calculating the predicted state covariance matrix,
     * in the equation we always need the difference between
     * the mean predicted state and a sigma points. The problem
     * here is that the state contains an angle. As you have
     * learned before, subtracting angles is a problem for Kalman
     * filters, because the result might be 2π plus a small angle,
     * instead of just a small angle. That’s why I normalize
     * the angle here.
     *
     *
     *
     * LIFE LESSON:
     *
     * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     *
     *          Make sure you always normalize when you calculate the difference between angles
     *
     * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     */
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

  //print result
//  std::cout << "Predicted state" << std::endl;
//  std::cout << x_ << std::endl;
//  std::cout << "Predicted covariance matrix" << std::endl;
//  std::cout << P_ << std::endl;
}

void UKF::PredictRadarMeasurement() {

  // mean state x
  VectorXd x = VectorXd(n_x_);

  size_t N = 2 * n_aug_ + 1;
  //transform sigma points into measurement space
  for (int i = 0; i < N; i++) {
    x = Xsig_pred_.col(i);

    double p_x = x(0);
    double p_y = x(1);
    double v = x(2);
    double yaw = x(3);
    double yawd = x(4);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    double rho = sqrt((p_x * p_x) + (p_y * p_y));
    double phi = atan2(p_y, p_x);
    double rho_dot = (p_x * v1 + p_y * v2) / rho;

    Zsig_(0, i) = rho;
    Zsig_(1, i) = phi;
    Zsig_(2, i) = rho_dot;
  }


  //calculate mean predicted measurement
  for (int i = 0; i < N; i++) {
    z_pred_ += weights_(i) * Zsig_.col(i);
  }

  //calculate measurement covariance matrix S
  VectorXd z_diff = VectorXd(n_z_);
  for (int i = 0; i < N; i++) {
    z_diff = Zsig_.col(i) - z_pred_;

    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    S_ += weights_(i) * z_diff * z_diff.transpose();
  }
  S_ += R_;

  //print result
  std::cout << "z_pred: " << std::endl << z_pred_ << std::endl;
  std::cout << "S: " << std::endl << S_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} measurement_pack
 */
//void UKF::Update(LaserMeasurement &measurement_pack) {
//  std::cout << "Update LASER" << std::endl;
//}

//void UKF::Update(RadarMeasurement &measurement_pack) {
//  std::cout << "Update RADAR" << std::endl;
//}

void UKF::UpdateLidar(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  std::cout << "Update LASER" << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} measurement_pack
 */
void UKF::UpdateRadar(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  PredictRadarMeasurement();

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_);
  z <<
    measurement_pack.raw_measurements_[0],   //rho in m
    measurement_pack.raw_measurements_[1],   //phi in rad
    measurement_pack.raw_measurements_[2];   //rho_dot in m/s

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  // Kalman Gain
  MatrixXd K = MatrixXd(n_x_, n_z_);

  //calculate cross correlation matrix
  size_t N = 2*n_aug_+1;
  Tc.fill(0.0);
  for(int i = 0; i < N; i++) {
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc += weights_(i) * (x_diff * z_diff.transpose());
  }

  //calculate Kalman gain K;
  K = Tc * S_.inverse();

  VectorXd z_diff = z - z_pred_;

  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_ * K.transpose();

  //print result
  std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;

  std::cout << "Updated RADAR" << std::endl << std::endl << std::endl;
}

//void UKF::GenerateSigmaPoints() {
//  double sqr_lambda = sqrt(lambda_ + n_x_);
//
//  //calculate square root of P (Cholesky decomposition)
//  MatrixXd A = P_.llt().matrixL();
//
//  //calculate sigma points ...
//  //set sigma points as columns of matrix Xsig_pred_
//
//  // mean state sigma point (xk|k read "x sub k pipe k")
//  Xsig_pred_.col(0) = x_;
//
//  // next sigma points are "x + sqrt((lambda + n_x) * Pk|k)"
//  MatrixXd B = sqr_lambda * A;
//  for (int i = 1; i <= n_x_; i++) {
//    Xsig_pred_.col(i) = x_ + B.col(i - 1);
//    Xsig_pred_.col(i + n_x_) = x_ - B.col(i - 1);
//  }
//  std::cout << "Xsig = " << std::endl << Xsig_pred_ << std::endl;
//}

#pragma clang diagnostic pop