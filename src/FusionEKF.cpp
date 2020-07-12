#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  KalmanFilter ekf_;
  // MatrixXd ekf_.P_(4,4);
  // VectorXd ekf_.x_(4);
  // MatrixXd ekf_.R_(2,2);
  // MatrixXd ekf_.Q_(4,4);
  // MatrixXd ekf_.H_(2,4);
  // MatrixXd ekf_.F_(4,4);

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    // first measurement
    // cout << "EKF: " << endl;
    // ekf_.x_ = VectorXd(4);
    // ekf_.x_ << 1, 1, 1, 1;
    VectorXd x(4);
    MatrixXd R(2,2), P(4,4), F(4,4), Q(4,4), H(2,4);
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      x << measurement_pack.raw_measurements_(0),
                measurement_pack.raw_measurements_(1),
                0,
                0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Initialize state.
      float rho = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);
      x <<  rho*cos(theta),
                  rho*sin(theta),
                  rho_dot*cos(theta),
                  rho_dot*sin(theta);
    }
    R << 1, 0,
          0, 1;

    P << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    H<< 1, 0, 0, 0,
              0, 1, 0, 0;

    F<< 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;

    double dt = (measurement_pack.timestamp_-previous_timestamp_)/(1e6);
    double dt_2 = dt*dt, dt_3 = dt_2*dt, dt_4 = dt_3*dt;
    int noise_ax = 9, noise_ay = 9;

    Q<<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
                0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
                dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
                0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
    previous_timestamp_ = measurement_pack.timestamp_/(1e6);

    ekf_.Init(x, P, F, H, R, Q);
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  double dt = (measurement_pack.timestamp_-previous_timestamp_)/(1e6), dt_2 = dt*dt, dt_3 = dt_2*dt, dt_4 = dt_3*dt;
  cout << "prediction step" << endl;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  int noise_ax = 9, noise_ay = 9;
  ekf_.Q_<<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  cout << "F and Q initialized" << endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
    ekf_.x_ << measurement_pack.raw_measurements_(0),
                measurement_pack.raw_measurements_(1),
                0,
                0;

      ekf_.R_ = R_radar_;
    cout << "Laser states initalized" << endl;
  }
  else{
    float rho = measurement_pack.raw_measurements_(0);
    float theta = measurement_pack.raw_measurements_(1);
    float rho_dot = measurement_pack.raw_measurements_(2);
    ekf_.x_ <<  rho*cos(theta),
                rho*sin(theta),
                rho_dot*cos(theta),
                rho_dot*sin(theta);

    ekf_.R_ = R_laser_;
    cout << "Radar states intialized" << endl;
  }
  ekf_.Predict();
  cout << "Out from EKF prediction step" << endl;
  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
    
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
    if(measurement_pack.sensor_type_ == MeasurementPackage::LASER){
      // Laser updates
      ekf_.Update(measurement_pack.raw_measurements_);
      cout << "Laser measurement done" << endl;
    }
    else{
      // Radar updates
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
      cout << "Radar measurement done" << endl;
    }
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
