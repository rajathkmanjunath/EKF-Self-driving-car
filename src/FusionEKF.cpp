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
          
  Hj_ << 1, 1, 0, 0,
          1, 1, 0, 0,
          1, 1, 1, 1;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  KalmanFilter ekf_;
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
    VectorXd x(4);
    MatrixXd R(2,2), P(4,4), F(4,4), Q(4,4), H(2,4);
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      // cout << "1" << endl;
      x << measurement_pack.raw_measurements_(0),
                measurement_pack.raw_measurements_(1),
                0,
                0;
      // cout << "1" << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Initialize state.
      // cout << "2" << endl;
      float rho = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);
      x <<  rho*cos(theta),
                  rho*sin(theta),
                  rho_dot*cos(theta),
                  rho_dot*sin(theta);
    }
    // cout << "3" << endl;
    x(2) = 5.2;
    x(3) = 1.8 / 1000.0;

    // cout << "3" << endl;

    R << 1, 0,
        0, 1;

    // cout << "3" << endl;

    P << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // cout << "3" << endl;

    H<< 1, 0, 0, 0,
        0, 1, 0, 0;

    // cout << "3" << endl;

    F<< 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // cout << "3" << endl;

    double dt = (measurement_pack.timestamp_-previous_timestamp_)/(1e6), dt_2 = dt*dt, dt_3 = dt_2*dt, dt_4 = dt_3*dt;
    int noise_ax = 9, noise_ay = 9;
    previous_timestamp_ = measurement_pack.timestamp_;

    // cout << "3" << endl;

    Q<< dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

    // cout << "3" << endl;
    
    ekf_.Init(x, P, F, H, R, Q);
    // done initializing, no need to predict or update
    is_initialized_ = true;
    // cout << "Initialization done" << endl;
    return;

  }

  /**
   * Prediction
   */
  double dt = (measurement_pack.timestamp_-previous_timestamp_)/(1e6), dt_2 = dt*dt, dt_3 = dt_2*dt, dt_4 = dt_3*dt;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  int noise_ax = 9, noise_ay = 9;
  
  ekf_.Q_<<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
              
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
    ekf_.x_ << measurement_pack.raw_measurements_(0),
                measurement_pack.raw_measurements_(1),
                0,
                0;

    ekf_.R_ = R_radar_; 
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
  }
  ekf_.Predict();
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
      ekf_.R_ = R_laser_;
      cout << "laser updates" << endl;
      ekf_.Update(measurement_pack.raw_measurements_);
      cout << "Laser update complete" << endl;
    }
    else{
      // Radar updates
        cout << "Radar update" << endl;
      Hj_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      ekf_.R_ = R_radar_;
      cout << "Entering the update function" << endl;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
