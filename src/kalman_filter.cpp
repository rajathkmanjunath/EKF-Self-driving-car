#include "kalman_filter.h"
#include<iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

VectorXd x_;
MatrixXd P_, F_, H_, R_, Q_;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  //cout << "Predict function called " << endl;
  //cout << "x" << endl;
  //cout << x_ << endl;
  //cout << "P" << endl;
  //cout << P_  << endl;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd pHt = P_*Ht;
  MatrixXd K = pHt*Si;

  x_ = x_ + (K* y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I-K*H_)*P_;
  /*cout << "Laser update" << endl;
  cout << "x" << endl;
  cout << x_ << endl;
  cout << "P" << endl;
  cout << P_ << endl;*/
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd h = VectorXd(3);
  h << sqrt(x_(0) * x_(0) + x_(1) * x_(1)),
        atan2(x_(1), x_(0)),
        (x_(0) * x_(2) + x_(1) * x_(3)) / (sqrt(x_(0) * x_(0) + x_(1) * x_(1)));

  /*cout << "update Radar update" << endl;
  cout << "h" << endl;
  cout << h << endl;*/
  VectorXd y = z - h;
  y(1) = atan2(sin(y(1)), cos(y(1)));
  /*cout << "y" << endl;
  cout << y << endl;
  cout << "Hj" << endl;
  cout << H_ << endl;*/
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  /*cout << "Si" << endl;
  cout << Si << endl;*/
  MatrixXd pHt = P_*Ht;
  MatrixXd K = pHt*Si;
  /*cout << K << endl;
  cout << y << endl;*/

  x_ = x_ + (K* y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I-K*H_)*P_;
  /*cout << "x" << endl;
  cout << x_ << endl;
  cout << "P" << endl;
  cout << P_ << endl;*/

}
