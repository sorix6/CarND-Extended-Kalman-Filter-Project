#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * predict the state
  */

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  
  UpdateState(y);

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // h(xprime) = (rho, phi, rhodot).transpose()

  double row1 = sqrt(pow(x_(0), 2) + pow(x_(1), 2));
  double row2 = atan2(x_(1),x_(0));
  double row3 = ((x_(0) * x_(2)) + (x_(1) * x_(3))) / row1;
  VectorXd hprime = VectorXd(3);
  hprime << row1, row2, row3;

  // y = z - h(xprime)
  VectorXd y = z - hprime;

  // the Kalman filter is expecting small angle values between the range -pi and pi. 
  // add 2π or subtract 2π until the angle is within the desired range.

  while (abs(y(1)) > M_PI){
    if (y(1) > M_PI){
      y(1) -= 2 * M_PI;
    }
    else{
      y(1) += 2 * M_PI;
    }
  }

  UpdateState(y);
}

void KalmanFilter::UpdateState(const VectorXd &y) {
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  P_ = (MatrixXd::Identity(x_size, x_size) - K * H_) * P_;

}

