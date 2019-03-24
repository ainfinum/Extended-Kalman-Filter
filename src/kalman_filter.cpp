#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

# define M_PI           3.14159265358979  /* pi */
# define T_PI           6.28318530717958  /* 2 x pi */


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
  // Predict the state by using Kalman Filter equations
  x_ = F_ * x_;
  //cout << "Predict x_ " << x_ << endl;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  //cout << "Predict P_" << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  // Update the state
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  
  UpdateKFCommon(y);
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
   // Update the state by using Extended Kalman Filter equations
   
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float c1 = px*px + py*py;
  float ro = sqrt(c1);
  // check division by zero
  if (fabs(ro) < 0.0001) {
    cout << "UpdateEKF () - Error - Division by Zero" << endl;
    ro = 0.0001;
  }
  
  float phi = atan2(py, px);
  float ro_dot = ((px * vx) + (py * vy))/ro;
  
  VectorXd h_x = VectorXd(3);
  h_x << ro, phi, ro_dot;
  
  VectorXd y = z - h_x;
  
  // Normalizing Angles
  if(y(1) < -M_PI) y(1) += T_PI;
  else if (y(1) > M_PI) y(1) -= T_PI;

  UpdateKFCommon(y);
  
}

void KalmanFilter::UpdateKFCommon(const VectorXd &y) {
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}
