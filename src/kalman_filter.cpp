#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  // state prediction
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Kalman Filter update
	MatrixXd H_trans = H_.transpose();

	VectorXd y = z - H_ * x_;
	MatrixXd S = H_ * P_ * H_trans + R_;
	MatrixXd K = P_ * H_trans * S.inverse();
	MatrixXd I = MatrixXd::Identity(4, 4);

	x_ = x_ + K * y;
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // extended Kalman Filter update

  // define state variables
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];

  // define h(x) vector
	VectorXd h(3);

  // rho
	h(0) = sqrt(px * px + py * py);

  // phi
	h(1) = atan2(py, px);

  // rho dot (if dividing by 0, just set to 100,000)
  if (px * px + py * py > .001) {
    h(2) = (px * vx + py * vy) / sqrt(px * px + py * py);
  } else {
    h(2) = 100000;
  }

  // define Jacobian matrix Hj
	MatrixXd Hj = tools.CalculateJacobian(x_);

  // perform update
	MatrixXd Hj_trans = Hj.transpose();
	
	VectorXd y = z - h;
	MatrixXd S = Hj * P_ * Hj_trans + R_;
	MatrixXd K = P_ * Hj_trans * S.inverse();
	MatrixXd I = MatrixXd::Identity(4, 4);

	x_ = x_ + K * y;
	P_ = (I - K * Hj) * P_;
}
