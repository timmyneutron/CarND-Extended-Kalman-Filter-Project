#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

	// initialize rmse
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	for (int i=0; i<estimations.size(); i++) {
		VectorXd residual = estimations[i] - ground_truth[i];
		VectorXd residual_squared = residual.array() * residual.array();
		rmse += residual_squared;
	}

	rmse /= estimations.size();

	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	float px = x_state[0];
	float py = x_state[1];
	float vx = x_state[2];
	float vy = x_state[3];

	MatrixXd Hj(3, 4);

	Hj << 0, 0, 0, 0,
		  0, 0, 0, 0,
		  0, 0, 0, 0;

	if (px * px + py * py > 0.001) {
		Hj(0, 0) = px / sqrt(px * px + py * py);
		Hj(0, 1) = py / sqrt(px * px + py * py);

		Hj(1, 0) = -py / (px * px + py * py);
		Hj(1, 1) = px / (px * px + py * py);

		Hj(2, 0) = py * (vx * py - vy * px) / pow(px * px + py * py, 1.5);
		Hj(2, 1) = px * (vy * px - vx * py) / pow(px * px + py * py, 1.5);
		Hj(2, 2) = px / sqrt(px * px + py * py);
		Hj(2, 3) = py / sqrt(px * px + py * py);
	} else {
		Hj << 100000, 100000, 0, 0,
			  100000, 100000, 0, 0,
			  100000, 100000, 100000, 100000;
	}

	return Hj;
}
