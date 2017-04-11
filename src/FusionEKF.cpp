#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  // H matrix for laser measurements
  H_laser_ << 1, 0, 0, 0,
  			  0, 1, 0, 0;

  // F matrix (initialize with dt = 0)
  ekf_.F_ = MatrixXd::Identity(4, 4);

  // P matrix (initialize with high initial uncertainties)
  MatrixXd P(4, 4);
  P << 1000, 0, 0, 0,
       0, 1000, 0, 0,
       0, 0, 10000, 0,
       0, 0, 0, 10000;
  ekf_.P_ = P;

  // Q matrix (initialize as zeros)
  ekf_.Q_ = MatrixXd::Zero(4, 4);

  // set acceleration noise
  noise_ax = 9.0;
  noise_ay = 9.0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;

    // initialize state vector as 1, 1, 0, 0
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // if first measurement is from radar

      // define input variables
    	float rho = measurement_pack.raw_measurements_(0);
    	float phi = measurement_pack.raw_measurements_(1);

      // initialize state from radar rho and phi
      // px = rho * cos(phi)
    	ekf_.x_(0) = rho * cos(phi);

      // py = rho * sin(phi)
    	ekf_.x_(1) = rho * sin(phi);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // if first measurement is from laser

      // initialize state from laser x and y
      // px = x
    	ekf_.x_(0) = measurement_pack.raw_measurements_(0);

      // py = y
    	ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    // initialize timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // define dt (change in time since last measurement)
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  float dt_2 = pow(dt, 2);
  float dt_3 = pow(dt, 3);
  float dt_4 = pow(dt, 4);

  // update timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  // update F matrix with new dt
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // update Q matrix
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
