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

  
  H_laser_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;
  
  // state covariance matrix P
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
  
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  
  Q_ = MatrixXd(4, 4);
 
  
  noise_ax = 9.0;
  noise_ay = 9.0;
  
 
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
 
 
    // first measurement
    cout << "EKF: " << endl;
    VectorXd x_ = VectorXd(4);
    x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      
      float radar_ro = measurement_pack.raw_measurements_[0];
      float radar_phi = measurement_pack.raw_measurements_[1];
      float radar_ro_dot = measurement_pack.raw_measurements_[2];
 
      // Convert radar from polar to cartesian coordinates
      // Initialize the state vector
      x_ << radar_ro * cos(radar_phi), 
      	    radar_ro * sin(radar_phi),
            radar_ro_dot * sin(radar_phi), 
          	radar_ro_dot * cos(radar_phi);
 
      // Initialize ekf  
      ekf_.Init(x_, P_, F_, Hj_, R_radar_, Q_);
      //cout << "Radar init . x_:" << x_ << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
       // Initialize the state vector
       x_ << measurement_pack.raw_measurements_[0], 
             measurement_pack.raw_measurements_[1], 
             0, 
             0;
       // Initialize ekf 
       ekf_.Init(x_, P_, F_, H_laser_,  R_laser_, Q_);
       //cout << "Laser init x_:" << x_ << endl;
      
    }

   
    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modifying the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Set the process covariance matrix Q
  
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         	  0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         	  dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         	  0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();
  cout << "Predict ok" << endl;
  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Update EKF object with radar R and H
 
    ekf_.R_ = R_radar_; 
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    //cout << "UpdateEKF radar updated" << endl;
  } else {
    // Update EKF object with laser R and H
	ekf_.R_ = R_laser_; 
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
    //cout << "Update laser updated" << endl;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
