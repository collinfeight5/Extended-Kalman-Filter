#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

//using namespace std;
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

  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

// reference measurement_pack func param has meas_pack data from main
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /**
   *  Initialization Step
   */

  if (!is_initialized_) { //Note timestep == 0 here
    // first meas. here
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    //Create covariance state matrix 
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;
    
    //transition matrix
    ekf_.F_ = MatrixXd(4,4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

    //Radar data grab
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);
      
      // polar to cartesian conversion:
      // r * cos(angle) for x 
      // r * sin(angle) for y
      //ekf_.x_ == Xx, Vy, doubleVx, doubleVy
      ekf_.x_ << rho * cos(phi), rho * sin(phi), rho_dot * cos(phi), rho_dot * sin(phi);
    }
    // Lasar data grab
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0.0, 0.0;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // set is_initialized to true, done initializing
    is_initialized_ = true;

    // Do not Need return statement since function is type void?
    return;
  }

  /**
   *  Prediction Step
  */

  //compute the time elapsed between the current and previous measurements
  float delta_time = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//delta_time in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float delta_time_2 = delta_time * delta_time;
  float delta_time_3 = delta_time_2 * delta_time;
  float delta_time_4 = delta_time_3 * delta_time;
  
  //Integrate time to F matrix
  ekf_.F_(0, 2) = delta_time;
  ekf_.F_(1, 3) = delta_time;
  
  //process noise
  int noise_ax = 9;
  int noise_ay = 9;
  
  //process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << delta_time_4/4*noise_ax, 0, delta_time_3/2*noise_ax, 0,
            0, delta_time_4/4*noise_ay, 0, delta_time_3/2*noise_ay,
            delta_time_3/2*noise_ax, 0, delta_time_2*noise_ax, 0,
            0, delta_time_3/2*noise_ay, 0, delta_time_2*noise_ay;
  
  //call predict function
  ekf_.Predict();

  /**
   *  Update Step
   */
  // Radar updates
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    Tools tools;
    //Meas Matrix
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    //Meas. Covariance Matrix
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }

  //Laser Updates
   else {
     //Meas. Matrix
    ekf_.H_ = H_laser_;
    //Meas. Covariance Matrix
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}