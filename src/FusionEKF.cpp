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
    
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
    
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
               0, 0.0009, 0,
               0, 0, 0.09;
  //measurement matrix -laser
  H_laser_ << 1,0,0,0,
              0,1,0,0;
    

  // Initialize Measuremnet Function for Radar sensor
  Hj_    = MatrixXd::Zero(3, 4);
    

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
      previous_timestamp_ = measurement_pack.timestamp_;
      
      
      //Initalize state
      ekf_.x_ = VectorXd::Ones(4);

      // Initialize State Covariance
      ekf_.P_  = MatrixXd(4, 4);
      ekf_.P_ << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1000, 0,
                 0, 0, 0, 1000;
      
      
      // Initialize State transition matrix
      ekf_.F_ = MatrixXd::Zero(4,4);
      
      //Initialize Process-covariance matrix
      ekf_.Q_ = MatrixXd::Zero(4,4);
      
      
      
      
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

       float px;
       float py;
       float rho  = measurement_pack.raw_measurements_[0];
       float phi = measurement_pack.raw_measurements_[1];
       px = rho * cos(phi);
       py = rho * sin(phi);
       ekf_.x_ << px, py, 0, 0;


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],0,0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
   float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0; ;
   previous_timestamp_ = measurement_pack.timestamp_;

   ekf_.F_ << 1, 0 , dt,0,
              0, 1 , 0 ,dt,
              0, 0, 1, 0 ,
              0, 0, 0, 1;

  float ax =9.0;
  float ay =9.0;
  //Update process covariance matrix with time differential
  ekf_.Q_ << pow(dt,4)/4 * ax, 0, pow(dt,3) /2 * ax, 0,
             0, pow(dt,4)/4 * ay, 0, pow(dt,3) /2 *ay,
             ax * pow(dt,3)/2 , 0, ax *pow(dt,2), 0 ,
             0, pow(dt,3)/2 * ay, 0, pow(dt,2) * ay;

  ekf_.Predict();


  /*****************************************************************************
   *  Update
   ****************************************************************************/
  // Radar update
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);


    // laser updates
  } else {
    
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_ ;
      ekf_.Update(measurement_pack.raw_measurements_);
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
