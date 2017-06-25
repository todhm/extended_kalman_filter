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
    x_ = F_ * x_ ;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ +(K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    
    //prevent the case where px and py are execessively small
    if(fabs(px) < 0.0001){
        px = 0.0001;
    }
    if(fabs(py) < 0.0001){
        py  = 0.0001;
    }
    
    
    

    VectorXd hx = VectorXd(3);
    
    float rho = sqrt(pow(px,2.0) + pow(py,2.0));
    if(fabs(rho) < 0.0001){
        rho = 0.0001;
    }
    float phi = atan2(py,px);
    float rho_dot = (px*vx + py *vy) /rho;

    hx << rho, phi, rho_dot;
    VectorXd y = z - hx;
    
    //normalize angle between [-PI, PI]
    y[1] -= (2 * M_PI) * floor((y[1] + M_PI) / (2 * M_PI));
    
    
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ +(K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_) * P_;

}
