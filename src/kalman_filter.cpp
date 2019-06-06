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
//full eq --> x_ = F_*x_ + B(Control input)*mu(Control Vector) + v(Process noise)
//B*mu above is external forces, considered zero here
  x_ = F_ * x_; 
  MatrixXd F_trans ;
  F_trans = F_.transpose();
  // P is prediced covariance
  P_ = F_ * P_ * F_trans + Q_;
}

//Standard Kalman Filter Equations -> Lidar(Laser)
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_predict = H_ * x_; 
  VectorXd y = z - z_predict; //diff b/t measured value and predicted value
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; //total error from measured value, R(known) and predicted value
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si; //Kalman Gain
    
  //new guess
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

//Extended Kalman Filter Equations --> Radar
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //state params
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
    
  // Equations for H_Function below
  float rho = sqrt(px * px + py * py);

  ///check division by zero
  // If rho == 0, skip the update step to avoid dividing by zero.
  /*
  if( px == 0. && py == 0. )
    return;
  */
	//better results
  if(rho < .00001) {
    px += .001;
    py += .001;
    rho = sqrt(px * px + py * py);
  }
  
  float rho2 = atan2(py,px);
  float rho3 = (px*vx+py*vy)/rho;
    
  VectorXd H_Function(3);
  H_Function << rho, rho2, rho3;
  
  //Normal Kalman Filter eqns
  VectorXd y = z - H_Function;
  // Normalize angle 
  while (y(1)>M_PI) {
    y(1) -= 2 * M_PI;
  }
  while (y(1)<-M_PI) {
    y(1) += 2 * M_PI;
  }
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
    
  //new guess
  x_ = x_ + (K * y);
  long x_size = x_.size();
  //x_size == 4
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}