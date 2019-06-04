#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;
    
  unsigned int i;
  for(i=0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
    
  // mean
  rmse = rmse / estimations.size();
  //mean square rooted
  rmse = rmse.array().sqrt();    
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    
  MatrixXd Hj(3,4);
    
  //state params
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float rho = px * px + py * py;
 
  //check division by zero
  /*
  if( px == 0 && py == 0 ){
    cout << "Error; div by zero in tools.cpp Jac matrix" << endl;
    return Hj;
  }
  */
  if(rho < .00001) {
    px += .001;
    py += .001;
    rho = px * px + py * py;
  }
  float rho2 = sqrt(rho);
  float rho3 = rho * rho2;
    
  //Jacobian matrix
  Hj << px/rho2, py/rho2, 0, 0,
        -py/rho, px/rho, 0, 0,
        (py*(vx*py - vy*px))/rho3, (px*(vy*px - vx*py))/rho3, px/rho2, py/rho2;
  
  return Hj;
}
