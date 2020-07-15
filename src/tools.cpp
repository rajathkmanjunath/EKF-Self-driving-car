#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if(estimations.size() != ground_truth.size()){
     cout << "Sizes  of estimations and ground truths don't match" << endl;
     return rmse;
  }
   // cout << "Matrix dimensionality check" << endl;
   // cout << estimations.size() << " " << ground_truth.size() << endl;
  for(int i=0; i<(int)estimations.size(); i++){
   //   cout << estimations[i] << " " << ground_truth[i] << endl;
     VectorXd res = estimations[i] - ground_truth[i];
     res = res.array()*res.array();
     rmse+=res;
  }
//   cout << "Dimensions matching" << endl;
//   cout << "RMSE returned" << endl;
return rmse/estimations.size();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   double px = x_state[0];
   double py = x_state[1];
   double vx = x_state[2];
   double vy = x_state[3];

   // Squares taken to avoid repeat calculations
   double px_2 = px * px;
   double py_2 = py * py;
   double squareAdd = px_2 + py_2;
   double squareAddRoot = sqrt(squareAdd);
   double cubeRoot = squareAddRoot * squareAdd;

   MatrixXd Jacobian = MatrixXd(3, 4);

   //check division by zero
   if (fabs(squareAdd) < 0.0001)
      return Jacobian;

   //compute the Jacobian matrix
   Jacobian << px / squareAddRoot, py / squareAddRoot, 0, 0,
         -py / squareAdd, px / squareAdd, 0, 0,
         py * (vx * py - vy * px) / cubeRoot, px * (vy * px - vx * py) / cubeRoot, px / squareAddRoot, py /
                                                                                                         squareAddRoot;
   return Jacobian;

}
