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

  for(int i=0; i<estimations.size(); i++){
     VectorXd res = estimations[i] - ground_truth[i];
     rmse+=res*res;
  }
  cout << "RMSE returned" << endl;
return rmse/estimations.size();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
