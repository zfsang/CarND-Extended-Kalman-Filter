#include "tools.h"
#include <iostream>
using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    if (estimations.size() != ground_truth.size() || estimations.size() ==0){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    };
    for (int i=0; i<estimations.size(); i++) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    };
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

    MatrixXd Hj(3,4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // TODO: YOUR CODE HERE
    float s = px * px + py * py;
    // check division by zero
    if (s < 0.00001) {

        cout<< "CalculateJacobian() - Error - Division By Zero" << endl;
    } else {
        Hj << px / sqrt(s),  py / sqrt(s), 0, 0,
                - py / s,   px / s, 0, 0,
                (py * (vx * py - vy * px)) / sqrt(s*s*s) , (px * (vy * px - vx * py)) / sqrt(s*s*s), px / sqrt(s), py / sqrt(s);
    };
    // compute the Jacobian matrix

    return Hj;
}
