#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	//  * the estimation vector size should not be zero
	//  * and should equal ground truth vector size
	if ((estimations.size() <= 0) || (estimations.size() != ground_truth.size())) {
	    cout << "Error: Estimation vector size is invalid.\n";
	    return rmse;
	}

	//accumulate squared residuals
	for(size_t i=0; i < estimations.size(); ++i){
        
        VectorXd residuals = estimations[i] - ground_truth[i];

	    //calculate the mean
        VectorXd r = residuals.array() * residuals.array();
        rmse += r;
	}
		
	//calculate the squared root
	rmse = rmse/estimations.size();
	rmse = rmse.array().sqrt();
	
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj_(3, 4);

    //recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	if ((pow(px,2) + pow(py,2)) < 0.0001) {
		cout << "Error: division by zero\n";
	    return Hj_;
	}
	
	//compute the Jacobian matrix
	float hj11 = px / pow(pow(px,2) + pow(py,2),0.5);
	float hj12 = py / pow(pow(px,2) + pow(py,2),0.5);
	float hj21 = -py / (pow(px,2) + pow(py,2));
	float hj22 = px / (pow(px,2) + pow(py,2));
	float hj31 = py*(vx*py - vy*px) / pow(pow(px,2) + pow(py,2),1.5);
	float hj32 = px*(vy*px - vx*py) / pow(pow(px,2) + pow(py,2),1.5);

    Hj_ <<   hj11, hj12, 0, 0,
            hj21, hj22, 0, 0,
            hj31, hj32, hj11, hj12;

	return Hj_;
}
