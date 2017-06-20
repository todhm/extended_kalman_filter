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


    if (estimations.size() <= 0 or estimations.size() != ground_truth.size()){
        cout << "invalid estimation size"<<endl;
        return rmse;
    }
    else{

	for(int i=0; i < estimations.size(); ++i){
           VectorXd a =  estimations[i] - ground_truth[i];
           VectorXd b = a.array() * a.array();
           rmse = rmse + b;


	}
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;

    }

	return rmse;

  /**
  TODO:
    * Calculate the RMSE here.
  */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj = MatrixXd::Zero(3,4) ;
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    if(px==0 or py ==0 ){
        cout<<"invalid position"<<endl;
        return Hj;
    }
    else{
        float vsum = vx*py - vy*px;
        float rootsum = pow((pow(px, 2.0) + pow(py, 2.0)), 0.5);
        Hj << px/ rootsum, py/rootsum, 0, 0,
              (-1) * py/ pow(rootsum,2.0), px / pow(rootsum, 2.0), 0, 0,
              py * vsum/ pow(rootsum,3),px * (-1) * vsum / pow(rootsum,3), px/rootsum, py/rootsum;
        return Hj;


    }

}
