#include <iostream>
#include "tools.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "TemplateArgumentsIssues"
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

Eigen::VectorXd Tools::CalculateRMSE(const vector<Eigen::VectorXd> &estimations,
                                     const vector<Eigen::VectorXd> &ground_truth) {
  Eigen::VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  int n1 = estimations.size();
  int n2 = ground_truth.size();

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if((n1+n2 == 0) || (n1 != n2)) {
    std::cout << "estimations and ground truth vectors must not be empty and must be of equal dimensions." << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  Eigen::VectorXd residual;
  for (int i = 0; i < n1; ++i) {
    std::cout << "estimations[i] = " << std::endl << estimations[i] << std::endl;
    std::cout << "ground_truth[i] = " << std::endl << ground_truth[i] << std::endl << std::endl << "****************************" << std::endl << std::endl;
    residual = estimations[i]-ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  rmse = rmse.array()/n1; //calculates the mean
  rmse = rmse.array().sqrt(); //calculates the squared root
  return rmse;

}

#pragma clang diagnostic pop