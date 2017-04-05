//
// Created by Matthew Zimmer on 3/21/17.
//

#include "LaserMeasurement.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"

LaserMeasurement::LaserMeasurement() {
  R_ = Eigen::MatrixXd(2, 2);
  H_ = Eigen::MatrixXd(2, 4);

  H_ << 1, 0, 0, 0,
          0, 1, 0, 0;

  //measurement covariance matrix - laser
  R_ << 0.0225, 0,
          0, 0.0225;
}

void LaserMeasurement::Update(UKF &ukf) {
//  ukf.Update(this);
//  ukf.UpdateLidar((*this));
}

#pragma clang diagnostic pop