//
// Created by Matthew Zimmer on 3/21/17.
//

#ifndef EXTENDEDKF_LASER_MEASUREMENT_H
#define EXTENDEDKF_LASER_MEASUREMENT_H


#include "measurement_package.h"

class UKF; // forward declaration to avoid circular dependency compiler errors
class LaserMeasurement : public MeasurementPackage {
public:
    LaserMeasurement();

    virtual ~LaserMeasurement() {};

    void Update(UKF &ukf);
    void ProcessMeasurement(UKF &ukf);

private:
    Eigen::MatrixXd R_;
    Eigen::MatrixXd H_;
};


#endif //EXTENDEDKF_LASER_MEASUREMENT_H
