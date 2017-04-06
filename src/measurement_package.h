#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"
#include "ukf.h"

class UKF; // forward declaration to avoid circular dependency compiler errors
class MeasurementPackage {
public:
    long long timestamp_;

    enum SensorType {
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;

    virtual ~MeasurementPackage() = default;

    virtual void Update(UKF &ukf) = 0;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
