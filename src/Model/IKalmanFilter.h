#pragma once
#include <vector>

/**
 * Abstract interface for the Kalman Filter used in sensor fusion / state estimation.
 */
class IKalmanFilter {
public:
    virtual ~IKalmanFilter() = default;

    // Projects the state forward based on the motion model.
    // motionModel: control input vector (velocity, heading, etc.)
    virtual void Predict(const std::vector<double>& motionModel) = 0;

    // Corrects the prediction with real sensor readings.
    // sensorMeasurements: observation vector from EO/Thermal sensors
    virtual void Update(const std::vector<double>& sensorMeasurements) = 0;
};
