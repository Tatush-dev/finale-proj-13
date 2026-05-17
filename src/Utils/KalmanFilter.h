#pragma once
#include "Model/IKalmanFilter.h"

namespace AIGD {

/**
 * 1D Kalman Filter for sensor fusion / state estimation.
 *
 * Process model (constant-displacement):
 *   x_k = x_{k-1} + u_k          (F=1, B=1)
 *   P_k = P_{k-1} + Q
 *
 * Observation model (direct position read):
 *   z_k = x_k + noise             (H=1)
 *
 * motionModel vector layout  : [0] = control input u (expected displacement)
 * sensorMeasurements layout  : [0] = raw scalar position measurement
 */
class KalmanFilter : public IKalmanFilter {
public:
    /**
     * @param initialState      Initial position estimate (x_0).
     * @param initialP          Initial error covariance (uncertainty).
     * @param processNoise      Q — variance introduced by unmodelled dynamics.
     * @param measurementNoise  R — variance of the sensor reading.
     */
    KalmanFilter(double initialState, double initialP,
                 double processNoise, double measurementNoise);

    // Projects the state forward: x = x + u,  P = P + Q
    void Predict(const std::vector<double>& motionModel) override;

    // Corrects with a new sensor reading via the standard K gain update
    void Update(const std::vector<double>& sensorMeasurements) override;

    double getState()      const;
    double getCovariance() const;

private:
    double m_x;  // state estimate (position)
    double m_P;  // error covariance
    double m_Q;  // process noise covariance
    double m_R;  // measurement noise covariance
};

} // namespace AIGD
