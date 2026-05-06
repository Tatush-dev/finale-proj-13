#include "Utils/KalmanFilter.h"
#include <stdexcept>

namespace AIGD {

KalmanFilter::KalmanFilter(double initialState, double initialP,
                           double processNoise, double measurementNoise)
    : m_x(initialState), m_P(initialP), m_Q(processNoise), m_R(measurementNoise)
{}

void KalmanFilter::Predict(const std::vector<double>& motionModel) {
    if (motionModel.empty()) {
        throw std::invalid_argument("motionModel must contain the control input u at index 0");
    }

    const double u = motionModel[0];  // expected displacement this step

    // State prediction:      x = F*x + B*u  (F=1, B=1)
    m_x = m_x + u;

    // Covariance prediction: P = F*P*F^T + Q  (F=1)
    m_P = m_P + m_Q;
}

void KalmanFilter::Update(const std::vector<double>& sensorMeasurements) {
    if (sensorMeasurements.empty()) {
        throw std::invalid_argument("sensorMeasurements must contain a measurement at index 0");
    }

    const double z = sensorMeasurements[0];  // observation (H=1)

    // Innovation:             y = z - H*x  =  z - x
    const double y = z - m_x;

    // Innovation covariance:  S = H*P*H^T + R  =  P + R
    const double S = m_P + m_R;

    // Kalman gain:            K = P*H^T / S  =  P / S
    const double K = m_P / S;

    // State update:           x = x + K*y
    m_x = m_x + K * y;

    // Covariance update:      P = (1 - K*H)*P  =  (1-K)*P
    m_P = (1.0 - K) * m_P;
}

double KalmanFilter::getState() const {
    return m_x;
}

double KalmanFilter::getCovariance() const {
    return m_P;
}

} // namespace AIGD
