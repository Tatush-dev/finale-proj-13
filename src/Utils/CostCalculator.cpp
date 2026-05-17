#include "CostCalculator.h"

namespace AIGD {

CostCalculator::CostCalculator() {
    // Default to BALANCED priority
    setPriority(MissionPriority::BALANCED);
}

void CostCalculator::setPriority(MissionPriority priority) {
    if (priority == MissionPriority::BALANCED) {
        // Equal distribution: each component gets ~33.3%
        w_distance_ = 0.33;
        w_risk_ = 0.33;
        w_energy_ = 0.34;  // Slight adjustment to sum to 1.0
    } else if (priority == MissionPriority::STEALTH) {
        // High weight on risk factor to minimize detection
        w_distance_ = 0.20;
        w_risk_ = 0.60;
        w_energy_ = 0.20;
    } else if (priority == MissionPriority::AGGRESSIVE) {
        // High weight on distance, lower on risk for speed
        w_distance_ = 0.50;
        w_risk_ = 0.20;
        w_energy_ = 0.30;
    }
}

double CostCalculator::calculateCost(double distance, double riskFactor, double energyConsumption) const {
    return (w_distance_ * distance) + (w_risk_ * riskFactor) + (w_energy_ * energyConsumption);
}

} // namespace AIGD