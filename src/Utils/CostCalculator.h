#ifndef COST_CALCULATOR_H
#define COST_CALCULATOR_H

/**
 * @file CostCalculator.h
 * @brief Dynamic cost function engine for mission path planning.
 *
 * Evaluates movement costs based on mission priorities, balancing distance,
 * risk factor, and energy consumption.
 */

#include "../Common/AppEnums.h"

namespace AIGD {

/**
 * @class CostCalculator
 * @brief Calculates movement costs based on configurable mission priorities.
 *
 * Supports three priority profiles:
 * - BALANCED: Equal weights for distance, risk, and energy
 * - STEALTH: High weight on risk factor to minimize detection
 * - AGGRESSIVE: High weight on distance, lower on risk for speed
 */
class CostCalculator {
public:
    /**
     * @brief Constructs a CostCalculator with default BALANCED priority.
     */
    CostCalculator();

    /**
     * @brief Sets the mission priority profile and adjusts internal weights.
     * @param priority The mission priority to apply.
     */
    void setPriority(MissionPriority priority);

    /**
     * @brief Calculates the total cost of movement.
     * @param distance The distance to travel (in meters).
     * @param riskFactor The risk factor (0.0 to 1.0, higher is riskier).
     * @param energyConsumption The energy consumption (in arbitrary units).
     * @return The calculated total cost.
     */
    double calculateCost(double distance, double riskFactor, double energyConsumption) const;

private:
    double w_distance_;     ///< Weight for distance component
    double w_risk_;         ///< Weight for risk factor component
    double w_energy_;       ///< Weight for energy consumption component
};

} // namespace AIGD

#endif // COST_CALCULATOR_H