#pragma once
#include "../Common/Types.h"

/**
 * Abstract interface for navigation logic.
 * Concrete implementations may use A* (initial path) and D* Lite (dynamic re-routing).
 */
class INavigationModel {
public:
    virtual ~INavigationModel() = default;

    // Calculates the optimal path from start to target using A*.
    virtual Path CalculateInitialPath(const Coordinates& start,
                                      const Coordinates& target) = 0;

    // Re-routes around a newly detected threat using D* Lite.
    virtual Path UpdateDynamicPath(const Coordinates& currentLocation,
                                   const ThreatData&  newThreat) = 0;
};
