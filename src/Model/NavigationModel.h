#pragma once
#include "INavigationModel.h"

// Stub implementation — A* and D* Lite bodies added in Task 1.2.
class NavigationModel : public INavigationModel {
public:
    Path CalculateInitialPath(const Coordinates& start,
                              const Coordinates& target) override;

    Path UpdateDynamicPath(const Coordinates& currentLocation,
                           const ThreatData&  newThreat) override;
};
