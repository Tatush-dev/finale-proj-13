#include "NavigationModel.h"
#include <iostream>

Path NavigationModel::CalculateInitialPath(const Coordinates& start,
                                           const Coordinates& target) {
    std::cout << "[NavigationModel] CalculateInitialPath: A* stub\n";
    // TODO: implement A* over WeightedGraph in Task 1.2
    return { start, target };
}

Path NavigationModel::UpdateDynamicPath(const Coordinates& currentLocation,
                                        const ThreatData&  newThreat) {
    std::cout << "[NavigationModel] UpdateDynamicPath: D* Lite stub\n";
    // TODO: implement D* Lite re-routing in Task 1.2
    return { currentLocation };
}
