#include "MissionView.h"
#include <iostream>

void MissionView::UpdateDronePosition(const Coordinates& position) {
    std::cout << "[MissionView] Drone at ("
              << position.x << ", " << position.y
              << ", alt=" << position.altitude << ")\n";
}

void MissionView::DisplayAlert(const std::string& message) {
    std::cout << "[MissionView] ALERT: " << message << "\n";
}

void MissionView::GenerateFinalReport(const std::list<IntelData>& finalIntel) {
    std::cout << "[MissionView] === FINAL IMINT REPORT ===\n";
    for (const auto& intel : finalIntel) {
        std::cout << "  [" << intel.id << "] " << intel.description
                  << " @ (" << intel.location.x << ", " << intel.location.y << ")\n";
    }
    std::cout << "[MissionView] === END OF REPORT ===\n";
}
