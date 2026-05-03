#pragma once
#include <list>
#include <string>
#include "../Common/Types.h"

/**
 * Abstract interface for the mission view layer.
 * Handles all output: position display, alerts, and final IMINT report.
 */
class IMissionView {
public:
    virtual ~IMissionView() = default;

    // Updates the displayed drone position on the map/HUD.
    virtual void UpdateDronePosition(const Coordinates& position) = 0;

    // Pushes a status or warning message to the operator.
    virtual void DisplayAlert(const std::string& message) = 0;

    // Renders the final IMINT report from accumulated intel.
    virtual void GenerateFinalReport(const std::list<IntelData>& finalIntel) = 0;
};
