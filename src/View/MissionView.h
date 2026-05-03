#pragma once
#include "IMissionView.h"

// Stub implementation — rendering logic added in a later task.
class MissionView : public IMissionView {
public:
    void UpdateDronePosition(const Coordinates& position) override;
    void DisplayAlert(const std::string& message) override;
    void GenerateFinalReport(const std::list<IntelData>& finalIntel) override;
};
