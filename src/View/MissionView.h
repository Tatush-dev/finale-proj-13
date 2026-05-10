#pragma once
#include "IMissionView.h"
#include "../Model/OccupancyGrid.h"
#include "../Common/AppEnums.h"
#include <list>
#include <vector>
#include <string>

namespace AIGD {

/**
 * Concrete MVC View: real-time terminal dashboard for the drone mission.
 *
 * Provides:
 *  - ASCII occupancy-grid rendering with drone/target/path overlay
 *  - Formatted telemetry panel (state, battery bar, position)
 *  - Structured event log (tag + message)
 */
class MissionView : public IMissionView {
public:
    // ── IMissionView interface ────────────────────────────────────────────────
    void UpdateDronePosition(const Coordinates& position) override;
    void DisplayAlert(const std::string& message) override;
    void GenerateFinalReport(const std::list<IntelData>& finalIntel) override;

    // ── Task 5.1 — rendering methods ─────────────────────────────────────────

    /**
     * Renders the 2D occupancy grid (capped at 30×30) with overlay markers.
     * Legend: D=Drone  T=Target  *=Path  #=Obstacle  .=Free
     * Prints newlines before the frame to simulate screen refresh.
     * No break or continue used in any loop.
     */
    void displayGrid(const OccupancyGrid& grid,
                     const Coordinates&              dronePos,
                     const Coordinates&              targetPos,
                     const std::vector<Coordinates>& path);

    /**
     * Prints a formatted telemetry panel: FSM state, battery bar, and position.
     */
    void displayTelemetry(double       battery,
                          MissionState state,
                          const Coordinates& currentPos);

    /**
     * Logs a tagged mission event to the console.
     * @param tag     Short category label (e.g. "ALERT", "POS", "EVADE").
     * @param message Human-readable event description.
     */
    void logEvent(const std::string& tag, const std::string& message);

private:
    static std::string stateToString(MissionState state);
};

} // namespace AIGD
