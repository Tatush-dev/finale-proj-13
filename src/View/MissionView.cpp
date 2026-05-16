#include "MissionView.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <algorithm>

// Width (in characters) of the battery bar rendered in the telemetry panel.
// Defined here so the display constant is visible to the entire translation unit.
#define TELEMETRY_BAR_WIDTH 20

namespace AIGD {

// ── IMissionView overrides ────────────────────────────────────────────────────

void MissionView::UpdateDronePosition(const Coordinates& position)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1)
        << "(" << position.x << ", " << position.y
        << ", alt=" << position.altitude << ")";
    logEvent("POS", "Drone at " + oss.str());
}

void MissionView::DisplayAlert(const std::string& message)
{
    logEvent("ALERT", message);
}

void MissionView::GenerateFinalReport(const std::list<IntelData>& finalIntel)
{
    std::cout << "\n+============================+\n";
    std::cout << "|    FINAL IMINT REPORT      |\n";
    std::cout << "+============================+\n";
    std::cout << std::fixed << std::setprecision(1);
    for (const auto& intel : finalIntel) {
        std::cout << "  @ ("
                  << intel.getLocationX() << ", "
                  << intel.getLocationY() << ")  ["
                  << intel.getSensorTypeString() << "]  "
                  << intel.getDataSize() << " B\n";
    }
    std::cout << "+============================+\n\n";
}

// ── displayGrid ───────────────────────────────────────────────────────────────
// Renders a real-time ASCII snapshot of the occupancy grid (capped at 30×30)
// with four overlay layers applied in priority order:
//   1. Obstacle cells  '#'  (OccupancyGrid.IsOccupied)
//   2. Path waypoints  '*'  (active m_currentPath)
//   3. Target cell     'T'  (overwrites path marker if coincident)
//   4. Drone position  'D'  (highest priority — overwrites all other markers)
//
// Rows are rendered from top (high y) to bottom (y=0) so the grid orientation
// matches standard (x=right, y=up) world coordinates.
// No break or continue is used in any loop.
void MissionView::displayGrid(const OccupancyGrid&             grid,
                               const Coordinates&               dronePos,
                               const Coordinates&               targetPos,
                               const std::vector<Coordinates>&  path)
{
    const int W = std::min(grid.GetWidth(),  30);
    const int H = std::min(grid.GetLength(), 30);

    // Simulate frame refresh by scrolling the previous frame out of view.
    for (int i = 0; i < 25; ++i) std::cout << '\n';

    // Build character display buffer, default to free space '.'.
    std::vector<std::vector<char>> display(H, std::vector<char>(W, '.'));

    // Pass 1 – mark occupied cells as obstacles '#'.
    for (int row = 0; row < H; ++row) {
        for (int col = 0; col < W; ++col) {
            if (grid.IsOccupied(col, row)) {
                display[row][col] = '#';
            }
        }
    }

    // Pass 2 – mark active path waypoints '*'.
    for (int i = 0; i < static_cast<int>(path.size()); ++i) {
        int px = static_cast<int>(path[i].x);
        int py = static_cast<int>(path[i].y);
        if (px >= 0 && px < W && py >= 0 && py < H) {
            display[py][px] = '*';
        }
    }

    // Pass 3 – mark target 'T' (overwrites path marker if coincident).
    int tx = static_cast<int>(targetPos.x);
    int ty = static_cast<int>(targetPos.y);
    if (tx >= 0 && tx < W && ty >= 0 && ty < H) {
        display[ty][tx] = 'T';
    }

    // Pass 4 – mark drone 'D' at highest priority (overwrites all other markers).
    int dx = static_cast<int>(dronePos.x);
    int dy = static_cast<int>(dronePos.y);
    if (dx >= 0 && dx < W && dy >= 0 && dy < H) {
        display[dy][dx] = 'D';
    }

    // Render header legend.
    std::cout << "+-- GRID (" << W << "x" << H << ")"
              << "  [D=Drone  T=Target  *=Path  #=Obstacle  .=Free] --+\n";

    // Render rows from top (high y) to bottom (y = 0) — no break/continue.
    for (int row = H - 1; row >= 0; --row) {
        std::cout << std::setw(3) << row << " |";
        for (int col = 0; col < W; ++col) {
            std::cout << display[row][col] << ' ';
        }
        std::cout << "|\n";
    }

    // X-axis index row.
    std::cout << "     ";
    for (int col = 0; col < W; ++col) {
        std::cout << (col % 10) << ' ';
    }
    std::cout << '\n';
}

// ── displayTelemetry ──────────────────────────────────────────────────────────
// Prints a formatted telemetry panel showing the current FSM state, a
// proportional battery bar (TELEMETRY_BAR_WIDTH characters wide), and
// the drone's grid coordinates and altitude.
void MissionView::displayTelemetry(double             battery,
                                    MissionState        state,
                                    const Coordinates&  currentPos)
{
    std::string stateStr = stateToString(state);

    // Battery bar: TELEMETRY_BAR_WIDTH characters wide, proportional fill.
    int filled = static_cast<int>(battery / 100.0 * TELEMETRY_BAR_WIDTH);
    int empty  = TELEMETRY_BAR_WIDTH - filled;

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "\n======= TELEMETRY =======\n";
    std::cout << "  State   : " << stateStr << "\n";
    std::cout << "  Battery : [";
    for (int i = 0; i < filled; ++i) std::cout << '=';
    for (int i = 0; i < empty;  ++i) std::cout << ' ';
    std::cout << "] " << battery << "%\n";
    std::cout << "  Position: ("
              << currentPos.x << ", "
              << currentPos.y
              << ", alt=" << currentPos.altitude << ")\n";
    std::cout << "=========================\n";
}

// ── logEvent ──────────────────────────────────────────────────────────────────

void MissionView::logEvent(const std::string& tag, const std::string& message)
{
    std::cout << "[" << std::left << std::setw(8) << tag << "] " << message << "\n";
}

// ── private helpers ───────────────────────────────────────────────────────────

// Maps a MissionState enum value to its human-readable name string.
// Single-return form: result is initialised to "UNKNOWN" (safe default for
// any future enum value not yet covered by the else-if chain).
std::string MissionView::stateToString(MissionState state)
{
    std::string result = "UNKNOWN";
    if      (state == MissionState::IDLE)     { result = "IDLE"; }
    else if (state == MissionState::OUTBOUND) { result = "OUTBOUND"; }
    else if (state == MissionState::RECON)    { result = "RECON"; }
    else if (state == MissionState::EVADE)    { result = "EVADE"; }
    else if (state == MissionState::RETURN)   { result = "RETURN"; }
    else if (state == MissionState::LANDED)   { result = "LANDED"; }
    return result;
}

} // namespace AIGD
