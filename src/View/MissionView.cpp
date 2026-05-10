#include "MissionView.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <algorithm>

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

void MissionView::displayGrid(const OccupancyGrid&             grid,
                               const Coordinates&               dronePos,
                               const Coordinates&               targetPos,
                               const std::vector<Coordinates>&  path)
{
    const int W = std::min(grid.GetWidth(),  30);
    const int H = std::min(grid.GetLength(), 30);

    // Simulate frame refresh by scrolling previous frame out of view
    for (int i = 0; i < 25; ++i) std::cout << '\n';

    // Build character display buffer, default to free space
    std::vector<std::vector<char>> display(H, std::vector<char>(W, '.'));

    // Pass 1 – mark occupied cells as obstacles
    for (int row = 0; row < H; ++row) {
        for (int col = 0; col < W; ++col) {
            if (grid.IsOccupied(col, row)) {
                display[row][col] = '#';
            }
        }
    }

    // Pass 2 – mark active path waypoints
    for (int i = 0; i < static_cast<int>(path.size()); ++i) {
        const int px = static_cast<int>(path[i].x);
        const int py = static_cast<int>(path[i].y);
        if (px >= 0 && px < W && py >= 0 && py < H) {
            display[py][px] = '*';
        }
    }

    // Pass 3 – mark target (overwrites path marker if coincident)
    const int tx = static_cast<int>(targetPos.x);
    const int ty = static_cast<int>(targetPos.y);
    if (tx >= 0 && tx < W && ty >= 0 && ty < H) {
        display[ty][tx] = 'T';
    }

    // Pass 4 – mark drone at highest priority (overwrites all other markers)
    const int dx = static_cast<int>(dronePos.x);
    const int dy = static_cast<int>(dronePos.y);
    if (dx >= 0 && dx < W && dy >= 0 && dy < H) {
        display[dy][dx] = 'D';
    }

    // Render header legend
    std::cout << "+-- GRID (" << W << "x" << H << ")"
              << "  [D=Drone  T=Target  *=Path  #=Obstacle  .=Free] --+\n";

    // Render rows from top (high y) to bottom (y = 0) — no break/continue
    for (int row = H - 1; row >= 0; --row) {
        std::cout << std::setw(3) << row << " |";
        for (int col = 0; col < W; ++col) {
            std::cout << display[row][col] << ' ';
        }
        std::cout << "|\n";
    }

    // X-axis index row
    std::cout << "     ";
    for (int col = 0; col < W; ++col) {
        std::cout << (col % 10) << ' ';
    }
    std::cout << '\n';
}

// ── displayTelemetry ──────────────────────────────────────────────────────────

void MissionView::displayTelemetry(double             battery,
                                    MissionState        state,
                                    const Coordinates&  currentPos)
{
    const std::string stateStr = stateToString(state);

    // Battery bar: 20 characters wide, proportional fill
    const int BAR    = 20;
    const int filled = static_cast<int>(battery / 100.0 * BAR);
    const int empty  = BAR - filled;

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

std::string MissionView::stateToString(MissionState state)
{
    if (state == MissionState::IDLE)     return "IDLE";
    if (state == MissionState::OUTBOUND) return "OUTBOUND";
    if (state == MissionState::RECON)    return "RECON";
    if (state == MissionState::EVADE)    return "EVADE";
    if (state == MissionState::RETURN)   return "RETURN";
    if (state == MissionState::LANDED)   return "LANDED";
    return "UNKNOWN";
}

} // namespace AIGD
