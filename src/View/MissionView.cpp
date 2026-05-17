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

    // Render rows from top (high y) to bottom (y = 0) 
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

// ══════════════════════════════════════════════════════════════════════════════
// Orchestration Output Methods
// ══════════════════════════════════════════════════════════════════════════════
//
// These nine methods receive structured data from MissionController::runMission()
// and are the sole source of all pipeline-stage console output.  The Controller
// owns zero std::cout calls; every printed character originates here.
//
// ──────────────────────────────────────────────────────────────────────────────

// ── printOrchestratorBanner ───────────────────────────────────────────────────
//
// Prints the top-of-run header banner and confirms the configuration file path
// before ConfigParser reads it.  The operator sees this line first, before any
// JSON parsing begins, so a wrong filename can be spotted immediately.
// ──────────────────────────────────────────────────────────────────────────────
void MissionView::printOrchestratorBanner(const std::string& configFilePath)
{
    std::cout << "\n=== AIGD ORCHESTRATOR — runMission() ===\n";
    std::cout << "[CONFIG] File        : " << configFilePath << "\n";
}

// ── printConfigSummary ────────────────────────────────────────────────────────
//
// Renders the four mission-critical fields from the parsed AppConfig:
// Kalman noise parameters (Q process, R measurement), OccupancyGrid dimensions
// (width × height), and the navigation priority profile (STEALTH / AGGRESSIVE /
// BALANCED).  The trailing blank line visually separates the config block
// from the Phase 1 obstacle announcement that follows.
// ──────────────────────────────────────────────────────────────────────────────
void MissionView::printConfigSummary(double             kalmanQ,
                                      double             kalmanR,
                                      int                gridWidth,
                                      int                gridHeight,
                                      const std::string& navigationPriority)
{
    std::cout << "[CONFIG] Kalman Q    : " << kalmanQ
              << "  R: " << kalmanR << "\n";
    std::cout << "[CONFIG] Grid        : " << gridWidth
              << " x " << gridHeight << "\n";
    std::cout << "[CONFIG] Priority    : " << navigationPriority << "\n\n";
}

// ── printPhaseOneObstacles ────────────────────────────────────────────────────
//
// Announces the two static obstacle walls before UpdateCell() is called,
// giving the operator a textual preview they can cross-reference against
// the ASCII grid that printOccupancyGrid() renders immediately after.
//
// Wall A column and row range are printed first; Wall B second.  The format
// "col=N  rows=M..K" is compact but unambiguous: the examiner can immediately
// see where each barrier sits on the grid without opening any source file.
// ──────────────────────────────────────────────────────────────────────────────
void MissionView::printPhaseOneObstacles(int wallACol, int wallAYMin, int wallAYMax,
                                          int wallBCol, int wallBYMin, int wallBYMax)
{
    std::cout << "[PHASE 1] Injecting static obstacle barriers:\n";
    std::cout << "  Wall A: col=" << wallACol
              << "  rows=" << wallAYMin << ".." << wallAYMax << "\n";
    std::cout << "  Wall B: col=" << wallBCol
              << "  rows=" << wallBYMin << ".." << wallBYMax << "\n\n";
}

// ── printOccupancyGrid ────────────────────────────────────────────────────────
//
// Delegates to OccupancyGrid::PrintGrid() so the View does not duplicate the
// probability-to-character mapping logic that already lives in the Model class.
// This is the one permitted case where the View calls a Model method directly —
// standard MVC practice where the View reads from the Model to render state.
// The extra blank line after the grid creates visual separation before the
// Phase 2 trigger schedule announcement that follows.
// ──────────────────────────────────────────────────────────────────────────────
void MissionView::printOccupancyGrid(const OccupancyGrid& grid)
{
    grid.PrintGrid();
    std::cout << "\n";
}

// ── printPhaseTwoTriggers ─────────────────────────────────────────────────────
//
// Displays the two SensorModule trigger events scheduled for the OUTBOUND leg.
// Each line shows the STA loop step index (when) and the x-offset (where the
// obstacle will appear relative to the drone).  The parenthetical description
// tells the operator what to watch for in the real-time grid output during the
// subsequent RunMainLoop().
// ──────────────────────────────────────────────────────────────────────────────
void MissionView::printPhaseTwoTriggers(int    trigger1Step, double trigger1OffX,
                                         int    trigger2Step, double trigger2OffX)
{
    std::cout << "[PHASE 2] Dynamic obstacle trigger schedule:\n";
    std::cout << "  Trigger 1: STA step=" << trigger1Step
              << "  offX=" << trigger1OffX
              << "  (fires " << static_cast<int>(trigger1OffX)
              << " cells ahead — forces D* Lite replanning 1)\n";
    std::cout << "  Trigger 2: STA step=" << trigger2Step
              << "  offX=" << trigger2OffX
              << "  (fires " << static_cast<int>(trigger2OffX)
              << " cells ahead — forces D* Lite replanning 2)\n\n";
}

// ── printRouteDetails ─────────────────────────────────────────────────────────
//
// Prints the departure and target coordinates immediately before RunMainLoop()
// lifts the drone off.  The operator can verify the mission geometry one final
// time after A* has confirmed a valid path exists (InitializeMission succeeded).
// Double newline at the end creates visual separation before the real-time
// Sense-Think-Act output stream begins.
// ──────────────────────────────────────────────────────────────────────────────
void MissionView::printRouteDetails(const Coordinates& start, const Coordinates& target)
{
    std::cout << "[ORCHESTRATOR] Route : ("
              << start.x  << ", " << start.y  << ")"
              << " \xe2\x86\x92 (" << target.x << ", " << target.y << ")\n\n";
}

// ── printMissionSummary ───────────────────────────────────────────────────────
//
// Post-mission debrief panel printed immediately after RunMainLoop() returns.
// Four fields are always printed: outcome, battery, IMINT record count, and
// a blank separator line.
// The payload size line is suppressed when intelCount = 0 (payloadBytes == 0)
// via a plain if block — no return statement inside, so single-return holds.
// std::fixed / setprecision(1) ensures battery is rendered as "71.0 %" rather
// than a platform-dependent default floating-point format.
// ──────────────────────────────────────────────────────────────────────────────
void MissionView::printMissionSummary(bool   success,
                                       double batteryLevel,
                                       int    intelCount,
                                       int    payloadBytes)
{
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "\n[ORCHESTRATOR] Mission: "
              << (success ? "SUCCESS" : "FAILURE") << "\n";
    std::cout << "[ORCHESTRATOR] Battery: " << batteryLevel << " %\n";
    std::cout << "[ORCHESTRATOR] IMINT  : " << intelCount << " record(s)\n";
    if (payloadBytes > 0) {
        std::cout << "[ORCHESTRATOR] Payload: "
                  << payloadBytes << " B (Base64-encoded)\n";
    }
    std::cout << "\n";
}

// ── printReportStatus ─────────────────────────────────────────────────────────
//
// Reports whether the JSON intelligence report was written successfully.
// The status message is built via an if/else assignment into a local string
// variable, then printed in a single cout — no ternary operator in the stream
// insertion, no return inside the if block.  MISSION_REPORT_FILE (defined in
// MissionView.h) supplies the filename for the success message so the operator
// knows exactly where to find the output without checking the source code.
// ──────────────────────────────────────────────────────────────────────────────
void MissionView::printReportStatus(bool reportOk)
{
    std::string statusMsg = "FAILED";
    if (reportOk) {
        statusMsg = std::string("WRITTEN \xe2\x86\x92 ") + MISSION_REPORT_FILE;
    }
    std::cout << "[ORCHESTRATOR] Report : " << statusMsg << "\n";
}

// ── printOrchestratorComplete ─────────────────────────────────────────────────
//
// Prints the final line of the mission run, clearly marking the boundary
// between the pipeline output and the shell prompt that follows.
// ──────────────────────────────────────────────────────────────────────────────
void MissionView::printOrchestratorComplete()
{
    std::cout << "=== runMission() complete ===\n\n";
}

} // namespace AIGD
