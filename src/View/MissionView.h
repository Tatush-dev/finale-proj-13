#pragma once
#include "IMissionView.h"
#include "../Model/OccupancyGrid.h"
#include "../Common/AppEnums.h"
#include <list>
#include <vector>
#include <string>

// ── Report output constant ─────────────────────────────────────────────────────
// Centralised in MissionView.h so both the View (printReportStatus) and the
// Controller (ReportGenerator::generateReport call in runMission) reference
// the same filename via the same token, preventing silent mismatches.
#define MISSION_REPORT_FILE  "mission_report.json"  ///< JSON intelligence report output path

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

    // ── Orchestration output methods ──────────────────────────────────────────
    //
    // Called exclusively by MissionController::runMission() to render every
    // stage of the mission pipeline.  Encapsulating all std::cout calls behind
    // these methods upholds the strict MVC contract: the Controller orchestrates
    // logic, the View owns every byte of console output.
    //

    /**
     * @brief Prints the AIGD orchestrator startup banner and the config file path.
     *
     * Called as the very first console output of runMission(), before the JSON
     * file is parsed.  Establishes the visual identity of the mission run and
     * confirms the configuration file name so the operator can verify it before
     * any system state is modified.
     *
     * @param configFilePath  Relative or absolute path to the JSON config file
     *                        being parsed (e.g. "config.json").
     */
    void printOrchestratorBanner(const std::string& configFilePath);

    /**
     * @brief Prints the parsed mission configuration for pre-flight verification.
     *
     * Displays the four mission-critical fields extracted from AppConfig:
     * Kalman Q/R noise parameters, OccupancyGrid dimensions, and the navigation
     * priority profile.  Printed immediately after ConfigParser::parse() so any
     * misconfiguration can be caught before grid initialisation or path planning.
     *
     * @param kalmanQ             Kalman process-noise variance Q (≥ 0).
     * @param kalmanR             Kalman measurement-noise variance R (≥ 0).
     * @param gridWidth           OccupancyGrid width  (number of x-axis cells).
     * @param gridHeight          OccupancyGrid height (number of y-axis cells).
     * @param navigationPriority  Priority string: "STEALTH", "AGGRESSIVE", or "BALANCED".
     */
    void printConfigSummary(double kalmanQ, double kalmanR,
                             int gridWidth, int gridHeight,
                             const std::string& navigationPriority);

    /**
     * @brief Announces the Phase 1 static obstacle walls being injected into the grid.
     *
     * Prints the column index and inclusive row range of each wall before the
     * UpdateCell() call sequence begins.  Gives the operator a textual preview
     * of the dual-barrier maze that A* will need to navigate around so they can
     * cross-reference it with the grid render that follows (printOccupancyGrid).
     *
     * @param wallACol   Column index (x) of static obstacle barrier A.
     * @param wallAYMin  First blocked row of barrier A (inclusive).
     * @param wallAYMax  Last  blocked row of barrier A (inclusive).
     * @param wallBCol   Column index (x) of static obstacle barrier B.
     * @param wallBYMin  First blocked row of barrier B (inclusive).
     * @param wallBYMax  Last  blocked row of barrier B (inclusive).
     */
    void printPhaseOneObstacles(int wallACol, int wallAYMin, int wallAYMax,
                                 int wallBCol, int wallBYMin, int wallBYMax);

    /**
     * @brief Renders the full OccupancyGrid probabilistic map to the console.
     *
     * Delegates to OccupancyGrid::PrintGrid() so the View does not duplicate
     * the grid-rendering logic that already lives in the Model.  Called after
     * Phase 1 static obstacle injection to give the operator visual confirmation
     * that both barrier walls are correctly stamped at their expected columns
     * and row ranges before A* computes the initial path.
     * Appends a blank separator line after the grid for visual clarity.
     *
     * @param grid  The occupancy grid after static obstacles have been injected.
     */
    void printOccupancyGrid(const OccupancyGrid& grid);

    /**
     * @brief Announces the Phase 2 dynamic obstacle trigger schedule.
     *
     * Displays the STA loop iteration step and X-offset for each SensorModule
     * trigger registered via addObstacleTrigger().  These events fire during
     * the OUTBOUND leg and each force D* Lite incremental replanning.
     * Printed just before controller construction so the operator can anticipate
     * the two replanning events they will observe in the real-time grid output.
     *
     * @param trigger1Step  STA iteration index at which the first  trigger fires.
     * @param trigger1OffX  X-offset (grid cells ahead of drone) for trigger 1.
     * @param trigger2Step  STA iteration index at which the second trigger fires.
     * @param trigger2OffX  X-offset (grid cells ahead of drone) for trigger 2.
     */
    void printPhaseTwoTriggers(int    trigger1Step, double trigger1OffX,
                                int    trigger2Step, double trigger2OffX);

    /**
     * @brief Prints the planned mission route from departure to reconnaissance target.
     *
     * Called after InitializeMission() confirms a valid A* path exists, immediately
     * before RunMainLoop() lifts the drone off.  Provides the operator with a
     * quick sanity check that the start and target coordinates match the mission
     * plan before any waypoint movement begins.
     *
     * @param start   Mission departure coordinate (x, y, altitude).
     * @param target  Reconnaissance target coordinate (x, y, altitude).
     */
    void printRouteDetails(const Coordinates& start, const Coordinates& target);

    /**
     * @brief Prints the post-mission outcome summary to the console.
     *
     * Provides a compact debrief after RunMainLoop() returns:
     *   - Mission outcome (SUCCESS / FAILURE)
     *   - Remaining battery level in percent
     *   - Number of IntelData records stored in the IntelligenceManager
     *   - Encoded payload size of the first record (omitted if intelCount = 0)
     *
     * The payload line is conditional on intelCount > 0 — the if block contains
     * only a std::cout statement (no return), so single-return compliance holds.
     *
     * @param success       true if the drone reached LANDED after a valid RECON.
     * @param batteryLevel  Remaining battery charge in percent at mission end.
     * @param intelCount    Total IntelData records stored by IntelligenceManager.
     * @param payloadBytes  Base64 payload size in bytes of the first record;
     *                      pass 0 if intelCount is 0 (suppresses payload line).
     */
    void printMissionSummary(bool   success,
                              double batteryLevel,
                              int    intelCount,
                              int    payloadBytes);

    /**
     * @brief Prints the result of the JSON intelligence report generation step.
     *
     * On success, prints the output filename (MISSION_REPORT_FILE) so the
     * operator knows precisely where to find the report on disk.  On failure,
     * prints a concise error indicator.
     *
     * Internally uses if/else string assignment rather than a ternary in a
     * stream insertion, keeping the conditional logic explicit and readable
     * while satisfying single-return compliance (no return inside the if block).
     *
     * @param reportOk  true if ReportGenerator::generateReport() succeeded.
     */
    void printReportStatus(bool reportOk);

    /**
     * @brief Prints the orchestrator completion banner.
     *
     * The last console output line of runMission().  Clearly marks the end
     * of the automated pipeline in the operator log so any subsequent output
     * (e.g. shell prompt) is visually separated from the mission transcript.
     */
    void printOrchestratorComplete();

private:
    static std::string stateToString(MissionState state);
};

} // namespace AIGD
