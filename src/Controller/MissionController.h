#pragma once

#include "../Common/AppEnums.h"
#include "../Model/OccupancyGrid.h"
#include "../Model/IIntelligenceManager.h"
#include "../View/MissionView.h"
#include "../Utils/KalmanFilter.h"
#include "../Utils/AStarPlanner.h"
#include "../Utils/DStarLitePlanner.h"
#include "../Utils/CostCalculator.h"

#include <string>
#include <vector>
#include <memory>
#include <random>

// ── Mission-controller compile-time configuration macros ──────────────────────
// Defined at file scope (outside the class and namespace) so that both the
// controller and any future diagnostic utilities can reference them without
// instantiating the controller.  #define rather than constexpr keeps them
// visible to the preprocessor (e.g. for conditional-compilation guards).
#define BATTERY_DRAIN_PER_STEP  1.0      ///< Battery (%) consumed per active waypoint step
#define BATTERY_FAILSAFE        20.0     ///< Battery (%) threshold that triggers emergency RETURN
#define MAX_LOOP_ITERATIONS     10000    ///< Hard safety cap on Sense-Think-Act iterations
#define SENSOR_NOISE_STD        0.3      ///< σ (metres) for simulated GPS/INS position noise

// ── Multi-obstacle course layout macros ───────────────────────────────────────
//
// These macros configure the two-phase challenge embedded in runMission().
//
// PHASE 1 — Static barriers (pre-mission grid injection):
//   Wall A: vertical barrier at column WALL_A_COL spanning rows
//           WALL_A_Y_MIN..WALL_A_Y_MAX (inclusive). Gaps at y < WALL_A_Y_MIN
//           (top) and y > WALL_A_Y_MAX (bottom).
//   Wall B: vertical barrier at column WALL_B_COL spanning rows
//           WALL_B_Y_MIN..WALL_B_Y_MAX (inclusive). Gaps at y < WALL_B_Y_MIN
//           (top, very narrow: y=0,1 only) and y > WALL_B_Y_MAX (bottom: y=7,8,9).
//
//   Dual-barrier maze effect:
//     Wall A blocks the center corridor (y=3..7 at x=3).
//     Wall B blocks y=2..6 at x=6, so the ONLY path that clears both walls
//     without a major detour is the bottom corridor: y >= 8 through Wall A,
//     then y >= 7 through Wall B.  A* (STEALTH, equal per-cell cost at p=0.5)
//     selects the shortest path, which is the 15-step bottom corridor.
//
//   STATIC_HITS_PER_CELL: the soft Bayesian rule needs 3 consecutive
//   UpdateCell(true) calls to push probability past OCCUPANCY_THRESHOLD (0.7):
//     0.50 → 0.605 → 0.6785 → ~0.730  (impassable to both planners).
//
// PHASE 2 — Dynamic obstacles (SensorModule trigger schedule):
//   Two mid-flight injection events fire during the OUTBOUND Sense-Think-Act loop:
//
//   Trigger 1 at step DYNAMIC_TRIGGER_1_STEP (iteration 5):
//     Drone has advanced 4 waypoints along the bottom corridor to cell (1,8).
//     Obstacle placed at (1 + 2, 8) = (3,8) — the exact gap cell of Wall A.
//     D* Lite REPLANNING 1: seals the bottom gap; detours through y=9 corridor.
//
//   Trigger 2 at step DYNAMIC_TRIGGER_2_STEP (iteration 10):
//     After replanning, drone has advanced to cell (5,9) on the y=9 corridor.
//     Obstacle placed at (5 + 2, 9) = (7,9) — directly on the new path.
//     D* Lite REPLANNING 2: detours from y=9 down through (6,8)→(7,8)→...→(9,5).
// ─────────────────────────────────────────────────────────────────────────────
#define WALL_A_COL              3        ///< Grid column of static obstacle barrier A
#define WALL_A_Y_MIN            3        ///< First blocked row of barrier A (inclusive)
#define WALL_A_Y_MAX            7        ///< Last  blocked row of barrier A (inclusive)
#define WALL_B_COL              6        ///< Grid column of static obstacle barrier B
#define WALL_B_Y_MIN            2        ///< First blocked row of barrier B (inclusive)
#define WALL_B_Y_MAX            6        ///< Last  blocked row of barrier B (inclusive)
#define STATIC_HITS_PER_CELL    3        ///< UpdateCell(true) calls needed to exceed OCCUPANCY_THRESHOLD
#define DYNAMIC_TRIGGER_1_STEP  5        ///< STA iteration index for first  dynamic obstacle
#define DYNAMIC_TRIGGER_2_STEP  10       ///< STA iteration index for second dynamic obstacle
#define DYNAMIC_TRIGGER_OFF_X   2.0      ///< X-offset (cells ahead of drone) for obstacle placement
#define DYNAMIC_TRIGGER_OFF_Y   0.0      ///< Y-offset from drone row (zero = same row as drone)

namespace AIGD {

// ── DroneState ────────────────────────────────────────────────────────────────
/**
 * @brief Mutable runtime state of the physical drone platform.
 *
 * Passed by reference into MissionController so the controller can read and
 * modify drone state without owning it (dependency-injection pattern).
 * Fields are intentionally plain data — no invariants to enforce.
 */
struct DroneState {
    Coordinates position;       ///< Current grid position (x, y, altitude)
    double      batteryLevel;   ///< Remaining charge [0, 100] percent
    bool        gpsAvailable;   ///< false when GPS signal is lost (triggers fail-safe)

    explicit DroneState(const Coordinates& startPos,
                        double battery = 100.0)
        : position(startPos), batteryLevel(battery), gpsAvailable(true) {}
};

// ── SensorReading ─────────────────────────────────────────────────────────────
/**
 * @brief Bundle of all data produced by one sensor frame (EO/Thermal + GPS/INS).
 *
 * Produced by SensorModule::sense() and consumed entirely within one
 * Sense-Think-Act iteration.  The rawImagePayload is later Base64-encoded
 * and stored in the IntelligenceManager during the RECON phase.
 */
struct SensorReading {
    double      noisyPositionX;    ///< Raw GPS/INS x measurement before Kalman correction
    double      noisyPositionY;    ///< Raw GPS/INS y measurement before Kalman correction
    bool        obstacleDetected;  ///< true when a new obstacle is detected on the path
    Coordinates obstacleCell;      ///< Grid cell of the detected obstacle
    std::string rawImagePayload;   ///< Simulated EO/Thermal image bytes (ASCII placeholder)
};

// ── SensorModule ──────────────────────────────────────────────────────────────
/**
 * @brief Simulated EO/Thermal sensor suite with Gaussian position noise.
 *
 * Provides a realistic simulation of:
 *  - GPS/INS measurement noise (normal distribution, σ = SENSOR_NOISE_STD).
 *  - A single mid-flight obstacle injection to exercise the D* Lite
 *    replanning path (HandleThreatDetection).
 *
 * Modularity note: replace this class with a real sensor driver (e.g. FLIR
 * Tau 2 for thermal, Sony Alpha for EO) without modifying any other component.
 * The constructor signature and sense() interface remain stable.
 */
class SensorModule {
public:
    /**
     * @param seed          RNG seed — fixed seed gives reproducible simulation runs.
     * @param obstacleStep  Loop iteration at which the simulated obstacle fires.
     * @param obstacleOffX  X-offset from the drone's position when obstacle fires.
     */
    explicit SensorModule(unsigned int seed = 42,
                          int          obstacleStep  = 5,
                          double       obstacleOffX  = 2.0);

    /**
     * @brief Produces one complete sensor frame for the given drone position.
     *
     * Returns noisy GPS/INS position (drawn from N(0, SENSOR_NOISE_STD²)),
     * an optional obstacle detection (fired exactly once at obstacleStep),
     * and a simulated EO image payload string.
     *
     * @param truePos    True drone position (model ground truth).
     * @param stepIndex  Current Sense-Think-Act iteration index.
     * @return           Populated SensorReading struct.
     */
    SensorReading sense(const Coordinates& truePos, int stepIndex);

    /**
     * @brief Registers an additional mid-flight obstacle injection event.
     *
     * Appends a new ObstacleTrigger to the internal schedule vector.
     * Each registered trigger fires exactly once when the STA loop reaches
     * the specified `step` iteration, placing a synthetic obstacle at
     * (truePos.x + offX, truePos.y + offY) in the sensor reading.
     *
     * Multiple triggers at distinct step indices create a multi-event dynamic
     * obstacle course that exercises D* Lite incremental replanning repeatedly
     * — one replanning event per trigger that lands on the drone's active path.
     *
     * This method enables the runMission() orchestrator to add Phase 2 triggers
     * after SensorModule construction without changing the constructor signature
     * or the sense() / MissionController call sites.
     *
     * @param step  Zero-based STA loop iteration at which this trigger fires.
     * @param offX  X-offset (grid cells) from the drone's current position.
     * @param offY  Y-offset (grid cells) from the drone's current position.
     */
    void addObstacleTrigger(int step, double offX, double offY = 0.0);

private:
    /**
     * @brief One mid-flight obstacle injection event.
     *
     * Stored in a parallel-vector pair (m_triggers / m_triggered) so any
     * number of obstacles can be scheduled without modifying the constructor
     * or the sense() call site.  offX / offY are relative to the drone's
     * true position at the moment the trigger fires, guaranteeing the obstacle
     * lands on (or near) the drone's current heading regardless of which
     * replanned path it is following at that instant.
     */
    struct ObstacleTrigger {
        int    step;   ///< STA loop iteration index at which this trigger activates
        double offX;   ///< X-offset (grid cells) from drone position at trigger time
        double offY;   ///< Y-offset (grid cells) from drone position at trigger time
    };

    std::mt19937                     m_rng;
    std::normal_distribution<double> m_posDist;    ///< Position noise ~N(0, SENSOR_NOISE_STD²)
    std::vector<ObstacleTrigger>     m_triggers;   ///< All scheduled trigger events
    std::vector<bool>                m_triggered;  ///< Per-trigger fired flag; 1:1 index match with m_triggers
};

// ── MissionConfig ─────────────────────────────────────────────────────────────
/**
 * @brief External configuration bundle loaded from config.json by ConfigParser.
 *
 * Holds the Kalman filter noise parameters and the mission priority profile.
 * Default values replicate the previously hard-coded constants so the system
 * degrades gracefully if the config file is absent.
 */
struct MissionConfig {
    double          kalmanQ  = 0.01;                      ///< Process noise variance Q
    double          kalmanR  = 0.09;                      ///< Measurement noise variance R (σ²)
    MissionPriority priority = MissionPriority::BALANCED; ///< Path-planning cost profile
};

// ── MissionController ─────────────────────────────────────────────────────────
/**
 * @class MissionController
 * @brief FSM manager and Sense-Think-Act loop orchestrator (MVC Controller layer).
 *
 * Manages the full mission lifecycle through six FSM states:
 *
 *   [IDLE] ──StartMission()──► [OUTBOUND] ──arrival──► [RECON]
 *                                   │                       │
 *                               obstacle                IMINT stored
 *                                   ▼                       │
 *                              [EVADE]               [RETURN] ──arrival──► [LANDED]
 *                                   │ D* Lite replanned    ▲
 *                                   └──────────────────────┘
 *
 * Each Sense-Think-Act iteration performs three phases:
 *   SENSE  — raw sensor frame + Kalman filter predict/update
 *   THINK  — fail-safe evaluation, obstacle detection, battery drain
 *   ACT    — waypoint advance, IMINT capture, state transitions
 *
 * Architecture (MVC):
 *   Model      OccupancyGrid, IIntelligenceManager, KalmanFilter,
 *              AStarPlanner, DStarLitePlanner, CostCalculator
 *   View       MissionView  (real-time telemetry, ASCII grid, event log)
 *   Controller this class   (FSM, STA loop, planner orchestration)
 *
 * All dependencies are constructor-injected (non-owning references).
 * No global state or singletons — the design is fully unit-testable.
 */
class MissionController {
public:
    /**
     * @brief Constructs the controller by injecting all dependencies.
     *
     * Stores non-owning references to all model and view components.
     * The Kalman filters are seeded with placeholder (0,0) states;
     * InitializeMission() re-seeds them with the actual start position.
     *
     * @param grid      Shared occupancy map consulted and updated by planners.
     * @param drone     Drone runtime state (position, battery, GPS).
     * @param sensors   Sensor driver / simulation module.
     * @param intelMgr  Hash-table-backed IMINT store (O(1) insert/lookup).
     * @param view      Mission HUD: telemetry, ASCII grid, alert log.
     * @param config    Kalman Q/R and mission priority (from config.json).
     */
    MissionController(OccupancyGrid&        grid,
                      DroneState&           drone,
                      SensorModule&         sensors,
                      IIntelligenceManager& intelMgr,
                      MissionView&          view,
                      const MissionConfig&  config = MissionConfig{});

    /**
     * @brief Pre-flight setup: stamps obstacles, runs A*, constructs D* Lite.
     *
     * Steps performed:
     *   1. Store mission geometry (start, target).
     *   2. Re-seed Kalman filters at the true start position.
     *   3. Stamp known obstacles onto the OccupancyGrid (3 hits → p > 0.7).
     *   4. Compute initial global path via A*.
     *   5. Construct DStarLitePlanner (builds full backward g/rhs surface).
     *   6. Set pathIndex = 1 (skip origin cell already occupied by drone).
     *   7. Arm FSM to IDLE.  If A* fails, FSM goes directly to LANDED.
     *
     * @param start           Mission departure coordinate.
     * @param target          Reconnaissance target coordinate.
     * @param knownObstacles  Pre-mission obstacle cells; may be empty.
     */
    void InitializeMission(const Coordinates&              start,
                           const Coordinates&              target,
                           const std::vector<Coordinates>& knownObstacles);

    /**
     * @brief Top-level entry point: arms the FSM and runs the STA loop.
     *
     * Calls StartMission() then ExecuteSenseThinkActLoop() to completion.
     *
     * @return true  if the drone reached LANDED after a successful RECON.
     *         false if the mission was aborted (fail-safe, no path found, etc.).
     */
    bool RunMainLoop();

    /**
     * @brief FSM transition: IDLE → OUTBOUND.
     *
     * Ignored (with a logged alert) if the FSM is not in IDLE state,
     * preventing out-of-order calls from corrupting the state machine.
     */
    void StartMission();

    /**
     * @brief Runs the Sense-Think-Act loop until LANDED or MAX_LOOP_ITERATIONS.
     *
     * Each iteration:
     *   Sense  — sensor frame + Kalman predict + Kalman update
     *   Think  — fail-safe check, obstacle detection, battery drain
     *   Act    — waypoint advance (OUTBOUND/RETURN), IMINT capture (RECON),
     *            state transitions
     *
     * A hard cap of MAX_LOOP_ITERATIONS prevents infinite loops in edge cases
     * (e.g. path cycles due to numerical drift).
     */
    void ExecuteSenseThinkActLoop();

    /**
     * @brief EVADE handler: triggers D* Lite replanning around a new obstacle.
     *
     * Marks the obstacle in the grid (3 Bayesian hits internally), calls
     * D* Lite updatePath() to reprocess only the affected vertices, then
     * either resumes OUTBOUND (valid detour found) or falls back to RETURN
     * (target is completely surrounded).
     *
     * @param obstacleCell Grid cell of the newly detected obstacle.
     */
    void HandleThreatDetection(const Coordinates& obstacleCell);

    /**
     * @brief Static mission orchestrator — the single top-level entry point for
     *        the full autonomous IMINT collection pipeline.
     *
     * Encapsulates the complete mission lifecycle in one self-contained call:
     *
     *   1. Parse the JSON configuration (Kalman Q/R, grid dimensions, Base64
     *      map payload, mission coordinates, navigation priority string).
     *
     *   2. Decode the Base64 map and initialise OccupancyGrid; stamp any '#'
     *      cells as pre-known obstacles (STATIC_HITS_PER_CELL Bayesian updates
     *      push each cell's probability past OCCUPANCY_THRESHOLD = 0.7).
     *
     *   3. PHASE 1 — Static obstacle injection (before InitializeMission):
     *      Inject two vertical barrier walls — Wall A at column WALL_A_COL
     *      (rows WALL_A_Y_MIN..WALL_A_Y_MAX) and Wall B at column WALL_B_COL
     *      (rows WALL_B_Y_MIN..WALL_B_Y_MAX) — via UpdateCell calls.
     *      These walls create a dual-barrier maze forcing A* to plan a
     *      dramatically non-linear corridor path from start (0,5) to target
     *      (9,5); the straight horizontal route is completely blocked.
     *
     *   4. PHASE 2 — Dynamic obstacle schedule (SensorModule configuration):
     *      Register two obstacle triggers (at steps DYNAMIC_TRIGGER_1_STEP and
     *      DYNAMIC_TRIGGER_2_STEP, each with offset DYNAMIC_TRIGGER_OFF_X cells
     *      ahead of the drone).  During the STA loop each trigger fires right on
     *      the drone's active route, forcing D* Lite incremental replanning twice.
     *
     *   5. Construct all MVC components (OccupancyGrid, DroneState, SensorModule,
     *      IntelligenceManager, MissionView, MissionController), call
     *      InitializeMission() to run A* and build the D* Lite cost surface,
     *      then execute the full Sense-Think-Act loop via RunMainLoop().
     *
     *   6. Call ReportGenerator::generateReport() to write the structured JSON
     *      intelligence report with per-record Base64 integrity validation.
     *
     * Single-return compliance: a single local bool `outcome` is declared at the
     * top of the function body and is the only variable returned.  The sole
     * return statement sits at the absolute final line of the function.
     *
     * @param configFilePath  Relative or absolute path to the JSON config file
     *                        (e.g., "config.json" in the working directory).
     * @return true  if the drone reached LANDED after a successful RECON cycle
     *               AND the JSON report was written without I/O error.
     *         false if the mission aborted (fail-safe, no path found, max
     *               iterations) OR the report could not be written.
     */
    static bool runMission(const std::string& configFilePath);

private:
    // ── Injected dependencies (non-owning references) ────────────────────────
    OccupancyGrid&        m_grid;      ///< Shared probability map
    DroneState&           m_drone;     ///< Mutable drone platform state
    SensorModule&         m_sensors;   ///< Sensor driver / simulation
    IIntelligenceManager& m_intelMgr;  ///< IMINT hash-table store
    MissionView&          m_view;      ///< Terminal HUD / event log

    // ── Mission geometry ──────────────────────────────────────────────────────
    Coordinates               m_startPoint;    ///< Departure cell (used for RETURN path)
    Coordinates               m_targetPoint;   ///< Reconnaissance target cell

    // ── Active path state ─────────────────────────────────────────────────────
    std::vector<Coordinates>  m_currentPath;   ///< Ordered waypoints (inclusive of start)
    int                       m_pathIndex;     ///< Index of the NEXT waypoint to visit

    // ── Planner instances ─────────────────────────────────────────────────────
    CostCalculator                    m_costCalc;  ///< Weighted cost engine (shared by both planners)
    AStarPlanner                      m_astar;     ///< Global path planner (initial + return paths)
    std::unique_ptr<DStarLitePlanner> m_dstar;     ///< Incremental replanner (constructed in Init)

    // ── External configuration ────────────────────────────────────────────────
    MissionConfig m_config;   ///< Kalman Q/R and mission priority from config.json

    // ── Sensor fusion: one 1-D Kalman filter per spatial axis ────────────────
    KalmanFilter  m_kalmanX;  ///< Fuses noisy GPS x-measurements
    KalmanFilter  m_kalmanY;  ///< Fuses noisy GPS y-measurements

    // ── FSM state ─────────────────────────────────────────────────────────────
    MissionState  m_state;          ///< Current FSM state (IDLE/OUTBOUND/…/LANDED)
    bool          m_missionSuccess; ///< Set true only on successful RETURN → LANDED

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * @brief Replaces the active path and resets the path index.
     *
     * Skips waypoint[0] because both A* and D* Lite return paths that include
     * the drone's current cell as the first element.
     *
     * @param newPath  New ordered waypoint list (moved-from, not copied).
     */
    void setActivePath(std::vector<Coordinates> newPath);

    /**
     * @brief Evaluates battery and GPS fail-safe conditions.
     *
     * Checks battery first; if critical, logs an alert and returns true.
     * Falls through to the GPS check only if battery is nominal.
     * Single-return via else-if chain.
     *
     * @return true if any fail-safe condition is active.
     */
    bool checkFailSafe() const;

    /**
     * @brief Returns true when the integer grid coordinates of a and b match.
     *
     * Grid cells are 1×1 unit squares so truncation to int is the correct
     * comparison for arrival detection (floating-point equality is unreliable).
     *
     * @param a First coordinate.
     * @param b Second coordinate.
     * @return  true when ⌊a.x⌋ == ⌊b.x⌋ and ⌊a.y⌋ == ⌊b.y⌋.
     */
    static bool coordsMatch(const Coordinates& a, const Coordinates& b);

    /**
     * @brief Base64-encodes the EO payload and stores it in the IntelligenceManager.
     *
     * Wraps the encoded string in an IntelData record stamped with the current
     * Unix timestamp in milliseconds, then calls m_intelMgr.addIntel() for
     * O(1) insertion into the hash table.
     *
     * @param location    Drone position at time of capture.
     * @param rawPayload  Raw (unencoded) EO image string.
     */
    void captureAndStoreImint(const Coordinates& location,
                              const std::string& rawPayload);

    /**
     * @brief Returns the human-readable name string for a FSM state.
     *
     * Defaults to "UNKNOWN" for any value not covered by the else-if chain,
     * providing a safe fallback for future enum additions.
     *
     * @param state  FSM state to convert.
     * @return       "IDLE", "OUTBOUND", "RECON", "EVADE", "RETURN",
     *               "LANDED", or "UNKNOWN".
     */
    static std::string stateToString(MissionState state);
};

} // namespace AIGD
