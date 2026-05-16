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

private:
    std::mt19937                           m_rng;
    std::normal_distribution<double>       m_posDist;       ///< Position noise ~N(0, σ)
    int                                    m_obstacleStep;
    double                                 m_obstacleOffX;
    bool                                   m_obstacleTriggered;
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
