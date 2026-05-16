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

namespace AIGD {

// ── Drone State ───────────────────────────────────────────────────────────────
/**
 * Holds mutable runtime state for the physical drone platform.
 * Passed by reference into MissionController for dependency injection.
 */
struct DroneState {
    Coordinates position;
    double      batteryLevel;   ///< [0, 100] percent
    bool        gpsAvailable;

    explicit DroneState(const Coordinates& startPos,
                        double battery = 100.0)
        : position(startPos), batteryLevel(battery), gpsAvailable(true) {}
};

// ── Sensor Reading Bundle ─────────────────────────────────────────────────────
/**
 * All data produced by one sensor frame (EO/Thermal + GPS/INS).
 */
struct SensorReading {
    double      noisyPositionX;    ///< Raw GPS/INS x before Kalman correction
    double      noisyPositionY;    ///< Raw GPS/INS y before Kalman correction
    bool        obstacleDetected;  ///< True when a new obstacle is sensed on path
    Coordinates obstacleCell;      ///< Grid cell of the detected obstacle
    std::string rawImagePayload;   ///< Simulated IMINT image bytes (ASCII placeholder)
};

// ── Sensor Simulation Module ──────────────────────────────────────────────────
/**
 * Simulates EO/Thermal sensor frames with Gaussian position noise and a
 * single mid-flight obstacle injection to exercise D* Lite replanning.
 *
 * Modularity note: replace this class with a real sensor driver without
 * touching any other component.
 */
class SensorModule {
public:
    /**
     * @param seed         RNG seed — fixed seed gives reproducible runs.
     * @param obstacleStep Loop iteration at which the simulated obstacle appears.
     * @param obstacleOffX X-offset from the drone's position when obstacle fires.
     */
    explicit SensorModule(unsigned int seed = 42,
                          int          obstacleStep  = 5,
                          double       obstacleOffX  = 2.0);

    /**
     * Produces one sensor frame for the given drone position and loop step.
     * The obstacle is injected exactly once at obstacleStep to mirror a
     * real mid-flight threat discovery.
     */
    SensorReading sense(const Coordinates& truePos, int stepIndex);

private:
    std::mt19937                           m_rng;
    std::normal_distribution<double>       m_posDist;   ///< Position noise ~N(0, σ)
    int                                    m_obstacleStep;
    double                                 m_obstacleOffX;
    bool                                   m_obstacleTriggered;

    static constexpr double NOISE_STD = 0.3;   ///< σ for position noise (metres)
};

// ── External Configuration Bundle ────────────────────────────────────────────
/**
 * Kalman Filter tuning parameters loaded from config.json.
 * Defaults match the previously hard-coded values (Q=0.01, R=0.09).
 */
struct MissionConfig {
    double          kalmanQ  = 0.01;                    ///< Process noise variance
    double          kalmanR  = 0.09;                    ///< Measurement noise variance (σ²)
    MissionPriority priority = MissionPriority::BALANCED; ///< Path-planning cost profile
};

// ── Mission Controller (FSM + Sense-Think-Act) ────────────────────────────────
/**
 * Orchestrates the full mission lifecycle:
 *   IDLE → OUTBOUND → [EVADE] → RECON → RETURN → LANDED
 *
 * Architecture (MVC Controller layer):
 *  - Model:  OccupancyGrid, IIntelligenceManager, KalmanFilter, AStarPlanner,
 *            DStarLitePlanner, CostCalculator
 *  - View:   MissionView  (telemetry, alerts, final report)
 *  - Controller: this class drives the Sense-Think-Act loop and FSM transitions
 *
 * All dependencies are injected via the constructor (no global state).
 */
class MissionController {
public:
    /**
     * Constructor — all dependencies injected by reference (non-owning).
     *
     * @param grid      Probabilistic occupancy map shared with planners.
     * @param drone     Drone state (position, battery, GPS flag).
     * @param sensors   Sensor simulation / real sensor driver.
     * @param intelMgr  Hash-table-backed IMINT store.
     * @param view      Mission HUD / telemetry output.
     */
    MissionController(OccupancyGrid&        grid,
                      DroneState&           drone,
                      SensorModule&         sensors,
                      IIntelligenceManager& intelMgr,
                      MissionView&          view,
                      const MissionConfig&  config = MissionConfig{});

    /**
     * Populates the grid with any pre-known obstacles, computes the initial
     * global path via A*, constructs the D* Lite planner, and arms the FSM
     * (sets state to IDLE, ready for StartMission).
     *
     * @param start            Mission start coordinate.
     * @param target           Reconnaissance target coordinate.
     * @param knownObstacles   Pre-mission obstacle cells (may be empty).
     */
    void InitializeMission(const Coordinates&              start,
                           const Coordinates&              target,
                           const std::vector<Coordinates>& knownObstacles);

    /**
     * Top-level entry point: calls StartMission() then runs
     * ExecuteSenseThinkActLoop() to completion.
     *
     * @return true  if the drone reached LANDED state after a full RECON,
     *         false if the mission was aborted (fail-safe, no path, etc.).
     */
    bool RunMainLoop();

    // ── FSM action handlers ──────────────────────────────────────────────────
    /** Transitions IDLE → OUTBOUND and logs departure. */
    void StartMission();

    /**
     * Runs the Sense-Think-Act loop until LANDED or MAX_LOOP_ITERATIONS.
     * Each iteration:
     *   Sense  – sensor frame + Kalman filter correction
     *   Think  – fail-safe check, obstacle detection, replanning
     *   Act    – advance waypoint, capture IMINT, log telemetry
     */
    void ExecuteSenseThinkActLoop();

    /**
     * EVADE handler: updates the grid, invokes D* Lite updatePath, then
     * resumes OUTBOUND (or falls back to RETURN if no path exists).
     */
    void HandleThreatDetection(const Coordinates& obstacleCell);

private:
    // ── Injected dependencies ─────────────────────────────────────────────────
    OccupancyGrid&        m_grid;
    DroneState&           m_drone;
    SensorModule&         m_sensors;
    IIntelligenceManager& m_intelMgr;
    MissionView&          m_view;

    // ── Mission geometry ──────────────────────────────────────────────────────
    Coordinates               m_startPoint;
    Coordinates               m_targetPoint;

    // ── Active path state ─────────────────────────────────────────────────────
    std::vector<Coordinates>  m_currentPath;   ///< Ordered waypoints (inclusive)
    int                       m_pathIndex;     ///< Index of the NEXT waypoint to visit

    // ── Planner components ────────────────────────────────────────────────────
    CostCalculator                    m_costCalc;
    AStarPlanner                      m_astar;
    std::unique_ptr<DStarLitePlanner> m_dstar;  ///< Constructed in InitializeMission

    // ── External configuration ────────────────────────────────────────────────
    MissionConfig m_config;

    // ── Sensor fusion (1D Kalman per axis) ────────────────────────────────────
    KalmanFilter  m_kalmanX;
    KalmanFilter  m_kalmanY;

    // ── FSM ───────────────────────────────────────────────────────────────────
    MissionState  m_state;
    bool          m_missionSuccess;

    // ── Tuning constants ──────────────────────────────────────────────────────
    static constexpr double BATTERY_DRAIN_PER_STEP = 1.0;    ///< % consumed per waypoint
    static constexpr double BATTERY_FAILSAFE       = 20.0;   ///< % threshold → force RETURN
    static constexpr int    MAX_LOOP_ITERATIONS    = 10000;  ///< Hard safety cap

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * Replaces the active path and resets the path index, skipping the first
     * waypoint (which equals the drone's current position).
     */
    void setActivePath(std::vector<Coordinates> newPath);

    /**
     * Checks battery level and GPS availability.
     * @return true if a fail-safe condition is active.
     */
    bool checkFailSafe() const;

    /**
     * Returns true when grid-integer coordinates of a and b match.
     * Used for arrival detection on integer-spaced grids.
     */
    static bool coordsMatch(const Coordinates& a, const Coordinates& b);

    /** Encodes rawPayload via Base64, wraps in AIGD::IntelData, stores it. */
    void captureAndStoreImint(const Coordinates& location,
                              const std::string& rawPayload);

    /** Returns a human-readable string for the given FSM state. */
    static std::string stateToString(MissionState state);

};

} // namespace AIGD
