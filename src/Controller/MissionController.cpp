#include "MissionController.h"
#include "../Utils/Base64Codec.h"
#include "../Utils/ConfigParser.h"
#include "../Utils/ReportGenerator.h"
#include "../Model/IntelligenceManager.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace AIGD {

// ══════════════════════════════════════════════════════════════════════════════
// SensorModule — Simulated EO/Thermal Sensor Suite
// ══════════════════════════════════════════════════════════════════════════════
//
// In a production deployment this class is replaced by a real sensor driver
// (e.g. FLIR Tau 2 for thermal imaging, Sony Alpha for EO).  The public
// interface — constructor + sense() — remains identical so no other component
// requires modification when the sensor layer is swapped (modularity principle).
//
// Simulation design decisions:
//   Position noise  — drawn from N(0, SENSOR_NOISE_STD²) using a seeded
//                     Mersenne-Twister (std::mt19937).  A fixed seed guarantees
//                     deterministic, reproducible test runs.
//   Obstacle injection — fires exactly once at iteration `obstacleStep` to
//                     simulate a mid-flight threat discovery that exercises
//                     the D* Lite incremental replanning path.
// ──────────────────────────────────────────────────────────────────────────────

// ── SensorModule::SensorModule ────────────────────────────────────────────────
//
// Constructs the sensor suite with a single default obstacle trigger built from
// the constructor arguments.  This preserves full backward compatibility: any
// call site that passes (seed, obstacleStep, obstacleOffX) receives a
// SensorModule with exactly one trigger event at the given step and x-offset.
//
// Additional triggers (Phase 2 dynamic obstacle events) are registered after
// construction via addObstacleTrigger().  The m_triggers and m_triggered
// vectors grow in lockstep, maintaining the 1:1 index relationship required
// by the sense() iterator.
// ──────────────────────────────────────────────────────────────────────────────
SensorModule::SensorModule(unsigned int seed,
                           int          obstacleStep,
                           double       obstacleOffX)
    : m_rng(seed)
    , m_posDist(0.0, SENSOR_NOISE_STD)   // N(0, σ²) where σ = SENSOR_NOISE_STD
    , m_triggers()
    , m_triggered()
{
    ObstacleTrigger first;
    first.step = obstacleStep;
    first.offX = obstacleOffX;
    first.offY = 0.0;
    m_triggers.push_back(first);
    m_triggered.push_back(false);
}

// ── SensorModule::addObstacleTrigger ─────────────────────────────────────────
//
// Registers a new mid-flight obstacle injection event by appending an
// ObstacleTrigger entry to m_triggers and a corresponding false entry to
// m_triggered.  The two vectors are always the same length; sense() iterates
// them with a unified index so no separate synchronisation is needed.
//
// Design rationale: keeping trigger registration as a post-construction call
// (rather than a constructor parameter) allows the runMission() orchestrator to
// compose the multi-event schedule declaratively after reading config values,
// without needing to know at construction time how many triggers will be added.
// ──────────────────────────────────────────────────────────────────────────────
void SensorModule::addObstacleTrigger(int step, double offX, double offY) {
    ObstacleTrigger t;
    t.step = step;
    t.offX = offX;
    t.offY = offY;
    m_triggers.push_back(t);
    m_triggered.push_back(false);
}

// ── SensorModule::sense ───────────────────────────────────────────────────────
//
// Produces one complete sensor frame per Sense-Think-Act iteration.
//
// Multi-trigger obstacle injection (Phase 2 design):
//
//   The m_triggers vector is scanned linearly each iteration.  The compound
//   loop condition (i < size && !reading.obstacleDetected) enforces a strict
//   "at most one detection per frame" rule — real EO/Thermal sensors report a
//   single discrete detection per frame, not a simultaneous burst.
//
//   Each trigger fires exactly once: when stepIndex matches the trigger's step
//   AND m_triggered[i] is still false.  Setting m_triggered[i] = true prevents
//   re-firing in subsequent iterations at the same or higher step indices.
//
// Noisy position model:
//   noisyPositionX/Y = truePos + N(0, SENSOR_NOISE_STD²).
//   The Kalman filter in ExecuteSenseThinkActLoop subsequently corrects these
//   noisy GPS/INS readings back toward the true position.
// ──────────────────────────────────────────────────────────────────────────────
SensorReading SensorModule::sense(const Coordinates& truePos, int stepIndex) {
    SensorReading reading;
    reading.noisyPositionX   = truePos.x + m_posDist(m_rng);
    reading.noisyPositionY   = truePos.y + m_posDist(m_rng);
    reading.obstacleDetected = false;

    // Iterate over all scheduled trigger events.  The loop exits as soon as one
    // trigger fires (reading.obstacleDetected becomes true), implementing
    for (int i = 0;
         i < static_cast<int>(m_triggers.size()) && !reading.obstacleDetected;
         ++i)
    {
        if (stepIndex == m_triggers[i].step && !m_triggered[i]) {
            // Trigger condition met: inject a synthetic obstacle at the
            // programmed offset from the drone's current true position.
            reading.obstacleDetected = true;
            reading.obstacleCell     = Coordinates(
                truePos.x + m_triggers[i].offX,
                truePos.y + m_triggers[i].offY,
                truePos.altitude);
            m_triggered[i] = true;
        }
    }

    // Simulate EO frame: ASCII string encoding step index and drone coordinates.
    // In deployment this would be a JPEG/H.264-compressed image buffer.
    std::ostringstream oss;
    oss << "EO_FRAME|step=" << stepIndex
        << "|x=" << std::fixed << std::setprecision(1) << truePos.x
        << "|y=" << truePos.y
        << "|alt=" << truePos.altitude;
    reading.rawImagePayload = oss.str();

    return reading;
}

// ══════════════════════════════════════════════════════════════════════════════
// MissionController — FSM Manager and Sense-Think-Act Orchestrator
// ══════════════════════════════════════════════════════════════════════════════
//
// Architecture role (MVC Controller layer):
//
//   Model layer  →  OccupancyGrid    — probabilistic obstacle map
//                   IIntelligenceManager — IMINT hash-table storage
//                   KalmanFilter     — 1-D sensor fusion per axis
//                   AStarPlanner     — static global path planning
//                   DStarLitePlanner — incremental dynamic replanning
//                   CostCalculator   — weighted cost function (D + R + E)
//
//   View layer   →  MissionView      — real-time ASCII grid + telemetry
//
//   Config layer →  MissionConfig    — Kalman Q/R and priority from config.json
//
// Full FSM lifecycle:
//
//   [IDLE] ──StartMission()──► [OUTBOUND] ──target reached──► [RECON]
//                                   │                              │
//                               obstacle                      IMINT stored
//                               detected                           │
//                                   ▼                         [RETURN]
//                              [EVADE]                             │
//                                   │ D* Lite replanned        arrival
//                                   └──────────────────────► [LANDED]
//
// All dependencies are constructor-injected (non-owning references).
// No global state; the class is fully unit-testable with mock objects.
// ──────────────────────────────────────────────────────────────────────────────

MissionController::MissionController(OccupancyGrid&        grid,
                                     DroneState&           drone,
                                     SensorModule&         sensors,
                                     IIntelligenceManager& intelMgr,
                                     MissionView&          view,
                                     const MissionConfig&  config)
    : m_grid(grid)
    , m_drone(drone)
    , m_sensors(sensors)
    , m_intelMgr(intelMgr)
    , m_view(view)
    , m_startPoint()
    , m_targetPoint()
    , m_pathIndex(0)
    , m_costCalc()
    , m_astar()
    , m_dstar(nullptr)
    , m_config(config)
    , m_kalmanX(0.0, 1.0, config.kalmanQ, config.kalmanR)
    , m_kalmanY(0.0, 1.0, config.kalmanQ, config.kalmanR)
    , m_state(MissionState::IDLE)
    , m_missionSuccess(false)
{
    // Apply the mission-wide cost profile (STEALTH / AGGRESSIVE / BALANCED)
    // to the shared CostCalculator used by both A* and D* Lite planners.
    m_costCalc.setPriority(m_config.priority);
}

// ── InitializeMission ─────────────────────────────────────────────────────────
//
// Pre-flight sequence (six steps):
//
//   1. Store mission geometry (start, target) for later use in path planning.
//
//   2. Re-seed both Kalman filter axes at the true start position so their
//      initial state estimates match physical reality before any noisy sensor
//      observations are introduced.
//      Parameters Q (process noise) and R (measurement noise) come from
//      config.json via MissionConfig.
//
//   3. Stamp all pre-known obstacles onto the OccupancyGrid.
//      The soft Bayesian update requires three consecutive obstacle detections
//      to raise a cell's probability past OCCUPANCY_THRESHOLD (0.7):
//        0.50 → 0.605 → 0.6785 → ~0.730 (occupied)
//      Calling UpdateCell(true) three times achieves this in one shot.
//
//   4. Run A* to compute the initial global path from start to target.
//      Cost function: Cost = w_d * distance + w_r * risk + w_e * energy
//      where weights are determined by the active MissionPriority profile.
//      If A* finds no valid path (fully blocked grid), the FSM is immediately
//      set to LANDED and the function exits — no further initialisation needed.
//
//   5. Construct the DStarLitePlanner.
//      Construction pre-computes the full backward-search g/rhs value surface
//      (a Dijkstra from goal to all reachable cells).  Subsequent calls to
//      updatePath() only reprocess the vertices whose cost paths pass through
//      the newly discovered obstacle — far more efficient than a full re-run.
//
//   6. Set pathIndex = 1 to skip waypoint[0], which equals the start cell
//      the drone is already occupying (both A* and D* Lite return inclusive
//      paths).  Arm FSM to IDLE, ready for StartMission().
// ──────────────────────────────────────────────────────────────────────────────
void MissionController::InitializeMission(
        const Coordinates&              start,
        const Coordinates&              target,
        const std::vector<Coordinates>& knownObstacles)
{
    m_startPoint  = start;
    m_targetPoint = target;

    // Step 2: re-seed Kalman filters at the true start position.
    m_kalmanX = KalmanFilter(start.x, 1.0, m_config.kalmanQ, m_config.kalmanR);
    m_kalmanY = KalmanFilter(start.y, 1.0, m_config.kalmanQ, m_config.kalmanR);

    // Step 3: stamp pre-known obstacles (3 hits each → probability > 0.7).
    for (const Coordinates& obs : knownObstacles) {
        int cx = static_cast<int>(obs.x);
        int cy = static_cast<int>(obs.y);
        m_grid.UpdateCell(cx, cy, true);
        m_grid.UpdateCell(cx, cy, true);
        m_grid.UpdateCell(cx, cy, true);
    }

    // Step 4: compute initial global path with A*.
    m_currentPath = m_astar.findPath(start, target, m_grid, m_costCalc);

    // Steps 5-6 only execute if A* found a valid path.
    if (!m_currentPath.empty()) {
        // Step 5: construct D* Lite.  Constructor runs a full backward Dijkstra;
        // incremental updates during flight reprocess only affected vertices.
        m_dstar = std::make_unique<DStarLitePlanner>(start, target, m_grid, m_costCalc);

        // Step 6: skip waypoint[0] (= start cell the drone already occupies).
        m_pathIndex = (static_cast<int>(m_currentPath.size()) > 1) ? 1 : 0;
        m_state     = MissionState::IDLE;

        m_view.DisplayAlert("[INIT] Mission ready. A* path: "
                            + std::to_string(m_currentPath.size()) + " waypoints.");
    } else {
        // A* found no valid path — abort mission before departure.
        m_view.DisplayAlert("[INIT] A* found no valid path — mission aborted.");
        m_state = MissionState::LANDED;
    }
}

// ── RunMainLoop ───────────────────────────────────────────────────────────────
//
// Top-level entry point called from main.cpp.
// Delegates to StartMission() (FSM arm) and ExecuteSenseThinkActLoop() (main
// loop) in sequence, then returns the mission outcome flag.
//
// @return  true  — LANDED after a successful RECON + RETURN.
//          false — mission aborted (fail-safe, no path found, max iterations).
// ──────────────────────────────────────────────────────────────────────────────
bool MissionController::RunMainLoop() {
    StartMission();
    ExecuteSenseThinkActLoop();
    return m_missionSuccess;
}

// ── StartMission ──────────────────────────────────────────────────────────────
//
// FSM transition: IDLE → OUTBOUND.
// Guards against out-of-order calls (e.g. calling StartMission() twice or
// while already flying).  A logged alert lets operators diagnose the issue.
// Single-return form: both branches (ignored / executed) are in an if/else.
// ──────────────────────────────────────────────────────────────────────────────
void MissionController::StartMission() {
    if (m_state != MissionState::IDLE) {
        m_view.DisplayAlert("[FSM] StartMission ignored — not in IDLE state.");
    } else {
        m_state = MissionState::OUTBOUND;
        m_view.DisplayAlert("[FSM] IDLE → OUTBOUND: Departing to target.");
    }
}

// ── ExecuteSenseThinkActLoop ──────────────────────────────────────════════════
//
// Core Sense-Think-Act (STA) control loop.
// Executes one full iteration per loop step until the FSM reaches LANDED
// or the hard safety cap MAX_LOOP_ITERATIONS is hit.
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │  SENSE PHASE                                                             │
// │                                                                          │
// │  • SensorModule::sense() produces a raw sensor frame:                   │
// │      - noisyPositionX/Y — GPS/INS readings corrupted by N(0, σ²) noise. │
// │      - obstacleDetected / obstacleCell — mid-flight obstacle flag.       │
// │      - rawImagePayload  — simulated EO image (ASCII string).             │
// │                                                                          │
// │  • Kalman Predict (stationary assumption, u = 0):                       │
// │      x⁻ = x + 0      (state propagates unchanged)                       │
// │      P⁻ = P + Q      (uncertainty grows by process noise Q each step)   │
// │                                                                          │
// │  • Kalman Update (fuse noisy GPS/INS measurement):                      │
// │      K   = P⁻ / (P⁻ + R)   (Kalman gain: balance model vs sensor)      │
// │      x   = x⁻ + K * (z - x⁻)  (state corrected toward measurement)     │
// │      P   = (1 - K) * P⁻    (uncertainty reduced by the gain)            │
// │                                                                          │
// │  • filteredPos — de-noised Kalman estimate used as the D* Lite           │
// │    replanning origin (more accurate than raw GPS for short segments).    │
// ├─────────────────────────────────────────────────────────────────────────┤
// │  THINK PHASE                                                             │
// │                                                                          │
// │  • checkFailSafe(): if battery < BATTERY_FAILSAFE (20%) or GPS is lost, │
// │    compute an emergency A* return path and force FSM to RETURN.          │
// │    The EVADE and RETURN states are excluded to prevent the fail-safe     │
// │    from interrupting an already-active emergency procedure.              │
// │                                                                          │
// │  • Obstacle detection: if OUTBOUND and a new obstacle was sensed,        │
// │    delegate to HandleThreatDetection (→ EVADE → D* Lite replan).        │
// │                                                                          │
// │  • Battery drain: BATTERY_DRAIN_PER_STEP (1%) per active movement step. │
// │    Only OUTBOUND and RETURN deplete the battery; RECON and EVADE do not. │
// ├─────────────────────────────────────────────────────────────────────────┤
// │  ACT PHASE                                                               │
// │                                                                          │
// │  • OUTBOUND: advance drone one cell along m_currentPath.                 │
// │    Post-move Kalman Predict corrects the filter for the executed         │
// │    displacement (Δx, Δy) so the next iteration starts from an           │
// │    accurate state estimate.                                              │
// │    Arrival → RECON.                                                      │
// │                                                                          │
// │  • RECON: capture EO payload → Base64-encode → store in IntelMgr.       │
// │    Plan return path with A* (grid may contain new obstacles from EVADE). │
// │    Transition → RETURN (or LANDED if no return path exists).            │
// │                                                                          │
// │  • RETURN: advance drone back toward start along the return path.        │
// │    Arrival at start → LANDED + missionSuccess = true.                   │
// └─────────────────────────────────────────────────────────────────────────┘
// ──────────────────────────────────────────────────────────────────────────────
void MissionController::ExecuteSenseThinkActLoop() {
    int iteration = 0;

    while (m_state != MissionState::LANDED && iteration < MAX_LOOP_ITERATIONS) {
        ++iteration;

        // ══════════════════════════════════════════════════════════════════════
        // SENSE PHASE — acquire sensor data and fuse with Kalman filter
        // ══════════════════════════════════════════════════════════════════════

        // Obtain one complete sensor frame: noisy GPS/INS position,
        // optional obstacle detection, and a simulated EO image payload.
        SensorReading reading =
            m_sensors.sense(m_drone.position, iteration);

        // Kalman Predict step: propagate state forward assuming no commanded
        // displacement (stationary model).  Covariance grows by Q: P = P + Q.
        m_kalmanX.Predict({0.0});
        m_kalmanY.Predict({0.0});

        // Kalman Update step: fuse the noisy GPS/INS measurement.
        // Gain K = P/(P+R) weights sensor reliability against model uncertainty.
        // State update: x = x + K*(z - x).  Covariance update: P = (1-K)*P.
        m_kalmanX.Update({reading.noisyPositionX});
        m_kalmanY.Update({reading.noisyPositionY});

        // filteredPos: de-noised position used as the D* Lite replanning origin.
        // Using the Kalman output rather than the raw GPS prevents the replanner
        // from starting from a position skewed by sensor noise.
        Coordinates filteredPos(m_kalmanX.getState(),
                                m_kalmanY.getState(),
                                m_drone.position.altitude);

        // ══════════════════════════════════════════════════════════════════════
        // THINK PHASE — evaluate safety conditions and plan responses
        // ══════════════════════════════════════════════════════════════════════

        // Fail-safe guard: battery critical or GPS lost → abort to RETURN.
        // EVADE and RETURN are excluded so an active emergency replan is not
        // re-interrupted by the same fail-safe it is already responding to.
        if (checkFailSafe() &&
            m_state != MissionState::RETURN &&
            m_state != MissionState::EVADE)
        {
            m_view.DisplayAlert("[FAIL-SAFE] Emergency return initiated.");
            m_currentPath = m_astar.findPath(filteredPos, m_startPoint,
                                              m_grid, m_costCalc);
            setActivePath(std::move(m_currentPath));
            m_state = MissionState::RETURN;
        }

        // Dynamic obstacle response: if a new obstacle is sensed on the
        // outbound leg, delegate to HandleThreatDetection which invokes
        // D* Lite to compute an alternative route around the blocked cell.
        if (m_state == MissionState::OUTBOUND && reading.obstacleDetected) {
            HandleThreatDetection(reading.obstacleCell);
        }

        // Battery drain: BATTERY_DRAIN_PER_STEP percent per movement step.
        // RECON and EVADE states do not advance waypoints so they are excluded.
        if (m_state == MissionState::OUTBOUND || m_state == MissionState::RETURN) {
            m_drone.batteryLevel -= BATTERY_DRAIN_PER_STEP;
        }

        // ══════════════════════════════════════════════════════════════════════
        // ACT PHASE — execute movement, IMINT collection, and state transitions
        // ══════════════════════════════════════════════════════════════════════

        // ── OUTBOUND: advance to next waypoint ──────────────────────────────
        // The drone moves one grid cell per iteration along m_currentPath.
        // After the move, a secondary Kalman Predict corrects the filter for
        // the displacement just executed so the next SENSE phase starts from
        // an accurate internal model state.
        if (m_state == MissionState::OUTBOUND) {
            if (m_pathIndex < static_cast<int>(m_currentPath.size())) {
                m_drone.position = m_currentPath[m_pathIndex];
                ++m_pathIndex;

                // Post-move Kalman Predict: account for the actual displacement
                // just executed (filteredPos was the pre-move Kalman estimate).
                m_kalmanX.Predict({m_drone.position.x - filteredPos.x});
                m_kalmanY.Predict({m_drone.position.y - filteredPos.y});

                m_view.UpdateDronePosition(m_drone.position);
                m_view.displayGrid(m_grid, m_drone.position, m_targetPoint, m_currentPath);
                m_view.displayTelemetry(m_drone.batteryLevel, m_state, m_drone.position);

                // Arrival check: compare integer-truncated grid coordinates.
                if (coordsMatch(m_drone.position, m_targetPoint)) {
                    m_state = MissionState::RECON;
                    m_view.DisplayAlert("[FSM] OUTBOUND → RECON: Target reached.");
                }
            } else {
                // Path exhausted — A* guarantees last waypoint equals the target.
                m_state = MissionState::RECON;
                m_view.DisplayAlert("[FSM] OUTBOUND → RECON: Path complete.");
            }
        }

        // ── RECON: capture intelligence at the target ───────────────────────
        // The EO image payload is Base64-encoded (communication protocol per
        // project spec) and stored in the IntelligenceManager hash table for
        // O(1) insertion keyed by Unix timestamp in milliseconds.
        // A fresh A* path is then computed for the return leg using the
        // most up-to-date obstacle map (which may include new cells from EVADE).
        if (m_state == MissionState::RECON) {
            captureAndStoreImint(m_drone.position, reading.rawImagePayload);
            m_view.DisplayAlert("[RECON] IMINT captured at target.");

            std::vector<Coordinates> returnPath =
                m_astar.findPath(m_drone.position, m_startPoint, m_grid, m_costCalc);

            if (returnPath.empty()) {
                // No viable return route — land at current position.
                m_view.DisplayAlert("[RECON] No return path — emergency landing.");
                m_state = MissionState::LANDED;
            } else {
                setActivePath(returnPath);
                m_state = MissionState::RETURN;
                m_view.DisplayAlert("[FSM] RECON → RETURN: Heading home.");
            }
        }

        // ── RETURN: advance back to start along the return path ─────────────
        // Mirrors OUTBOUND movement logic but targets m_startPoint.
        // Mission is declared successful only when the drone physically arrives
        // at the start cell (coordsMatch), not just when the path is exhausted.
        if (m_state == MissionState::RETURN) {
            if (m_pathIndex < static_cast<int>(m_currentPath.size())) {
                m_drone.position = m_currentPath[m_pathIndex];
                ++m_pathIndex;

                m_view.UpdateDronePosition(m_drone.position);
                m_view.displayGrid(m_grid, m_drone.position, m_startPoint, m_currentPath);
                m_view.displayTelemetry(m_drone.batteryLevel, m_state, m_drone.position);

                if (coordsMatch(m_drone.position, m_startPoint)) {
                    m_state          = MissionState::LANDED;
                    m_missionSuccess = true;
                    m_view.DisplayAlert("[FSM] RETURN → LANDED: Mission complete.");
                }
            } else {
                // Return path exhausted — A* final node equals the start cell.
                m_state          = MissionState::LANDED;
                m_missionSuccess = true;
                m_view.DisplayAlert("[FSM] LANDED: Return path complete.");
            }
        }
    }

    // Safety net: force LANDED if the iteration cap was reached.
    // Prevents infinite loops in pathological edge cases (e.g. path cycles
    // caused by floating-point drift on non-integer grid coordinates).
    if (iteration >= MAX_LOOP_ITERATIONS) {
        m_view.DisplayAlert("[SAFETY] Max iterations reached — forcing LANDED.");
        m_state = MissionState::LANDED;
    }
}

// ── HandleThreatDetection ─────────────────────────────────────────────────────
//
// Triggered by the THINK phase when the sensor detects a new obstacle on the
// outbound leg.  Procedure:
//
//   1. Transition FSM to EVADE (recorded in telemetry log).
//
//   2. Call D* Lite updatePath():
//      - Internally applies 3 Bayesian UpdateCell(true) calls on the obstacle
//        cell, raising its probability past 0.7 (impassable threshold).
//      - Reprocesses only the affected vertices in the open list (those whose
//        minimum-cost path passes through the now-blocked edge) — incremental
//        efficiency vs a full Dijkstra rebuild.
//      - Returns a replanned path from the drone's current cell to the
//        original target, or an empty vector if the target is unreachable.
//
//   3. If a detour exists: load the replanned path and resume OUTBOUND.
//      If the target is completely surrounded: fall back to A* for an
//      emergency direct return to base (RETURN state).
//
//   4. If D* Lite was not constructed (InitializeMission failed or was
//      skipped): fall back directly to RETURN.
// ──────────────────────────────────────────────────────────────────────────────
void MissionController::HandleThreatDetection(const Coordinates& obstacleCell) {
    m_state = MissionState::EVADE;
    m_view.DisplayAlert("[FSM] OUTBOUND → EVADE: Obstacle at ("
        + std::to_string(static_cast<int>(obstacleCell.x)) + ", "
        + std::to_string(static_cast<int>(obstacleCell.y)) + ").");

    if (m_dstar) {
        // D* Lite marks the obstacle (3 Bayesian hits internally) and reprocesses
        // only the affected vertices for efficient incremental replanning.
        std::vector<Coordinates> replanned =
            m_dstar->updatePath(m_drone.position, obstacleCell,
                                m_grid, m_costCalc);

        if (replanned.empty()) {
            // No route around the obstacle — abort outbound leg, return to base.
            m_view.DisplayAlert("[EVADE] D* Lite: no path — aborting to RETURN.");
            std::vector<Coordinates> retreat =
                m_astar.findPath(m_drone.position, m_startPoint,
                                 m_grid, m_costCalc);
            setActivePath(retreat);
            m_state = MissionState::RETURN;
        } else {
            // Valid detour found — load it and resume outbound navigation.
            setActivePath(replanned);
            m_state = MissionState::OUTBOUND;
            m_view.DisplayAlert("[FSM] EVADE → OUTBOUND: Replanned ("
                + std::to_string(m_currentPath.size()) + " waypoints).");
        }
    } else {
        // D* Lite not constructed (InitializeMission was skipped or failed).
        m_view.DisplayAlert("[EVADE] D* Lite not initialised — returning to base.");
        m_state = MissionState::RETURN;
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

// Replaces the active path and resets the path index.
// Skips waypoint[0] because A* and D* Lite both return paths inclusive of
// the origin cell — the drone is already at that position.
void MissionController::setActivePath(std::vector<Coordinates> newPath) {
    m_currentPath = std::move(newPath);
    m_pathIndex = (static_cast<int>(m_currentPath.size()) > 1) ? 1 : 0;
}

// Evaluates fail-safe conditions in priority order.
// Battery is checked first; if it fails, the GPS check is skipped to prevent
// duplicate alerts for different conditions in the same iteration.
// Single-return form: the else-if chain updates failSafe and a single
// return statement at the end delivers the result.
bool MissionController::checkFailSafe() const {
    bool failSafe = false;
    if (m_drone.batteryLevel < BATTERY_FAILSAFE) {
        m_view.DisplayAlert("[FAIL-SAFE] Battery at "
            + std::to_string(static_cast<int>(m_drone.batteryLevel)) + "%.");
        failSafe = true;
    } else if (!m_drone.gpsAvailable) {
        m_view.DisplayAlert("[FAIL-SAFE] GPS signal lost.");
        failSafe = true;
    }
    return failSafe;
}

// Returns true when integer grid coordinates of a and b match.
// Grid cells are 1×1 unit squares; integer truncation is the correct
// comparison for arrival detection (floating-point equality is unreliable
// due to accumulated Kalman filter corrections and movement noise).
bool MissionController::coordsMatch(const Coordinates& a, const Coordinates& b) {
    return static_cast<int>(a.x) == static_cast<int>(b.x) &&
           static_cast<int>(a.y) == static_cast<int>(b.y);
}

// Base64-encodes the raw EO payload, wraps it in an IntelData record stamped
// with the current Unix timestamp in milliseconds, and stores it in the
// IntelligenceManager's hash table (O(1) insertion keyed by timestamp).
// Base64 encoding is the project-specified communication protocol for
// transferring image data over the drone uplink to the ground station.
void MissionController::captureAndStoreImint(const Coordinates& location,
                                              const std::string& rawPayload)
{
    long long ts        = static_cast<long long>(std::time(nullptr)) * 1000;
    std::string encoded = Base64Codec::encode(rawPayload);
    Point2D     loc2d(location.x, location.y);

    IntelData intel(loc2d, ts, SensorType::IMINT, encoded);
    m_intelMgr.addIntel(intel);

    m_view.DisplayAlert("[IMINT] Stored @ ("
        + std::to_string(static_cast<int>(location.x)) + ", "
        + std::to_string(static_cast<int>(location.y)) + ") — "
        + std::to_string(encoded.size()) + " B encoded.");
}

// Maps a MissionState enum value to its human-readable name string.
// Single-return form: result is initialised to "UNKNOWN" (safe default for
// any future enum values not yet covered) and updated by the else-if chain.
std::string MissionController::stateToString(MissionState state) {
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
// MissionController::runMission — Static Mission Orchestrator
// ══════════════════════════════════════════════════════════════════════════════
//
// Top-level entry point for the autonomous IMINT pipeline.  All mission state
// is declared locally; no global variables are used.  The function is static so
// main() (or any test harness) can call it without an existing controller object.
//
// The local variable `outcome` is declared false at the top.  Every code path
// sets it via assignment; the sole `return outcome;` sits at the absolute
// final line of the function body.  No return statement appears inside any
// if, else, or loop block.
//
// Two-phase challenge architecture:
//
//   ┌─────────────────────────────────────────────────────────────────────────┐
//   │ PHASE 1 — Static Obstacle Maze (injected before InitializeMission)       │
//   │                                                                           │
//   │  Grid layout (10×10, start=(0,5), target=(9,5)):                         │
//   │                                                                           │
//   │   y  0  1  2  3  4  5  6  7  8  9                                        │
//   │   0  .  .  .  .  .  .  .  .  .  .                                        │
//   │   1  .  .  .  .  .  .  .  .  .  .                                        │
//   │   2  .  .  .  .  .  X  .  .  .  .   ← Wall B starts                     │
//   │   3  .  .  .  X  .  X  .  .  .  .   ← Wall A and B overlap              │
//   │   4  .  .  .  X  .  X  .  .  .  .                                        │
//   │   5  S  .  .  X  .  X  .  .  .  T   ← straight path fully blocked       │
//   │   6  .  .  .  X  .  X  .  .  .  .                                        │
//   │   7  .  .  .  X  .  .  .  .  .  .   ← Wall A ends, Wall B already ended  │
//   │   8  .  .  .  .  .  .  .  .  .  .   ← bottom corridor: y=8 clear for both│
//   │   9  .  .  .  .  .  .  .  .  .  .                                        │
//   │                                                                           │
//   │  A* (STEALTH) selects the 15-step bottom corridor: y=8 from x=0..7,     │
//   │  then ascends y=8→5 at x=7, then right to (9,5).                        │
//   └─────────────────────────────────────────────────────────────────────────┘
//
//   ┌─────────────────────────────────────────────────────────────────────────┐
//   │ PHASE 2 — Dynamic Obstacle Events (mid-flight SensorModule triggers)      │
//   │                                                                           │
//   │  Trigger 1 at STA step 5:                                                 │
//   │    Drone at (1,8) after 4 waypoint advances along the bottom corridor.    │
//   │    Obstacle injected at (1+2, 8) = (3,8) — the gap cell of Wall A.       │
//   │    → D* Lite REPLANNING 1: seals bottom gap; route shifts to y=9 row.    │
//   │                                                                           │
//   │  Trigger 2 at STA step 10:                                                │
//   │    Drone at (5,9) after 4 more advances on the new y=9 path.             │
//   │    Obstacle injected at (5+2, 9) = (7,9) — directly on the active path.  │
//   │    → D* Lite REPLANNING 2: detours via (6,9)→(6,8)→(7,8)→…→(9,5).       │
//   └─────────────────────────────────────────────────────────────────────────┘
// ──────────────────────────────────────────────────────────────────────────────
bool MissionController::runMission(const std::string& configFilePath) {
    // Single return value — all logic sets this variable; return is at line end.
    bool outcome = false;

    // ── Step 1: Construct the View — the only source of console output ────────
    // MissionView is constructed before any other component so that the banner
    // and config file path can be rendered before ConfigParser runs.  This
    // ensures every std::cout in this function is replaced by a View call and
    // the Controller itself contains zero direct output statements.
    MissionView missionView;
    missionView.printOrchestratorBanner(configFilePath);

    // ── Step 2: Parse external JSON configuration ──────────────────────────
    // ConfigParser reads the JSON file and populates an AppConfig struct with
    // Kalman Q/R noise parameters, grid dimensions and Base64 map payload,
    // mission start/target coordinates, and navigation priority string.
    // Throws std::runtime_error if the file cannot be opened.
    ConfigParser configParser;
    AppConfig    appConfig = configParser.parse(configFilePath);

    missionView.printConfigSummary(
        appConfig.kalmanFilter.Q,  appConfig.kalmanFilter.R,
        appConfig.mapData.width,   appConfig.mapData.height,
        appConfig.navigationPriority);

    // ── Step 2: Decode Base64 map and initialise OccupancyGrid ────────────
    // The Base64 map payload encodes the mission area as a flat ASCII string.
    // '#' characters indicate pre-surveyed obstacles; '.' indicates open space.
    // Three consecutive UpdateCell(true) calls push Bayesian probability above
    // OCCUPANCY_THRESHOLD (0.7): 0.50 → 0.605 → 0.6785 → ~0.730 (impassable).
    const std::string flatMap = ConfigParser::decodeBase64(appConfig.mapData.gridBase64);
    OccupancyGrid missionGrid(appConfig.mapData.width, appConfig.mapData.height);

    for (int gy = 0; gy < appConfig.mapData.height; ++gy) {
        for (int gx = 0; gx < appConfig.mapData.width; ++gx) {
            int idx = gy * appConfig.mapData.width + gx;
            if (idx < static_cast<int>(flatMap.size()) && flatMap[idx] == '#') {
                for (int hit = 0; hit < STATIC_HITS_PER_CELL; ++hit) {
                    missionGrid.UpdateCell(gx, gy, true);
                }
            }
        }
    }

    // ── Step 3: PHASE 1 — Inject static obstacle barriers ─────────────────
    // Two vertical walls are stamped into the grid BEFORE InitializeMission()
    // runs A*.  This guarantees A* sees the full obstacle layout and plans a
    // realistic detour path rather than a straight horizontal route.
    //
    // Wall A (column WALL_A_COL, rows WALL_A_Y_MIN..WALL_A_Y_MAX):
    //   Blocks the direct corridor at x=3, forcing the drone above or below.
    //
    // Wall B (column WALL_B_COL, rows WALL_B_Y_MIN..WALL_B_Y_MAX):
    //   Blocks x=6 from y=2 to y=6.  The narrow top gap (y<2) and wide bottom
    //   gap (y>=7) interact with Wall A's gaps to create a forced corridor:
    //   the only efficient route threads through the bottom (y=8 or y=9).
    //
    // Each cell requires STATIC_HITS_PER_CELL=3 UpdateCell(true) calls to
    // exceed the Bayesian OCCUPANCY_THRESHOLD of 0.7 and become impassable.
    missionView.printPhaseOneObstacles(WALL_A_COL, WALL_A_Y_MIN, WALL_A_Y_MAX,
                                        WALL_B_COL, WALL_B_Y_MIN, WALL_B_Y_MAX);

    for (int row = WALL_A_Y_MIN; row <= WALL_A_Y_MAX; ++row) {
        for (int hit = 0; hit < STATIC_HITS_PER_CELL; ++hit) {
            missionGrid.UpdateCell(WALL_A_COL, row, true);
        }
    }

    for (int row = WALL_B_Y_MIN; row <= WALL_B_Y_MAX; ++row) {
        for (int hit = 0; hit < STATIC_HITS_PER_CELL; ++hit) {
            missionGrid.UpdateCell(WALL_B_COL, row, true);
        }
    }

    missionView.printOccupancyGrid(missionGrid);

    // ── Step 4: Build mission coordinate objects ───────────────────────────
    // Coordinates are read directly from the parsed AppConfig, preserving the
    // altitude field for 3-D position tracking in the Kalman filter and view.
    const Coordinates mStart(
        appConfig.missionPoints.start.x,
        appConfig.missionPoints.start.y,
        appConfig.missionPoints.start.altitude);

    const Coordinates mTarget(
        appConfig.missionPoints.target.x,
        appConfig.missionPoints.target.y,
        appConfig.missionPoints.target.altitude);

    // ── Step 5: Derive MissionConfig from parsed values ───────────────────
    // Convert the navigation priority string ("STEALTH" / "AGGRESSIVE" /
    // "BALANCED") to the MissionPriority enum expected by MissionController.
    // Default to BALANCED so the system degrades gracefully on unexpected input.
    MissionPriority navPriority = MissionPriority::BALANCED;
    if (appConfig.navigationPriority == "STEALTH") {
        navPriority = MissionPriority::STEALTH;
    } else if (appConfig.navigationPriority == "AGGRESSIVE") {
        navPriority = MissionPriority::AGGRESSIVE;
    }

    MissionConfig missionConfig;
    missionConfig.kalmanQ  = appConfig.kalmanFilter.Q;
    missionConfig.kalmanR  = appConfig.kalmanFilter.R;
    missionConfig.priority = navPriority;

    // ── Step 6: Construct all MVC components ──────────────────────────────
    // Drone: positioned at mission start with 100% battery charge.
    DroneState drone(mStart, 100.0);

    // PHASE 2 sensor configuration:
    //   The SensorModule is initialised with the first dynamic obstacle trigger
    //   (step DYNAMIC_TRIGGER_1_STEP, DYNAMIC_TRIGGER_OFF_X cells ahead).
    //   A second trigger is then appended via addObstacleTrigger() for
    //   step DYNAMIC_TRIGGER_2_STEP.  Both triggers fire on the drone's active
    //   path, forcing D* Lite to replan twice during the OUTBOUND leg:
    //     Trigger 1 expected result: obstacle at (3,8) — seals Wall A bottom gap.
    //     Trigger 2 expected result: obstacle at (7,9) — blocks the y=9 detour.
    SensorModule sensors(/*seed=*/42,
                         /*obstacleStep=*/DYNAMIC_TRIGGER_1_STEP,
                         /*obstacleOffX=*/DYNAMIC_TRIGGER_OFF_X);
    sensors.addObstacleTrigger(DYNAMIC_TRIGGER_2_STEP,
                               DYNAMIC_TRIGGER_OFF_X,
                               DYNAMIC_TRIGGER_OFF_Y);

    missionView.printPhaseTwoTriggers(DYNAMIC_TRIGGER_1_STEP, DYNAMIC_TRIGGER_OFF_X,
                                       DYNAMIC_TRIGGER_2_STEP, DYNAMIC_TRIGGER_OFF_X);

    IntelligenceManager missionIntelMgr;

    MissionController controller(
        missionGrid, drone, sensors, missionIntelMgr, missionView, missionConfig);

    // ── Step 7: Pre-flight initialisation ─────────────────────────────────
    // InitializeMission() runs A* on the current grid (with both static walls
    // already injected), constructs the D* Lite backward cost surface, and sets
    // the FSM to IDLE.  No pre-known obstacles list is passed because the walls
    // were already stamped directly onto missionGrid above.
    controller.InitializeMission(mStart, mTarget, {} /* no extra known obstacles */);

    missionView.printRouteDetails(mStart, mTarget);

    // ── Step 8: Execute the Sense-Think-Act mission loop ──────────────────
    // RunMainLoop() calls StartMission() (IDLE → OUTBOUND) then runs the full
    // Sense-Think-Act loop until LANDED or MAX_LOOP_ITERATIONS is reached.
    // Phase 2 triggers fire from inside ExecuteSenseThinkActLoop at the
    // configured step indices, each causing HandleThreatDetection to call
    // D* Lite updatePath() for incremental replanning.
    bool missionSuccess = controller.RunMainLoop();

    // Gather intel summary data, then delegate all formatting to the View.
    // payloadSize is 0 when the intel store is empty; printMissionSummary
    // suppresses the payload line in that case (if block in the View, no return).
    const auto allIntel   = missionIntelMgr.getAllIntel();
    int        payloadSize = 0;
    if (!allIntel.empty()) {
        payloadSize = static_cast<int>(allIntel[0].getRawData().size());
    }
    missionView.printMissionSummary(
        missionSuccess,
        drone.batteryLevel,
        static_cast<int>(allIntel.size()),
        payloadSize);

    // ── Step 9: Generate structured JSON intelligence report ──────────────
    // ReportGenerator::generateReport() validates each Base64 payload, serialises
    // all IntelData records to prettified JSON, and writes the file.
    // Returns false if the file cannot be created or any validation fails.
    bool reportOk = ReportGenerator::generateReport(
        MISSION_REPORT_FILE, missionIntelMgr, missionSuccess);

    missionView.printReportStatus(reportOk);
    missionView.printOrchestratorComplete();

    // Single return point: true iff both the flight succeeded and the report
    // was written without error.  The sole return statement is here at the
    // absolute final line of the function body.
    outcome = (missionSuccess && reportOk);
    return outcome;
}

} // namespace AIGD
