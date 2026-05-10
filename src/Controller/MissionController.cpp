#include "MissionController.h"
#include "../Utils/Base64Codec.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace AIGD {

// ── SensorModule ──────────────────────────────────────────────────────────────

SensorModule::SensorModule(unsigned int seed,
                           int          obstacleStep,
                           double       obstacleOffX)
    : m_rng(seed)
    , m_posDist(0.0, NOISE_STD)
    , m_obstacleStep(obstacleStep)
    , m_obstacleOffX(obstacleOffX)
    , m_obstacleTriggered(false)
{}

SensorReading SensorModule::sense(const Coordinates& truePos, int stepIndex) {
    SensorReading reading;
    reading.noisyPositionX  = truePos.x + m_posDist(m_rng);
    reading.noisyPositionY  = truePos.y + m_posDist(m_rng);
    reading.obstacleDetected = false;

    // Inject the simulated obstacle exactly once at the configured step
    if (stepIndex == m_obstacleStep && !m_obstacleTriggered) {
        reading.obstacleDetected = true;
        reading.obstacleCell     = Coordinates(truePos.x + m_obstacleOffX,
                                               truePos.y,
                                               truePos.altitude);
        m_obstacleTriggered = true;
    }

    // Simulated IMINT frame: ASCII placeholder, unique per step and position
    std::ostringstream oss;
    oss << "EO_FRAME|step=" << stepIndex
        << "|x=" << std::fixed << std::setprecision(1) << truePos.x
        << "|y=" << truePos.y
        << "|alt=" << truePos.altitude;
    reading.rawImagePayload = oss.str();

    return reading;
}

// ── MissionController — construction ─────────────────────────────────────────

MissionController::MissionController(OccupancyGrid&        grid,
                                     DroneState&           drone,
                                     SensorModule&         sensors,
                                     IIntelligenceManager& intelMgr,
                                     MissionView&          view)
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
    // Q = 0.01, R = σ² = 0.09 (σ = NOISE_STD = 0.3 m)
    , m_kalmanX(0.0, 1.0, 0.01, 0.09)
    , m_kalmanY(0.0, 1.0, 0.01, 0.09)
    , m_state(MissionState::IDLE)
    , m_missionSuccess(false)
{}

// ── InitializeMission ─────────────────────────────────────────────────────────

void MissionController::InitializeMission(
        const Coordinates&              start,
        const Coordinates&              target,
        const std::vector<Coordinates>& knownObstacles)
{
    m_startPoint  = start;
    m_targetPoint = target;

    // Re-seed Kalman filters at the true start position
    m_kalmanX = KalmanFilter(start.x, 1.0, 0.01, 0.09);
    m_kalmanY = KalmanFilter(start.y, 1.0, 0.01, 0.09);

    // Mark pre-known obstacle cells (3 Bayesian updates exceed the 0.7 threshold)
    for (const Coordinates& obs : knownObstacles) {
        const int cx = static_cast<int>(obs.x);
        const int cy = static_cast<int>(obs.y);
        m_grid.UpdateCell(cx, cy, true);
        m_grid.UpdateCell(cx, cy, true);
        m_grid.UpdateCell(cx, cy, true);
    }

    // Compute initial global path with A*
    m_currentPath = m_astar.findPath(start, target, m_grid, m_costCalc);

    if (m_currentPath.empty()) {
        m_view.DisplayAlert("[INIT] A* found no valid path — mission aborted.");
        m_state = MissionState::LANDED;
        return;
    }

    // Construct D* Lite planner so it holds the full backward-search state
    // for incremental replanning when obstacles are discovered mid-flight.
    m_dstar = std::make_unique<DStarLitePlanner>(start, target, m_grid, m_costCalc);

    // Skip waypoint[0] — it equals the start position the drone is already at
    m_pathIndex = (static_cast<int>(m_currentPath.size()) > 1) ? 1 : 0;
    m_state     = MissionState::IDLE;

    m_view.DisplayAlert("[INIT] Mission ready. A* path: "
                        + std::to_string(m_currentPath.size()) + " waypoints.");
}

// ── RunMainLoop ───────────────────────────────────────────────────────────────

bool MissionController::RunMainLoop() {
    StartMission();
    ExecuteSenseThinkActLoop();
    return m_missionSuccess;
}

// ── StartMission ──────────────────────────────────────────────────────────────

void MissionController::StartMission() {
    if (m_state != MissionState::IDLE) {
        m_view.DisplayAlert("[FSM] StartMission ignored — not in IDLE state.");
        return;
    }
    m_state = MissionState::OUTBOUND;
    m_view.DisplayAlert("[FSM] IDLE → OUTBOUND: Departing to target.");
}

// ── ExecuteSenseThinkActLoop ──────────────────────────────────────────────────

void MissionController::ExecuteSenseThinkActLoop() {
    int iteration = 0;

    while (m_state != MissionState::LANDED && iteration < MAX_LOOP_ITERATIONS) {
        ++iteration;

        // ── SENSE ─────────────────────────────────────────────────────────────
        const SensorReading reading =
            m_sensors.sense(m_drone.position, iteration);

        // Kalman predict (stationary assumption) then correct with noisy GPS/INS
        m_kalmanX.Predict({0.0});
        m_kalmanY.Predict({0.0});
        m_kalmanX.Update({reading.noisyPositionX});
        m_kalmanY.Update({reading.noisyPositionY});

        // Refined position estimate — used for replanning origin
        const Coordinates filteredPos(m_kalmanX.getState(),
                                      m_kalmanY.getState(),
                                      m_drone.position.altitude);

        // ── THINK ─────────────────────────────────────────────────────────────

        // Fail-safe: battery critical or GPS lost → abort to RETURN
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

        // Obstacle on outbound leg → EVADE (D* Lite replan)
        if (m_state == MissionState::OUTBOUND && reading.obstacleDetected) {
            HandleThreatDetection(reading.obstacleCell);
        }

        // Drain battery each active step (not while idle or landed)
        if (m_state == MissionState::OUTBOUND || m_state == MissionState::RETURN) {
            m_drone.batteryLevel -= BATTERY_DRAIN_PER_STEP;
        }

        // ── ACT ───────────────────────────────────────────────────────────────

        if (m_state == MissionState::OUTBOUND) {
            if (m_pathIndex < static_cast<int>(m_currentPath.size())) {
                m_drone.position = m_currentPath[m_pathIndex];
                ++m_pathIndex;

                // Predict post-move displacement so next KF Predict is accurate
                m_kalmanX.Predict({m_drone.position.x - filteredPos.x});
                m_kalmanY.Predict({m_drone.position.y - filteredPos.y});

                m_view.UpdateDronePosition(m_drone.position);
                m_view.displayGrid(m_grid, m_drone.position, m_targetPoint, m_currentPath);
                m_view.displayTelemetry(m_drone.batteryLevel, m_state, m_drone.position);

                if (coordsMatch(m_drone.position, m_targetPoint)) {
                    m_state = MissionState::RECON;
                    m_view.DisplayAlert("[FSM] OUTBOUND → RECON: Target reached.");
                }
            } else {
                // Path exhausted — treat as arrival (A* guarantees last node = target)
                m_state = MissionState::RECON;
                m_view.DisplayAlert("[FSM] OUTBOUND → RECON: Path complete.");
            }
        }

        if (m_state == MissionState::RECON) {
            // Capture IMINT, encode, store
            captureAndStoreImint(m_drone.position, reading.rawImagePayload);
            m_view.DisplayAlert("[RECON] IMINT captured at target.");

            // Plan return path with A* (grid may have new obstacles from EVADE)
            const std::vector<Coordinates> returnPath =
                m_astar.findPath(m_drone.position, m_startPoint, m_grid, m_costCalc);

            if (returnPath.empty()) {
                m_view.DisplayAlert("[RECON] No return path — emergency landing.");
                m_state = MissionState::LANDED;
            } else {
                setActivePath(returnPath);
                m_state = MissionState::RETURN;
                m_view.DisplayAlert("[FSM] RECON → RETURN: Heading home.");
            }
        }

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
                // Path exhausted — treat as arrival at start
                m_state          = MissionState::LANDED;
                m_missionSuccess = true;
                m_view.DisplayAlert("[FSM] LANDED: Return path complete.");
            }
        }
    }

    if (iteration >= MAX_LOOP_ITERATIONS) {
        m_view.DisplayAlert("[SAFETY] Max iterations reached — forcing LANDED.");
        m_state = MissionState::LANDED;
    }
}

// ── HandleThreatDetection ─────────────────────────────────────────────────────

void MissionController::HandleThreatDetection(const Coordinates& obstacleCell) {
    m_state = MissionState::EVADE;
    m_view.DisplayAlert("[FSM] OUTBOUND → EVADE: Obstacle at ("
        + std::to_string(static_cast<int>(obstacleCell.x)) + ", "
        + std::to_string(static_cast<int>(obstacleCell.y)) + ").");

    // D* Lite updatePath marks the obstacle in the grid internally (3 sensor
    // hits) and reprocesses only the affected vertices for efficiency.
    if (m_dstar) {
        const std::vector<Coordinates> replanned =
            m_dstar->updatePath(m_drone.position, obstacleCell,
                                m_grid, m_costCalc);

        if (replanned.empty()) {
            m_view.DisplayAlert("[EVADE] D* Lite: no path — aborting to RETURN.");
            const std::vector<Coordinates> retreat =
                m_astar.findPath(m_drone.position, m_startPoint,
                                 m_grid, m_costCalc);
            setActivePath(retreat);
            m_state = MissionState::RETURN;
        } else {
            setActivePath(replanned);
            m_state = MissionState::OUTBOUND;
            m_view.DisplayAlert("[FSM] EVADE → OUTBOUND: Replanned ("
                + std::to_string(m_currentPath.size()) + " waypoints).");
        }
    } else {
        m_view.DisplayAlert("[EVADE] D* Lite not initialised — returning to base.");
        m_state = MissionState::RETURN;
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

void MissionController::setActivePath(std::vector<Coordinates> newPath) {
    m_currentPath = std::move(newPath);
    // Skip waypoint[0]: it is always the current drone position (A* and
    // D* Lite both return paths inclusive of the origin cell).
    m_pathIndex = (static_cast<int>(m_currentPath.size()) > 1) ? 1 : 0;
}

bool MissionController::checkFailSafe() const {
    if (m_drone.batteryLevel < BATTERY_FAILSAFE) {
        m_view.DisplayAlert("[FAIL-SAFE] Battery at "
            + std::to_string(static_cast<int>(m_drone.batteryLevel)) + "%.");
        return true;
    }
    if (!m_drone.gpsAvailable) {
        m_view.DisplayAlert("[FAIL-SAFE] GPS signal lost.");
        return true;
    }
    return false;
}

bool MissionController::coordsMatch(const Coordinates& a, const Coordinates& b) {
    return static_cast<int>(a.x) == static_cast<int>(b.x) &&
           static_cast<int>(a.y) == static_cast<int>(b.y);
}

void MissionController::captureAndStoreImint(const Coordinates& location,
                                              const std::string& rawPayload)
{
    const long long ts      = static_cast<long long>(std::time(nullptr)) * 1000;
    const std::string encoded = Base64Codec::encode(rawPayload);
    const Point2D     loc2d(location.x, location.y);

    const IntelData intel(loc2d, ts, SensorType::IMINT, encoded);
    m_intelMgr.addIntel(intel);

    m_view.DisplayAlert("[IMINT] Stored @ ("
        + std::to_string(static_cast<int>(location.x)) + ", "
        + std::to_string(static_cast<int>(location.y)) + ") — "
        + std::to_string(encoded.size()) + " B encoded.");
}

std::string MissionController::stateToString(MissionState state) {
    if (state == MissionState::IDLE)     return "IDLE";
    if (state == MissionState::OUTBOUND) return "OUTBOUND";
    if (state == MissionState::RECON)    return "RECON";
    if (state == MissionState::EVADE)    return "EVADE";
    if (state == MissionState::RETURN)   return "RETURN";
    if (state == MissionState::LANDED)   return "LANDED";
    return "UNKNOWN";
}


} // namespace AIGD
