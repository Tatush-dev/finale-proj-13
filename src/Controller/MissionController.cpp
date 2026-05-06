#include "MissionController.h"
#include <iostream>

MissionController::MissionController(
    std::shared_ptr<INavigationModel>     navModel,
    std::shared_ptr<AIGD::IIntelligenceManager> intelMgr,
    std::shared_ptr<IMissionView>         view)
    : m_navModel(navModel)
    , m_intelMgr(intelMgr)
    , m_view(view)
    , m_state(MissionState::IDLE)
    , m_currentPosition(0.0, 0.0, 0.0)
{}

void MissionController::StartMission() {
    m_state = MissionState::OUTBOUND;
    std::cout << "[MissionController] Mission started → OUTBOUND\n";
    m_view->DisplayAlert("Mission started. Proceeding to target area.");
}

void MissionController::ExecuteSenseThinkActLoop() {
    std::cout << "[MissionController] Sense-Think-Act loop running (stub)\n";
    // Sense  : read EO/Thermal sensor data
    // Think  : run Kalman filter, update occupancy grid, check for threats
    // Act    : advance along waypoint queue, store any intel collected
}

void MissionController::HandleThreatDetection() {
    m_state = MissionState::EVADE;
    std::cout << "[MissionController] Threat detected → EVADE\n";
    m_view->DisplayAlert("Threat detected! Initiating evasive re-routing.");
    // D* Lite re-route triggered here via m_navModel->UpdateDynamicPath(...)
}
