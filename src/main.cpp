#include <iostream>
#include <memory>

#include "Model/NavigationModel.h"
#include "Model/IntelligenceManager.h"
#include "View/MissionView.h"
#include "Controller/MissionController.h"

int main() {
    std::cout << "--- SYSTEM START ---" << std::endl;
    std::cout << "=== Autonomous IMINT Drone — MVC Skeleton Validation ===\n\n";

    // Wire up MVC via interfaces (dependency injection through shared_ptr).
    auto navModel  = std::make_shared<NavigationModel>();
    auto intelMgr  = std::make_shared<IntelligenceManager>();
    auto view      = std::make_shared<MissionView>();
    auto controller = std::make_shared<MissionController>(navModel, intelMgr, view);

    // ── Validate NavigationModel ─────────────────────────────────────────────
    Coordinates base(0.0, 0.0, 100.0);
    Coordinates target(5.0, 5.0, 100.0);
    Path path = navModel->CalculateInitialPath(base, target);
    std::cout << "Path waypoints: " << path.size() << "\n\n";

    // ── Validate IntelligenceManager ─────────────────────────────────────────
    IntelData intel{ 1, target, "Suspected activity", "2026-05-02T08:00Z", "" };
    intelMgr->StoreIntel(intel);
    auto allIntel = intelMgr->GetAllIntel();
    std::cout << "Intel entries stored: " << allIntel.size() << "\n\n";

    // ── Validate MissionController FSM ───────────────────────────────────────
    controller->StartMission();
    controller->ExecuteSenseThinkActLoop();
    controller->HandleThreatDetection();

    // ── Validate MissionView report ──────────────────────────────────────────
    std::cout << "\n";
    view->GenerateFinalReport(allIntel);

    std::cout << "\n=== Skeleton validation complete ===\n";
    return 0;
}
