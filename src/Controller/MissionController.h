#pragma once
#include "IMissionController.h"
#include "../Model/INavigationModel.h"
#include "../Model/IIntelligenceManager.h"
#include "../View/IMissionView.h"
#include "../Common/Types.h"
#include <memory>

// Concrete FSM controller — wires together Model and View via interfaces.
class MissionController : public IMissionController {
public:
    MissionController(std::shared_ptr<INavigationModel>    navModel,
                      std::shared_ptr<AIGD::IIntelligenceManager> intelMgr,
                      std::shared_ptr<IMissionView>         view);

    void StartMission() override;
    void ExecuteSenseThinkActLoop() override;
    void HandleThreatDetection() override;
        
private:
    std::shared_ptr<INavigationModel>    m_navModel;
    std::shared_ptr<AIGD::IIntelligenceManager> m_intelMgr;
    std::shared_ptr<IMissionView>        m_view;
    MissionState                         m_state;
    Coordinates                          m_currentPosition;
};
