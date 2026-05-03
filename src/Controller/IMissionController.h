#pragma once

/**
 * Abstract interface for the FSM-based mission controller.
 * Drives the Sense-Think-Act loop and manages state transitions.
 */
class IMissionController {
public:
    virtual ~IMissionController() = default;

    // Transitions FSM from IDLE → OUTBOUND and begins navigation.
    virtual void StartMission() = 0;

    // Main loop: Sense (sensors) → Think (Kalman + path) → Act (move/store).
    // Runs continuously through OUTBOUND and RECON states.
    virtual void ExecuteSenseThinkActLoop() = 0;

    // Called on threat detection; transitions FSM to EVADE and triggers D* Lite.
    virtual void HandleThreatDetection() = 0;
};
