#ifndef APP_ENUMS_H
#define APP_ENUMS_H

/**
 * @file AppEnums.h
 * @brief Core enumeration types for the Autonomous Intelligence Gathering Drone system.
 * 
 * Defines mission states and sensor types used throughout the MVC architecture.
 */

namespace AIGD {

/**
 * @enum MissionState
 * @brief Represents the different states in the mission Finite State Machine (FSM).
 */
enum class MissionState {
    IDLE,       ///< System is initialized and awaiting mission start
    OUTBOUND,   ///< Drone is traveling to the reconnaissance zone
    RECON,      ///< Drone is actively collecting intelligence
    EVADE,      ///< Drone detected threat and is evasive maneuvering
    RETURN,     ///< Drone is returning to the base location
    LANDED      ///< Drone has completed mission and landed
};

/**
 * @enum SensorType
 * @brief Identifies the type of sensor data being collected.
 */
enum class SensorType {
    IMINT,      ///< Imagery Intelligence - visual/optical data
    SIGNAL,     ///< Signal Intelligence - RF/electromagnetic data (renamed from SIGINT to avoid Windows macro conflict)
    THERMAL     ///< Thermal imaging data
};

} // namespace AIGD

#endif // APP_ENUMS_H
