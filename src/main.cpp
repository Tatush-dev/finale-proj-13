#include "Controller/MissionController.h"

int main() {
    return AIGD::MissionController::runMission("config.json") ? 0 : 1;
}
