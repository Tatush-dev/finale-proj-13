#include "IntelligenceManager.h"
#include <iostream>

void IntelligenceManager::StoreIntel(const IntelData& data) {
    m_store[data.id] = data;
    std::cout << "[IntelligenceManager] Stored intel id=" << data.id << "\n";
}

IntelData IntelligenceManager::RetrieveIntel(const Coordinates& location) {
    // Stub: return first entry that matches location (nearest-match in Task 1.3)
    for (const auto& [id, intel] : m_store) {
        if (intel.location.x == location.x && intel.location.y == location.y) {
            return intel;
        }
    }
    return IntelData{};
}

std::list<IntelData> IntelligenceManager::GetAllIntel() const {
    std::list<IntelData> result;
    for (const auto& [id, intel] : m_store) {
        result.push_back(intel);
    }
    return result;
}
