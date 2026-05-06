#include "IntelligenceManager.h"
#include <iostream>

namespace AIGD {

void IntelligenceManager::addIntel(const IntelData& data) {
    m_store[data.getTimestamp()] = data;
    std::cout << "[IntelligenceManager] Stored intel timestamp=" << data.getTimestamp() << "\n";
}

std::optional<IntelData> IntelligenceManager::getIntel(long long timestamp) const {
    auto it = m_store.find(timestamp);
    if (it != m_store.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<IntelData> IntelligenceManager::getAllIntel() const {
    std::vector<IntelData> result;
    result.reserve(m_store.size());
    for (const auto& [timestamp, intel] : m_store) {
        result.push_back(intel);
    }
    return result;
}

} // namespace AIGD

