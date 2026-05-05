#include "IntelManager.h"

namespace AIGD {

void IntelManager::addIntel(const IntelData& intel) {
    intelStore_[intel.getTimestamp()] = intel;
}

std::optional<IntelData> IntelManager::getIntel(long long timestamp) const {
    auto it = intelStore_.find(timestamp);
    if (it != intelStore_.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<IntelData> IntelManager::getAllIntel() const {
    std::vector<IntelData> all;
    all.reserve(intelStore_.size());
    for (const auto& pair : intelStore_) {
        all.push_back(pair.second);
    }
    return all;
}

bool IntelManager::deleteIntel(long long timestamp) {
    return intelStore_.erase(timestamp) > 0;
}

} // namespace AIGD
