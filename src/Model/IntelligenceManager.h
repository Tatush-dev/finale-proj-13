#pragma once
#include "IIntelligenceManager.h"
#include <unordered_map>

namespace AIGD {

// Backed by unordered_map for O(1) store/retrieve on IntelData::timestamp.
class IntelligenceManager : public IIntelligenceManager {
public:
    void addIntel(const IntelData& data) override;
    std::optional<IntelData> getIntel(long long timestamp) const override;
    std::vector<IntelData> getAllIntel() const override;

private:
    std::unordered_map<long long, IntelData> m_store;
};

} // namespace AIGD

