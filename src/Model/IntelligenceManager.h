#pragma once
#include "IIntelligenceManager.h"
#include <unordered_map>

// Backed by unordered_map for O(1) store/retrieve on IntelData::id.
class IntelligenceManager : public IIntelligenceManager {
public:
    void StoreIntel(const IntelData& data) override;
    IntelData RetrieveIntel(const Coordinates& location) override;
    std::list<IntelData> GetAllIntel() const override;

private:
    std::unordered_map<int, IntelData> m_store;
};
