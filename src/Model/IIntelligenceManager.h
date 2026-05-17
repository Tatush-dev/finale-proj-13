#pragma once
#include <vector>
#include <optional>
#include "IntelData.h"

namespace AIGD {

/**
 * Abstract interface for the intelligence store.
 * Backed by an unordered_map for O(1) insertion and lookup.
 */
class IIntelligenceManager {
public:
    virtual ~IIntelligenceManager() = default;

    // O(1) insertion via HashTable key on IntelData::timestamp.
    virtual void addIntel(const IntelData& data) = 0;

    // Returns intelligence collected at a specific timestamp.
    virtual std::optional<IntelData> getIntel(long long timestamp) const = 0;

    // Returns the full accumulated intel list for report generation.
    virtual std::vector<IntelData> getAllIntel() const = 0;
};

} // namespace AIGD

