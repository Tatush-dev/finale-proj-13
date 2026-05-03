#pragma once
#include <list>
#include "../Common/Types.h"

/**
 * Abstract interface for the intelligence store.
 * Backed by an unordered_map for O(1) insertion and lookup.
 */
class IIntelligenceManager {
public:
    virtual ~IIntelligenceManager() = default;

    // O(1) insertion via HashTable key on IntelData::id.
    virtual void StoreIntel(const IntelData& data) = 0;

    // Returns intelligence collected at a specific location (nearest match).
    virtual IntelData RetrieveIntel(const Coordinates& location) = 0;

    // Returns the full accumulated intel list for report generation.
    virtual std::list<IntelData> GetAllIntel() const = 0;
};
