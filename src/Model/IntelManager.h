#pragma once

#include "IntelData.h"
#include <optional>
#include <unordered_map>
#include <vector>

namespace AIGD {

class IntelManager {
public:
    void addIntel(const IntelData& intel);
    std::optional<IntelData> getIntel(long long timestamp) const;
    std::vector<IntelData> getAllIntel() const;
    bool deleteIntel(long long timestamp);

private:
    std::unordered_map<long long, IntelData> intelStore_;
};

} // namespace AIGD
