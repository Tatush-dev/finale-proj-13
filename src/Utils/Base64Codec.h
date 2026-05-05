#pragma once

#include <string>

namespace AIGD {

class Base64Codec {
public:
    static std::string encode(const std::string& input);
    static std::string decode(const std::string& input);
};

} // namespace AIGD
