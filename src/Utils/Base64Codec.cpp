#include "Base64Codec.h"

#include <array>
#include <cctype>
#include <stdexcept>
#include <vector>

namespace AIGD {

namespace {
    const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    inline bool isBase64(unsigned char c) {
        return std::isalnum(c) || (c == '+') || (c == '/');
    }
}

std::string Base64Codec::encode(const std::string& input) {
    std::string output;
    output.reserve(((input.size() + 2) / 3) * 4);

    int val = 0;
    int valb = -6;
    for (unsigned char c : input) {
        val = (val << 8) + c;
        valb += 8;
        while (valb >= 0) {
            output.push_back(base64_chars[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }
    if (valb > -6) {
        output.push_back(base64_chars[((val << 8) >> (valb + 8)) & 0x3F]);
    }
    while (output.size() % 4) {
        output.push_back('=');
    }
    return output;
}

std::string Base64Codec::decode(const std::string& input) {
    std::vector<int> T(256, -1);
    for (int i = 0; i < 64; i++) {
        T[static_cast<unsigned char>(base64_chars[i])] = i;
    }

    std::string output;
    output.reserve((input.size() / 4) * 3);

    int val = 0;
    int valb = -8;
    for (unsigned char c : input) {
        if (std::isspace(c) || c == '=') {
            if (c == '=') {
                break;
            }
            continue;
        }
        if (T[c] == -1) {
            throw std::invalid_argument("Invalid Base64 input character");
        }
        val = (val << 6) + T[c];
        valb += 6;
        if (valb >= 0) {
            output.push_back(static_cast<char>((val >> valb) & 0xFF));
            valb -= 8;
        }
    }

    return output;
}

} // namespace AIGD
