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
    // Reserves enough memory in output to avoid reallocations.
    // Base64 expands every 3 bytes into 4 characters, so this computes the expected length.
    output.reserve(((input.size() + 2) / 3) * 4);
    //val is the bit buffer where bytes are assembled.
    int val = 0;
    // valb tracks how many bits are currently available in val 
    // - initialized to -6 so the first 8-bit byte will bring it to 2, allowing immediate output of a Base64 character.
    int valb = -6;
    for (unsigned char c : input) {
        // Shifts the buffer left by 8 bits and appends the new byte
        val = (val << 8) + c;
        valb += 8;
        // While there are at least 6 bits available in the buffer
        while (valb >= 0) {
            // Extracts the top 6 bits from val, aligns the next 6 bits to the low end and masks to exactly 6 bits.
            output.push_back(base64_chars[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }
    // if there are leftover bits that were not emitted
    if (valb > -6) {
        // Shifts the remaining bits to the top of a 6-bit group and emits one final Base64 character
        output.push_back(base64_chars[((val << 8) >> (valb + 8)) & 0x3F]);
    }
    // Appends padding characters until the output length is divisible by 4
    while (output.size() % 4) {
        output.push_back('=');
    }
    return output;
}

std::string Base64Codec::decode(const std::string& input) {
    std::vector<int> T(256, -1); // Initializes all entries to -1 meaning “invalid Base64 character.”
    for (int i = 0; i < 64; i++) {
        T[static_cast<unsigned char>(base64_chars[i])] = i;
    }

    std::string output;
    output.reserve((input.size() / 4) * 3);

    int val = 0;
    // Starting at -8 means the first 6-bit group will not yet produce a byte until enough bits accumulate
    int valb = -8;
    bool shouldContinueDecoding = true;
    for (unsigned char c : input) {
        if (shouldContinueDecoding) {
            if (std::isspace(c)) {
                // Skip whitespace, continue to next character
            } else if (c == '=') {
                shouldContinueDecoding = false;
            } else {
                if (T[c] == -1) {
                    throw std::invalid_argument("Invalid Base64 input character");
                }
                // Shifts the buffer by 6 bits and appends the numeric value for the current Base64 character
                val = (val << 6) + T[c];
                // Increases the bit count by 6 because a full Base64 symbol was added
                valb += 6;
                // If there are at least 8 bits available in the buffer, extract the top 8 bits and append to output
                if (valb >= 0) {
                    output.push_back(static_cast<char>((val >> valb) & 0xFF));
                    valb -= 8;
                }
            }
        }
    }

    return output;
}

} // namespace AIGD
