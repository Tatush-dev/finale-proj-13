#include "ConfigParser.h"
#include "Base64Codec.h"

namespace AIGD {

// ── File I/O ──────────────────────────────────────────────────────────────────

std::string ConfigParser::readFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("ConfigParser: cannot open '" + filename + "'");
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

// ── Section Extraction ────────────────────────────────────────────────────────
// Locates a named JSON object (e.g. "kalman_filter") within the raw JSON string
// and returns the content between its outermost braces.
//
// Algorithm:
//   1. Find the key string ("section_name") in the JSON.
//   2. Locate the opening brace '{' that follows the key.
//   3. Walk forward using a depth counter to find the matching closing brace
//      (handles nested objects).  
//   4. Return the content between the braces, or "" if any step fails.
//
// Single-return form: result defaults to "" and is only set when all
// three location steps succeed.
std::string ConfigParser::extractSection(const std::string& json,
                                         const std::string& key) {
    std::string result   = "";
    std::string searchKey = "\"" + key + "\"";
    size_t keyPos         = json.find(searchKey);

    if (keyPos != std::string::npos) {
        size_t braceOpen = json.find('{', keyPos + searchKey.size());

        if (braceOpen != std::string::npos) {
            // Walk forward tracking brace depth to find the matching '}'.
            int    depth    = 1;
            size_t i        = braceOpen + 1;
            bool   finished = false;

            while (i < json.size() && !finished) {
                if (json[i] == '{') {
                    ++depth;
                    ++i;
                } else if (json[i] == '}') {
                    --depth;
                    if (depth == 0) {
                        finished = true;   // i now points to the closing brace
                    } else {
                        ++i;
                    }
                } else {
                    ++i;
                }
            }

            if (finished) {
                result = json.substr(braceOpen + 1, i - braceOpen - 1);
            }
        }
    }

    return result;
}

// ── Value Extraction Helpers ──────────────────────────────────────────────────

// Extracts a numeric value (integer or float) for the given JSON key.
//
// Algorithm:
//   1. Find the key string in the JSON.
//   2. Locate the ':' separator that follows.
//   3. Skip leading whitespace after the colon.
//   4. Consume numeric characters (digits, '.', '-', '+', 'e', 'E') into a
//      substring.  A boolean flag drives loop exit 
//   5. Convert the substring to double via std::stod.
//
// Single-return form: result defaults to 0.0.
double ConfigParser::extractFloat(const std::string& json,
                                   const std::string& key) {
    double result  = 0.0;
    std::string searchKey = "\"" + key + "\"";
    size_t keyPos  = json.find(searchKey);

    if (keyPos != std::string::npos) {
        size_t colonPos = json.find(':', keyPos + searchKey.size());

        if (colonPos != std::string::npos) {
            // Skip leading whitespace after the colon.
            size_t valueStart = colonPos + 1;
            while (valueStart < json.size() &&
                   (json[valueStart] == ' '  || json[valueStart] == '\t' ||
                    json[valueStart] == '\n' || json[valueStart] == '\r')) {
                ++valueStart;
            }

            // Consume numeric characters. 
            size_t valueEnd = valueStart;
            bool   reading  = true;
            while (valueEnd < json.size() && reading) {
                char c = json[valueEnd];
                if (std::isdigit(static_cast<unsigned char>(c)) ||
                    c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E') {
                    ++valueEnd;
                } else {
                    reading = false;
                }
            }

            if (valueEnd != valueStart) {
                try {
                    result = std::stod(json.substr(valueStart, valueEnd - valueStart));
                } catch (...) {
                    result = 0.0;
                }
            }
        }
    }

    return result;
}

// Extracts a quoted string value for the given JSON key.
//
// Algorithm:
//   1. Find the key string.
//   2. Locate the ':' separator.
//   3. Locate the opening '"' of the value.
//   4. Scan forward for the unescaped closing '"' using an escape-state flag.
//   5. Return the substring between the quotes, or "" if any step fails.
//
// Single-return form: result defaults to "".
std::string ConfigParser::extractString(const std::string& json,
                                        const std::string& key) {
    std::string result  = "";
    std::string searchKey = "\"" + key + "\"";
    size_t keyPos  = json.find(searchKey);

    if (keyPos != std::string::npos) {
        size_t colonPos = json.find(':', keyPos + searchKey.size());

        if (colonPos != std::string::npos) {
            size_t openQuote = json.find('"', colonPos + 1);

            if (openQuote != std::string::npos) {
                // Advance past the opening quote and scan for the unescaped closing quote.
                size_t closeQuote = openQuote + 1;
                bool   escaped    = false;
                bool   foundClose = false;

                while (closeQuote < json.size() && !foundClose) {
                    if (escaped) {
                        escaped = false;
                        ++closeQuote;
                    } else if (json[closeQuote] == '\\') {
                        escaped = true;
                        ++closeQuote;
                    } else if (json[closeQuote] == '"') {
                        foundClose = true;   // closeQuote points to closing '"'
                    } else {
                        ++closeQuote;
                    }
                }

                if (foundClose) {
                    result = json.substr(openQuote + 1, closeQuote - openQuote - 1);
                }
            }
        }
    }

    return result;
}

// ── Base64 Decoding ───────────────────────────────────────────────────────────

std::string ConfigParser::decodeBase64(const std::string& encoded) {
    return Base64Codec::decode(encoded);
}

// ── Main Parse ────────────────────────────────────────────────────────────────
// Reads the JSON config file and populates an AppConfig struct.
// Each logical section is extracted first, then individual fields are pulled
// from the section strings using the helpers above.
AppConfig ConfigParser::parse(const std::string& filename) const {
    std::string json = readFile(filename);
    AppConfig config;

    // Kalman filter parameters
    std::string kalmanSection = extractSection(json, "kalman_filter");
    config.kalmanFilter.Q = extractFloat(kalmanSection, "Q");
    config.kalmanFilter.R = extractFloat(kalmanSection, "R");

    // Mission coordinates
    std::string missionSection = extractSection(json, "mission_points");
    std::string startSection   = extractSection(missionSection, "start");
    std::string targetSection  = extractSection(missionSection, "target");

    config.missionPoints.start.x        = extractFloat(startSection,  "x");
    config.missionPoints.start.y        = extractFloat(startSection,  "y");
    config.missionPoints.start.altitude = extractFloat(startSection,  "altitude");

    config.missionPoints.target.x        = extractFloat(targetSection, "x");
    config.missionPoints.target.y        = extractFloat(targetSection, "y");
    config.missionPoints.target.altitude = extractFloat(targetSection, "altitude");

    // Map layout
    std::string mapSection   = extractSection(json, "map_data");
    config.mapData.width      = static_cast<int>(extractFloat(mapSection, "width"));
    config.mapData.height     = static_cast<int>(extractFloat(mapSection, "height"));
    config.mapData.gridBase64 = extractString(mapSection, "grid_base64");

    // Mission settings
    std::string settingsSection = extractSection(json, "mission_settings");
    std::string parsedPriority  = extractString(settingsSection, "navigation_priority");
    if (!parsedPriority.empty()) {
        config.navigationPriority = parsedPriority;
    }

    return config;
}

} // namespace AIGD
