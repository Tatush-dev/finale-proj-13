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

std::string ConfigParser::extractSection(const std::string& json,
                                         const std::string& key) {
    const std::string searchKey = "\"" + key + "\"";
    const size_t      keyPos    = json.find(searchKey);
    if (keyPos == std::string::npos) return "";

    const size_t braceOpen = json.find('{', keyPos + searchKey.size());
    if (braceOpen == std::string::npos) return "";

    // Scan forward to find the matching closing brace using a depth counter.
    // No break/continue: a boolean flag controls loop exit.
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

    if (!finished) return "";
    return json.substr(braceOpen + 1, i - braceOpen - 1);
}

// ── Value Extraction Helpers ──────────────────────────────────────────────────

double ConfigParser::extractFloat(const std::string& json,
                                   const std::string& key) {
    const std::string searchKey = "\"" + key + "\"";
    const size_t      keyPos    = json.find(searchKey);
    if (keyPos == std::string::npos) return 0.0;

    const size_t colonPos = json.find(':', keyPos + searchKey.size());
    if (colonPos == std::string::npos) return 0.0;

    // Skip leading whitespace after the colon
    size_t valueStart = colonPos + 1;
    while (valueStart < json.size() &&
           (json[valueStart] == ' ' || json[valueStart] == '\t' ||
            json[valueStart] == '\n' || json[valueStart] == '\r')) {
        ++valueStart;
    }

    // Consume numeric characters. No break/continue: boolean flag ends the loop.
    size_t valueEnd = valueStart;
    bool   reading  = true;
    while (valueEnd < json.size() && reading) {
        const char c = json[valueEnd];
        if (std::isdigit(static_cast<unsigned char>(c)) ||
            c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E') {
            ++valueEnd;
        } else {
            reading = false;
        }
    }

    if (valueEnd == valueStart) return 0.0;
    try {
        return std::stod(json.substr(valueStart, valueEnd - valueStart));
    } catch (...) {
        return 0.0;
    }
}


std::string ConfigParser::extractString(const std::string& json,
                                        const std::string& key) {
    const std::string searchKey = "\"" + key + "\"";
    const size_t      keyPos    = json.find(searchKey);
    if (keyPos == std::string::npos) return "";

    const size_t colonPos = json.find(':', keyPos + searchKey.size());
    if (colonPos == std::string::npos) return "";

    const size_t openQuote = json.find('"', colonPos + 1);
    if (openQuote == std::string::npos) return "";

    // Advance past the opening quote and scan for the unescaped closing quote.
    // No break/continue: boolean flag ends the loop.
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

    if (!foundClose) return "";
    return json.substr(openQuote + 1, closeQuote - openQuote - 1);
}

// ── Base64 Decoding ───────────────────────────────────────────────────────────

std::string ConfigParser::decodeBase64(const std::string& encoded) {
    return Base64Codec::decode(encoded);
}

// ── Main Parse ────────────────────────────────────────────────────────────────

AppConfig ConfigParser::parse(const std::string& filename) const {
    const std::string json = readFile(filename);
    AppConfig config;

    // Kalman filter parameters
    const std::string kalmanSection = extractSection(json, "kalman_filter");
    config.kalmanFilter.Q = extractFloat(kalmanSection, "Q");
    config.kalmanFilter.R = extractFloat(kalmanSection, "R");

    // Mission coordinates
    const std::string missionSection = extractSection(json, "mission_points");
    const std::string startSection   = extractSection(missionSection, "start");
    const std::string targetSection  = extractSection(missionSection, "target");

    config.missionPoints.start.x        = extractFloat(startSection,  "x");
    config.missionPoints.start.y        = extractFloat(startSection,  "y");
    config.missionPoints.start.altitude = extractFloat(startSection,  "altitude");

    config.missionPoints.target.x        = extractFloat(targetSection, "x");
    config.missionPoints.target.y        = extractFloat(targetSection, "y");
    config.missionPoints.target.altitude = extractFloat(targetSection, "altitude");

    // Map layout
    const std::string mapSection   = extractSection(json, "map_data");
    config.mapData.width      = static_cast<int>(extractFloat(mapSection, "width"));
    config.mapData.height     = static_cast<int>(extractFloat(mapSection, "height"));
    config.mapData.gridBase64 = extractString(mapSection, "grid_base64");

    // Mission settings
    const std::string settingsSection = extractSection(json, "mission_settings");
    const std::string parsedPriority  = extractString(settingsSection, "navigation_priority");
    if (!parsedPriority.empty()) {
        config.navigationPriority = parsedPriority;
    }

    return config;
}

} // namespace AIGD
