#pragma once

#include "../Common/Types.h"

#include <string>
#include <fstream>
#include <sstream>
#include <cctype>
#include <stdexcept>

namespace AIGD {

// ── Config Data Structures ────────────────────────────────────────────────────

struct KalmanConfig {
    double Q = 0.01;
    double R = 0.09;
};

struct MissionPointsConfig {
    Coordinates start;
    Coordinates target;
};

struct MapDataConfig {
    int         width      = 10;
    int         height     = 10;
    std::string gridBase64 = "";
};

struct AppConfig {
    KalmanConfig        kalmanFilter;
    MissionPointsConfig missionPoints;
    MapDataConfig       mapData;
    std::string         navigationPriority = "BALANCED";
};

// ── ConfigParser ──────────────────────────────────────────────────────────────

/**
 * Custom JSON parser using only std::ifstream and standard string manipulation.
 */
class ConfigParser {
public:
    /**
     * Reads and parses the given JSON config file.
     * @throws std::runtime_error if the file cannot be opened.
     */
    AppConfig parse(const std::string& filename) const;

    /**
     * Decodes a Base64-encoded string back to its original bytes.
     * Ignores non-Base64 characters (e.g. whitespace, '=').
     */
    static std::string decodeBase64(const std::string& encoded);

private:
    static std::string readFile(const std::string& filename);

    /** Returns the content between the outermost braces of the named JSON object. */
    static std::string extractSection(const std::string& json,
                                      const std::string& key);

    /** Extracts any numeric value for the given JSON key. Works for both floats and integers. */
    static double extractFloat(const std::string& json,
                               const std::string& key);

    /** Extracts the quoted string value for the given JSON key. */
    static std::string extractString(const std::string& json,
                                     const std::string& key);
};

} // namespace AIGD
