#pragma once
#include <string>
#include "../Model/IIntelligenceManager.h"

namespace AIGD {

/**
 * Compiles all gathered IntelData from an IIntelligenceManager into a
 * structured, prettified JSON report file with per-record Base64 integrity
 * validation.
 *
 * Accepts the abstract IIntelligenceManager interface so this utility is
 * decoupled from any concrete storage implementation.
 */
class ReportGenerator {
public:
    /**
     * Exports all intelligence records to a JSON file.
     *
     * Workflow:
     *   1. Retrieve all IntelData via intelManager.getAllIntel().
     *   2. Validate each Base64 payload (length, charset, padding, dry-run decode).
     *   3. Serialize metadata + intelligence array to prettified JSON.
     *   4. Write to filepath and log the result to stdout.
     *
     * @param filepath        Destination file path (e.g. "mission_report.json").
     * @param intelManager    Abstract interface to the collected intelligence store.
     * @param missionSuccess  Overall mission outcome for the report metadata.
     * @return true if the file was written successfully, false otherwise.
     */
    static bool generateReport(const std::string& filepath,
                                const IIntelligenceManager& intelManager,
                                bool missionSuccess = true);

private:
    /**
     * Validates a Base64 string:
     *   - Length is a multiple of 4.
     *   - Characters are in [A-Za-z0-9+/=].
     *   - '=' padding only appears in the last two positions.
     *   - Dry-run decode via Base64Codec succeeds without exception.
     */
    static bool validateBase64(const std::string& payload);

    // Escapes special characters so the string is safe inside a JSON value.
    static std::string jsonEscape(const std::string& s);

    // Returns current UTC time formatted as ISO 8601 (e.g. "2026-05-11T14:30:00Z").
    static std::string utcTimestamp();
};

} // namespace AIGD
