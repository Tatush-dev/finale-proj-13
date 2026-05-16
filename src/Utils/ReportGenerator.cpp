#include "ReportGenerator.h"
#include "Base64Codec.h"

#include <cctype>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace AIGD {

// ── public ────────────────────────────────────────────────────────────────────

// Compiles all IntelData records into a structured JSON report and writes it
// to the given file path.
//
// Workflow:
//   1. Retrieve all intel records from the IntelManager.
//   2. Validate every Base64 payload in a single pass — collect per-record
//      results and an aggregate allValid flag.
//   3. Serialise metadata + intelligence array to prettified JSON.
//   4. Attempt to open the output file.  If it fails, log an error and
//      return false.  If it succeeds, write and close the file, log a
//      confirmation summary, and return true.
//
// Single-return form: success is initialised to false and set to true only
// after the file is written successfully (no early return on file-open failure).
bool ReportGenerator::generateReport(const std::string& filepath,
                                      const IntelManager& intelManager,
                                      bool missionSuccess)
{
    std::vector<IntelData> records = intelManager.getAllIntel();

    // Validate every Base64 payload; collect per-record results in one pass.
    std::vector<bool> payloadValid(records.size(), true);
    bool allValid = true;
    for (int i = 0; i < static_cast<int>(records.size()); ++i) {
        payloadValid[i] = validateBase64(records[i].getRawData());
        allValid = allValid && payloadValid[i];
    }

    // ── Build prettified JSON ─────────────────────────────────────────────────
    std::ostringstream json;
    json << std::fixed << std::setprecision(4);

    json << "{\n";
    json << "  \"mission_report\": {\n";
    json << "    \"generated_at\": \""
         << jsonEscape(utcTimestamp()) << "\",\n";
    json << "    \"total_records\": " << records.size() << ",\n";
    json << "    \"mission_outcome\": \""
         << (missionSuccess ? "SUCCESS" : "FAILURE") << "\",\n";
    json << "    \"integrity_validated\": "
         << (allValid ? "true" : "false") << ",\n";
    json << "    \"intelligence\": [\n";

    for (int i = 0; i < static_cast<int>(records.size()); ++i) {
        IntelData& rec    = records[i];
        bool       isLast = (i == static_cast<int>(records.size()) - 1);

        json << "      {\n";
        json << "        \"index\": " << i << ",\n";
        json << "        \"location\": {\n";
        json << "          \"x\": " << rec.getLocationX() << ",\n";
        json << "          \"y\": " << rec.getLocationY() << "\n";
        json << "        },\n";
        json << "        \"timestamp_ms\": " << rec.getTimestamp() << ",\n";
        json << "        \"sensor_type\": \""
             << jsonEscape(rec.getSensorTypeString()) << "\",\n";
        json << "        \"base64_payload\": \""
             << jsonEscape(rec.getRawData()) << "\",\n";
        json << "        \"payload_size_bytes\": " << rec.getDataSize() << ",\n";
        json << "        \"payload_valid\": "
             << (payloadValid[i] ? "true" : "false") << "\n";
        json << "      }" << (isLast ? "" : ",") << "\n";
    }

    json << "    ]\n";
    json << "  }\n";
    json << "}\n";

    // ── Write to file ─────────────────────────────────────────────────────────
    // Single-return form: success flag updated inside the else branch.
    std::ofstream outFile(filepath);
    bool success = false;

    if (!outFile.is_open()) {
        std::cerr << "[ReportGenerator] ERROR: cannot open '"
                  << filepath << "' for writing.\n";
    } else {
        outFile << json.str();
        outFile.close();

        // ── Console confirmation ──────────────────────────────────────────────
        std::cout << "[ReportGenerator] Report exported  : " << filepath << "\n";
        std::cout << "[ReportGenerator] Records written  : " << records.size() << "\n";
        std::cout << "[ReportGenerator] Integrity check  : "
                  << (allValid ? "PASS" : "FAIL (see payload_valid per record)") << "\n";

        success = true;
    }

    return success;
}

// ── private helpers ───────────────────────────────────────────────────────────

// Validates a Base64-encoded string against four rules:
//   Rule 1: non-empty and length is a multiple of 4.
//   Rule 2: at most two '=' padding characters permitted.
//   Rule 3: '=' padding may only appear in the final two positions.
//   Rule 4: dry-run decode via Base64Codec succeeds without throwing.
//
// Single-return form: result defaults to false.  The entire validation logic
// is wrapped in an outer if-guard (rule 1); result is set to the computed
// valid flag at the end of that block.
bool ReportGenerator::validateBase64(const std::string& payload)
{
    bool result = false;

    // Rule 1 gate: reject empty strings and non-multiples-of-4 immediately.
    if (!payload.empty() && payload.size() % 4 == 0) {
        bool valid    = true;
        int  padsSeen = 0;

        for (int i = 0; i < static_cast<int>(payload.size()); ++i) {
            unsigned char c         = static_cast<unsigned char>(payload[i]);
            bool          isPad     = (c == '=');
            bool          isAlpha   = std::isalpha(c) != 0;
            bool          isDigit   = std::isdigit(c) != 0;
            bool          isSpecial = (c == '+' || c == '/');
            bool          isValidB64 = isAlpha || isDigit || isSpecial;

            // Rule 3: '=' padding may only appear in the final two positions.
            bool isInLastTwo = (i >= static_cast<int>(payload.size()) - 2);

            // Accumulate validity — once false it stays false.
            valid = valid && (isPad ? isInLastTwo : (isValidB64 && padsSeen == 0));

            // Track padding count after the check so padsSeen reflects chars before i.
            padsSeen += isPad ? 1 : 0;
        }

        // Rule 2 (aggregate): at most two padding characters permitted.
        valid = valid && (padsSeen <= 2);

        // Rule 4: dry-run decode to confirm the string decodes without error.
        if (valid) {
            try {
                Base64Codec::decode(payload);
            } catch (...) {
                valid = false;
            }
        }

        result = valid;
    }

    return result;
}

std::string ReportGenerator::jsonEscape(const std::string& s)
{
    std::ostringstream oss;
    for (int i = 0; i < static_cast<int>(s.size()); ++i) {
        char c = s[i];
        if      (c == '"')  oss << "\\\"";
        else if (c == '\\') oss << "\\\\";
        else if (c == '\n') oss << "\\n";
        else if (c == '\r') oss << "\\r";
        else if (c == '\t') oss << "\\t";
        else                oss << c;
    }
    return oss.str();
}

std::string ReportGenerator::utcTimestamp()
{
    std::time_t now = std::time(nullptr);
    char buf[32] = {};
    const struct tm* gmt = std::gmtime(&now);
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", gmt);
    return std::string(buf);
}

} // namespace AIGD
