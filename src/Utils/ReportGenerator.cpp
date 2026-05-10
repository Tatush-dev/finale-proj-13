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

bool ReportGenerator::generateReport(const std::string& filepath,
                                      const IntelManager& intelManager,
                                      bool missionSuccess)
{
    const std::vector<IntelData> records = intelManager.getAllIntel();

    // Validate every Base64 payload; collect per-record results in one pass
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
        const IntelData& rec    = records[i];
        const bool       isLast = (i == static_cast<int>(records.size()) - 1);

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
    std::ofstream outFile(filepath);
    if (!outFile.is_open()) {
        std::cerr << "[ReportGenerator] ERROR: cannot open '"
                  << filepath << "' for writing.\n";
        return false;
    }

    outFile << json.str();
    outFile.close();

    // ── Console confirmation ──────────────────────────────────────────────────
    std::cout << "[ReportGenerator] Report exported  : " << filepath << "\n";
    std::cout << "[ReportGenerator] Records written  : " << records.size() << "\n";
    std::cout << "[ReportGenerator] Integrity check  : "
              << (allValid ? "PASS" : "FAIL (see payload_valid per record)") << "\n";

    return true;
}

// ── private helpers ───────────────────────────────────────────────────────────

bool ReportGenerator::validateBase64(const std::string& payload)
{
    // Rule 1: non-empty and length must be a multiple of 4
    if (payload.empty() || payload.size() % 4 != 0) {
        return false;
    }

    bool valid    = true;
    int  padsSeen = 0;

    for (int i = 0; i < static_cast<int>(payload.size()); ++i) {
        const unsigned char c          = static_cast<unsigned char>(payload[i]);
        const bool          isPad      = (c == '=');
        const bool          isAlpha    = std::isalpha(c) != 0;
        const bool          isDigit    = std::isdigit(c) != 0;
        const bool          isSpecial  = (c == '+' || c == '/');
        const bool          isValidB64 = isAlpha || isDigit || isSpecial;

        // Rule 3: '=' padding may only appear in the final two positions
        const bool isInLastTwo = (i >= static_cast<int>(payload.size()) - 2);

        // Check validity before updating state
        valid = valid && (isPad ? isInLastTwo : (isValidB64 && padsSeen == 0));

        // Update padding counter after check so padsSeen reflects chars before index i
        padsSeen += isPad ? 1 : 0;
    }

    // Rule 2 (aggregate): at most two padding characters permitted
    valid = valid && (padsSeen <= 2);

    // Rule 4: dry-run decode — confirm the string decodes without error
    if (valid) {
        try {
            Base64Codec::decode(payload);
        } catch (...) {
            valid = false;
        }
    }

    return valid;
}

std::string ReportGenerator::jsonEscape(const std::string& s)
{
    std::ostringstream oss;
    for (int i = 0; i < static_cast<int>(s.size()); ++i) {
        const char c = s[i];
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
    const std::time_t now = std::time(nullptr);
    char buf[32] = {};
    const struct tm* gmt = std::gmtime(&now);
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", gmt);
    return std::string(buf);
}

} // namespace AIGD
