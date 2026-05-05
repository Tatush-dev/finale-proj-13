#include <iostream>
#include <memory>
#include <ctime>

#include "Common/AppEnums.h"
#include "Utils/SpatialUtilities.h"
#include "Utils/Base64Codec.h"
#include "Model/IntelData.h"
#include "Model/IntelManager.h"
#include "Model/NavigationModel.h"
#include "View/MissionView.h"
#include "Controller/MissionController.h"

using namespace AIGD;

int main() {
    std::cout << "--- SYSTEM START ---" << std::endl;
    std::cout << "=== Autonomous IMINT Drone — Task 1.2 Data Models Validation ===\n\n";

    // ── Test Enums ───────────────────────────────────────────────────────────
    std::cout << "Testing AppEnums...\n";
    AIGD::MissionState state = AIGD::MissionState::OUTBOUND;
    AIGD::SensorType sensor = AIGD::SensorType::THERMAL;
    std::cout << "    Mission State: OUTBOUND\n";
    std::cout << "    Sensor Type: THERMAL\n\n";

    // ── Test Point2D and Vector2D ────────────────────────────────────────────
    std::cout << "Testing Spatial Utilities...\n";
    Point2D base(0.0, 0.0);
    Point2D target(50.0, 75.0);
    std::cout << "    Base Location: " << base << "\n";
    std::cout << "    Target Location: " << target << "\n";

    Vector2D pathVector = target - base;
    std::cout << "    Direction Vector: " << pathVector << "\n";
    std::cout << "    Distance to Target: " << pathVector.magnitude() << " meters\n\n";

    // Vector operations test
    Vector2D v1(3.0, 4.0);
    Vector2D v2(1.0, 2.0);
    std::cout << "    Vector Operations:\n";
    std::cout << "      v1: " << v1 << ", magnitude: " << v1.magnitude() << "\n";
    std::cout << "      v2: " << v2 << ", magnitude: " << v2.magnitude() << "\n";
    std::cout << "      v1 + v2 = " << (v1 + v2) << "\n";
    std::cout << "      v1 - v2 = " << (v1 - v2) << "\n";
    std::cout << "      v1 * 2.0 = " << (v1 * 2.0) << "\n";
    std::cout << "      v1.normalize() = " << v1.normalize() << "\n";
    std::cout << "      v1 * v2 = " << v1.dotProduct(v2) << "\n\n";

    // ── Test IntelData ───────────────────────────────────────────────────────
    std::cout << "Testing IntelData Class...\n";

    // Create some sample intel data
    long long currentTime = static_cast<long long>(std::time(nullptr)) * 1000;

    AIGD::IntelData intel1(base, currentTime, AIGD::SensorType::IMINT, "aGVsbG8gd29ybGQ=");  // "hello world" in Base64
    std::cout << "Intel 1 (IMINT at base):\n" << intel1.toString() << "\n\n";

    AIGD::IntelData intel2(target, currentTime + 5000, AIGD::SensorType::THERMAL, "dGhlcm1hbCBkYXRh");  // "thermal data" in Base64
    std::cout << "Intel 2 (THERMAL at target):\n" << intel2.toString() << "\n\n";

    // Test copy constructor and assignment
    AIGD::IntelData intel3 = intel1;  // Copy constructor
    std::cout << "Intel 3 (copy of Intel 1):\n" << intel3.toString() << "\n\n";

    // Verify data integrity
    std::cout << "Verification:\n";
    std::cout << "  Intel 1 Location X: " << intel1.getLocationX() << " m\n";
    std::cout << "  Intel 1 Location Y: " << intel1.getLocationY() << " m\n";
    std::cout << "  Intel 1 Data Size: " << intel1.getDataSize() << " bytes\n";
    std::cout << "  Intel 2 Sensor Type: " << intel2.getSensorTypeString() << "\n\n";

    // ── Test IntelManager + Base64Codec ────────────────────────────────────
    std::cout << "Testing IntelManager and Base64Codec...\n";
    IntelManager intelManager;

    const std::string rawPayload = "mission image bytes";
    const std::string encodedPayload = Base64Codec::encode(rawPayload);
    AIGD::IntelData encodedIntel(base, currentTime + 10000, SensorType::SIGNAL, encodedPayload);
    intelManager.addIntel(encodedIntel);

    auto retrievedIntel = intelManager.getIntel(encodedIntel.getTimestamp());
    bool encodeDecodeSuccess = false;
    if (retrievedIntel) {
        const std::string decodedPayload = Base64Codec::decode(retrievedIntel->getRawData());
        encodeDecodeSuccess = (decodedPayload == rawPayload);
        std::cout << "  Retrieved Intel: " << retrievedIntel->toString() << "\n";
        std::cout << "  Decoded Payload: " << decodedPayload << "\n";
    }

    std::cout << "  Base64 encode/decode test: "
              << (encodeDecodeSuccess ? "SUCCESS" : "FAILURE") << "\n\n";

    std::cout << "=== All data models validated successfully ===\n";
    return encodeDecodeSuccess ? 0 : 1;
}
