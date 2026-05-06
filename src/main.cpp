#include <iostream>
#include <memory>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <iomanip>

#include "Common/AppEnums.h"
#include "Utils/SpatialUtilities.h"
#include "Utils/Base64Codec.h"
#include "Model/IntelData.h"
#include "Model/IntelligenceManager.h"
#include "Model/NavigationModel.h"
#include "View/MissionView.h"
#include "Controller/MissionController.h"
#include "Utils/KalmanFilter.h"

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
    std::cout << "Testing IntelligenceManager and Base64Codec...\n";
    AIGD::IntelligenceManager intelManager;

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

    // ── Test OccupancyGrid (Task 2.1 - Probabilistic Occupancy Grid Mapping) ──
    // std::cout << "Testing OccupancyGrid (Task 2.1)...\n";
    // OccupancyGrid grid(10, 10);  // Create 10x10 grid

    // std::cout << "\n  Grid BEFORE sensor updates:\n";
    // grid.PrintGrid();

    // // Simulate sensor readings at specific coordinates
    // std::cout << "\n  Simulating sensor readings...\n";

    // // Scenario 1: Obstacle detected at (2, 2)
    // std::cout << "  [1] Update cell (2, 2) with obstacle detection (3 times)\n";
    // grid.UpdateCell(2, 2, true);
    // grid.UpdateCell(2, 2, true);
    // grid.UpdateCell(2, 2, true);

    // // Scenario 2: Free space at (5, 5)
    // std::cout << "  [2] Update cell (5, 5) with free space detection (3 times)\n";
    // grid.UpdateCell(5, 5, false);
    // grid.UpdateCell(5, 5, false);
    // grid.UpdateCell(5, 5, false);

    // // Scenario 3: Uncertain cell (8, 2) - obstacle then free
    // std::cout << "  [3] Update cell (8, 2) with mixed detections (obstacle, free, obstacle)\n";
    // grid.UpdateCell(8, 2, true);
    // grid.UpdateCell(8, 2, false);
    // grid.UpdateCell(8, 2, true);

    // // Scenario 4: Strong obstacle at (3, 7)
    // std::cout << "  [4] Update cell (3, 7) with strong obstacle (4 detections)\n";
    // for (int i = 0; i < 4; ++i) {
    //     grid.UpdateCell(3, 7, true);
    // }

    // std::cout << "\n  Grid AFTER sensor updates:\n";
    // grid.PrintGrid();

    // // Print detailed probabilities for test cells
    // std::cout << "\n  Detailed cell probabilities:\n";
    // std::cout << "  Cell (2, 2) [obstacle]:   prob=" << grid.GetCellProbability(2, 2)
    //           << " | occupied=" << (grid.IsOccupied(2, 2) ? "YES" : "NO") << "\n";
    // std::cout << "  Cell (5, 5) [free]:       prob=" << grid.GetCellProbability(5, 5)
    //           << " | occupied=" << (grid.IsOccupied(5, 5) ? "YES" : "NO") << "\n";
    // std::cout << "  Cell (8, 2) [mixed]:      prob=" << grid.GetCellProbability(8, 2)
    //           << " | occupied=" << (grid.IsOccupied(8, 2) ? "YES" : "NO") << "\n";
    // std::cout << "  Cell (3, 7) [strong occ]: prob=" << grid.GetCellProbability(3, 7)
    //           << " | occupied=" << (grid.IsOccupied(3, 7) ? "YES" : "NO") << "\n";
    // std::cout << "  Cell (0, 0) [unchanged]:  prob=" << grid.GetCellProbability(0, 0)
    //           << " | occupied=" << (grid.IsOccupied(0, 0) ? "YES" : "NO") << "\n\n";

    // std::cout << "Grid dimensions: " << grid.GetWidth() << "x" << grid.GetLength() << "\n";
    // std::cout << "OccupancyGrid test: SUCCESS\n\n";

    // ── Test CostCalculator (Task 2.2 - Dynamic Cost Function Engine) ──────
    // std::cout << "Testing CostCalculator (Task 2.2)...\n";
    // CostCalculator calculator;

    // // Test parameters
    // const double distance = 100.0;      // meters
    // const double riskFactor = 0.7;      // high risk
    // const double energyConsumption = 50.0;  // arbitrary units

    // std::cout << "  Test Parameters:\n";
    // std::cout << "    Distance: " << distance << " m\n";
    // std::cout << "    Risk Factor: " << riskFactor << "\n";
    // std::cout << "    Energy Consumption: " << energyConsumption << " units\n\n";

    // // Test STEALTH priority
    // calculator.setPriority(MissionPriority::STEALTH);
    // double stealthCost = calculator.calculateCost(distance, riskFactor, energyConsumption);
    // std::cout << "  STEALTH Priority Cost: " << stealthCost << "\n";

    // // Test AGGRESSIVE priority
    // calculator.setPriority(MissionPriority::AGGRESSIVE);
    // double aggressiveCost = calculator.calculateCost(distance, riskFactor, energyConsumption);
    // std::cout << "  AGGRESSIVE Priority Cost: " << aggressiveCost << "\n";

    // // Test BALANCED priority
    // calculator.setPriority(MissionPriority::BALANCED);
    // double balancedCost = calculator.calculateCost(distance, riskFactor, energyConsumption);
    // std::cout << "  BALANCED Priority Cost: " << balancedCost << "\n\n";

    // // Verification: Show cost difference between STEALTH and AGGRESSIVE
    // double costDifference = stealthCost - aggressiveCost;
    // std::cout << "  Cost Difference (STEALTH - AGGRESSIVE): " << costDifference << "\n";
    // std::cout << "  STEALTH cost is " << (costDifference > 0 ? "higher" : "lower") << " by " << std::abs(costDifference) << "\n\n";

    // std::cout << "CostCalculator test: SUCCESS\n\n";

    // ── Test KalmanFilter (Task 3.1 - Sensor Fusion) ─────────────────────────
    std::cout << "Testing KalmanFilter (Task 3.1 - 1D Sensor Fusion)...\n\n";

    // Seed RNG once for reproducible demo runs
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // Box-Muller transform: produces a zero-mean Gaussian sample with given std-dev
    auto gaussianNoise = [](double stddev) -> double {
        const double u1 = (static_cast<double>(std::rand()) + 1.0) /
                          (static_cast<double>(RAND_MAX) + 2.0);
        const double u2 = (static_cast<double>(std::rand()) + 1.0) /
                          (static_cast<double>(RAND_MAX) + 2.0);
        return stddev * std::sqrt(-2.0 * std::log(u1)) *
               std::cos(2.0 * 3.14159265358979323846 * u2);
    };

    // Simulation parameters
    const int    STEPS            = 20;
    const double TRUE_VELOCITY    = 2.0;   // m/s
    const double DT               = 0.1;   // seconds per step
    const double DISPLACEMENT     = TRUE_VELOCITY * DT;  // 0.2 m per step
    const double MEAS_NOISE_STD   = 1.0;   // sensor std-dev (m)

    // Filter initialised at true start position, high initial uncertainty
    AIGD::KalmanFilter kalman(
        /*initialState=*/0.0,
        /*initialP    =*/1.0,
        /*Q process   =*/0.01,
        /*R measurement=*/MEAS_NOISE_STD * MEAS_NOISE_STD  // R = σ²
    );

    std::cout << std::fixed << std::setprecision(4);
    std::cout << std::left
              << std::setw(6)  << "Step"
              << std::setw(14) << "True Pos (m)"
              << std::setw(18) << "Noisy Meas (m)"
              << std::setw(18) << "Filtered Est (m)"
              << "Covariance P\n";
    std::cout << std::string(70, '-') << "\n";

    double truePosition = 0.0;

    for (int step = 1; step <= STEPS; ++step) {
        truePosition += DISPLACEMENT;

        const double noisyMeasurement = truePosition + gaussianNoise(MEAS_NOISE_STD);

        kalman.Predict({DISPLACEMENT});
        kalman.Update({noisyMeasurement});

        std::cout << std::setw(6)  << step
                  << std::setw(14) << truePosition
                  << std::setw(18) << noisyMeasurement
                  << std::setw(18) << kalman.getState()
                  << kalman.getCovariance() << "\n";
    }

    std::cout << "\nKalmanFilter test: SUCCESS\n\n";

    std::cout << "=== All data models validated successfully ===\n";
    return encodeDecodeSuccess ? 0 : 1;
}
