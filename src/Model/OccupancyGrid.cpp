#include "OccupancyGrid.h"
#include <iostream>
#include <iomanip>
#include <cmath>

/**
 * Constructor: Initializes grid with specified dimensions.
 * All cells are initialized to 0.5 (unknown probability).
 */
OccupancyGrid::OccupancyGrid(int width, int length)
    : width(width), length(length) {
    grid.resize(length);
    for (int y = 0; y < length; ++y) {
        grid[y].resize(width, 0.5);  // Initialize all cells to 0.5 (unknown)
    }
}

/**
 * Updates a cell using soft Bayesian update logic.
 *
 * Formula: new_p = old_p * (1 - GRID_UPDATE_FACTOR) + target_p * GRID_UPDATE_FACTOR
 *
 * Bayesian reasoning:
 *  - If an obstacle is detected, probability nudges toward PROB_OCCUPIED_IF_DETECTED.
 *  - If free space is detected, it nudges toward PROB_FREE_IF_NOT_DETECTED.
 *  - α = GRID_UPDATE_FACTOR = 0.30 prevents a single noisy reading from flipping
 *    a cell past OCCUPANCY_THRESHOLD; three consistent hits are needed (~0.5→0.73).
 *
 * Out-of-bounds coordinates are silently ignored (guarded by the outer if-block
 * to maintain the Single-Entry Single-Exit principle — no early returns).
 */
void OccupancyGrid::UpdateCell(int x, int y, bool sensorObservedObstacle) {
    // Only update if coordinates fall within the valid grid extent.
    if (x >= 0 && x < width && y >= 0 && y < length) {
        double& cellProb = grid[y][x];

        // Select the Bayesian target probability based on sensor observation type.
        double targetProb = sensorObservedObstacle
                            ? PROB_OCCUPIED_IF_DETECTED
                            : PROB_FREE_IF_NOT_DETECTED;

        // Soft Bayesian blend: moves probability toward evidence without jumping.
        cellProb = cellProb * (1.0 - GRID_UPDATE_FACTOR) + targetProb * GRID_UPDATE_FACTOR;

        // Clamp probability to the valid range [0, 1].
        if (cellProb < 0.0) { cellProb = 0.0; }
        if (cellProb > 1.0) { cellProb = 1.0; }
    }
}

/**
 * Checks if a cell is considered occupied based on the occupancy threshold.
 * The bounds check short-circuits via && so grid[] is only accessed for valid
 * coordinates — combines both guards into a single return expression.
 * Returns false for out-of-bounds (planners treat unknown boundary as passable).
 */
bool OccupancyGrid::IsOccupied(int x, int y) const {
    return (x >= 0 && x < width && y >= 0 && y < length) && (grid[y][x] > OCCUPANCY_THRESHOLD);
}

/**
 * Returns the probability value of a cell.
 * Out-of-bounds coordinates return -1.0 as a sentinel so callers can
 * distinguish "invalid query" from valid probabilities near 0.
 * Ternary keeps the logic as a single expression without a branch return.
 */
double OccupancyGrid::GetCellProbability(int x, int y) const {
    return (x >= 0 && x < width && y >= 0 && y < length) ? grid[y][x] : -1.0;
}

/**
 * IOccupancyGrid interface adapter.
 * Converts coordinate-based updates to grid-based indices by truncating the
 * floating-point world coordinates to integer cell positions.
 * Any sensor reading above 0.5 is treated as an obstacle detection;
 * a boolean flag prevents duplicate processing within the same sensor frame.
 */
void OccupancyGrid::UpdateGridProbability(const Coordinates&         cell,
                                          const std::vector<double>& sensorData) {
    int gridX = static_cast<int>(cell.x);
    int gridY = static_cast<int>(cell.y);

    // Determine whether any reading in the frame exceeds the obstacle threshold.
    // The foundObstacle flag prevents re-triggering on subsequent readings.
    bool observedObstacle = false;
    bool foundObstacle    = false;
    for (double reading : sensorData) {
        if (!foundObstacle && reading > 0.5) {
            observedObstacle = true;
            foundObstacle    = true;
        }
    }

    UpdateCell(gridX, gridY, observedObstacle);
}

/**
 * Prints the grid to stdout for visualization and debugging.
 * Symbols:
 *   '.' = free     (probability 0.0–0.3)
 *   'o' = unknown  (probability 0.3–0.7)
 *   'X' = occupied (probability 0.7–1.0)
 * Rows are rendered top-to-bottom (y=0 at top of output).
 */
void OccupancyGrid::PrintGrid() const {
    std::cout << "\n+- Occupancy Grid (" << width << "x" << length << ") -+\n";

    for (int y = 0; y < length; ++y) {
        std::cout << "| ";
        for (int x = 0; x < width; ++x) {
            double prob   = grid[y][x];
            char   symbol;
            if (prob < 0.3) {
                symbol = '.';  // Free
            } else if (prob < 0.7) {
                symbol = 'o';  // Unknown
            } else {
                symbol = 'X';  // Occupied
            }
            std::cout << symbol;
        }
        std::cout << " |\n";
    }

    std::cout << "+" << std::string(width + 2, '-') << "+\n";
    std::cout << "Legend: . = free (0.0-0.3)  |  o = unknown (0.3-0.7)  |  X = occupied (0.7-1.0)\n";
}

/**
 * Resets all cells to 0.5 (unknown state).
 * Called between simulation runs to clear accumulated sensor evidence
 * without reconstructing the grid object.
 */
void OccupancyGrid::ResetGrid() {
    for (int y = 0; y < length; ++y) {
        for (int x = 0; x < width; ++x) {
            grid[y][x] = 0.5;
        }
    }
}
