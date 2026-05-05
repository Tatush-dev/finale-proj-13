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
 * Updates a cell using Bayesian update logic (Log-Odds approach).
 * 
 * Bayesian Rule: P(occ|z) = P(z|occ) * P(occ) / P(z)
 * 
 * Simplified approach:
 * - If obstacle detected: increase probability toward PROB_OCCUPIED_IF_DETECTED
 * - If free space detected: decrease probability toward PROB_FREE_IF_NOT_DETECTED
 * - Use a soft update (doesn't jump to extremes, gradually converges)
 */
void OccupancyGrid::UpdateCell(int x, int y, bool sensorObservedObstacle) {
    // Boundary check
    if (x < 0 || x >= width || y < 0 || y >= length) {
        return;  // Silently ignore out-of-bounds updates
    }

    double& cellProb = grid[y][x];
    
    // Bayesian update: blend current probability with sensor observation
    double targetProb = sensorObservedObstacle 
                        ? PROB_OCCUPIED_IF_DETECTED 
                        : PROB_FREE_IF_NOT_DETECTED;
    
    // Soft update: move probability toward target with damping factor (0.3 = 30% update per call)
    const double UPDATE_FACTOR = 0.3;
    cellProb = cellProb * (1.0 - UPDATE_FACTOR) + targetProb * UPDATE_FACTOR;
    
    // Clamp to valid range [0, 1]
    if (cellProb < 0.0) cellProb = 0.0;
    if (cellProb > 1.0) cellProb = 1.0;
}

/**
 * Checks if a cell is considered occupied based on occupancy threshold.
 */
bool OccupancyGrid::IsOccupied(int x, int y) const {
    if (x < 0 || x >= width || y < 0 || y >= length) {
        return false;  // Out-of-bounds is considered free
    }
    return grid[y][x] > OCCUPANCY_THRESHOLD;
}

/**
 * Gets the probability value of a cell.
 */
double OccupancyGrid::GetCellProbability(int x, int y) const {
    if (x < 0 || x >= width || y < 0 || y >= length) {
        return -1.0;  // Invalid cell
    }
    return grid[y][x];
}

/**
 * IOccupancyGrid interface adapter.
 * Converts coordinate-based updates to grid-based indices.
 */
void OccupancyGrid::UpdateGridProbability(const Coordinates&         cell,
                                          const std::vector<double>& sensorData) {
    // Map world coordinates to grid indices (simple linear mapping)
    // Assumes world coordinates are in a reasonable range [0, width/height]
    int gridX = static_cast<int>(cell.x);
    int gridY = static_cast<int>(cell.y);
    
    // Determine observation: if any sensor reading indicates obstacle (> 0.5), flag it
    bool observedObstacle = false;
    for (double reading : sensorData) {
        if (reading > 0.5) {
            observedObstacle = true;
            break;
        }
    }
    
    UpdateCell(gridX, gridY, observedObstacle);
}

/**
 * Prints the grid to stdout for visualization.
 * Uses characters to represent probability ranges:
 * . = free (0.0-0.3)
 * o = unknown (0.3-0.7)
 * X = occupied (0.7-1.0)
 */
void OccupancyGrid::PrintGrid() const {
    std::cout << "\n+- Occupancy Grid (" << width << "x" << length << ") -+\n";
    
    for (int y = 0; y < length; ++y) {
        std::cout << "| ";
        for (int x = 0; x < width; ++x) {
            double prob = grid[y][x];
            char symbol;
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
 * Prints detailed probability values for specific cells (for debug).
 */
void OccupancyGrid::ResetGrid() {
    for (int y = 0; y < length; ++y) {
        for (int x = 0; x < width; ++x) {
            grid[y][x] = 0.5;  // Reset to unknown
        }
    }
}
