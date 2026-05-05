#pragma once
#include "../Common/Types.h"
#include "IOccupancyGrid.h"
#include <vector>

/**
 * Probabilistic Occupancy Grid Mapping (Task 2.1).
 * 
 * Maintains a 2D grid where each cell stores a probability [0,1] of being occupied.
 * Uses Bayesian update logic to refine probabilities based on sensor observations.
 * 
 * Probability Semantics:
 * - 0.0: Completely free space
 * - 0.5: Unknown (initial state)
 * - 1.0: Completely occupied
 * 
 * Occupancy Threshold: > 0.7 is considered occupied.
 */
class OccupancyGrid : public IOccupancyGrid {
private:
    int                                width;
    int                                length;
    std::vector<std::vector<double>>   grid;

    // Bayesian update parameters
    static constexpr double PROB_OCCUPIED_IF_DETECTED   = 0.85;   // P(occ|sensor=true)
    static constexpr double PROB_FREE_IF_NOT_DETECTED   = 0.15;   // P(occ|sensor=false)
    static constexpr double OCCUPANCY_THRESHOLD         = 0.7;    // Threshold for isOccupied()

public:
    /**
     * Constructor: Initializes the grid with specified dimensions.
     * All cells are initialized to 0.5 (unknown).
     */
    OccupancyGrid(int width = 100, int length = 100);

    virtual ~OccupancyGrid() = default;

    /**
     * Updates a single cell's occupancy probability using Bayesian logic.
     * 
     * @param x The grid column index.
     * @param y The grid row index.
     * @param sensorObservedObstacle True if sensor detected an obstacle, false otherwise.
     */
    void UpdateCell(int x, int y, bool sensorObservedObstacle);

    /**
     * Checks if a cell is considered occupied.
     * 
     * @param x The grid column index.
     * @param y The grid row index.
     * @return True if cell probability > OCCUPANCY_THRESHOLD, false otherwise.
     */
    bool IsOccupied(int x, int y) const;

    /**
     * Gets the probability value of a specific cell.
     * 
     * @param x The grid column index.
     * @param y The grid row index.
     * @return The occupancy probability [0,1], or -1.0 if out of bounds.
     */
    double GetCellProbability(int x, int y) const;

    /**
     * Returns the width of the grid.
     */
    int GetWidth() const { return width; }

    /**
     * Returns the length of the grid.
     */
    int GetLength() const { return length; }

    /**
     * IOccupancyGrid interface implementation.
     * Adapter method that converts coordinate-based updates to grid indices.
     */
    void UpdateGridProbability(const Coordinates&         cell,
                              const std::vector<double>& sensorData) override;

    /**
     * Prints the entire grid to stdout for visualization and debugging.
     */
    void PrintGrid() const;

    /**
     * Resets all cells to 0.5 (unknown state).
     */
    void ResetGrid();
};
