#pragma once
#include "../Common/Types.h"
#include "IOccupancyGrid.h"
#include <vector>

// ── Bayesian occupancy-grid tuning macros ─────────────────────────────────────
// These compile-time constants govern the probability model used by every cell
// in the OccupancyGrid.  Defined here (outside the class) so that the planners
// and other model components can reference them without instantiating the grid.
#define PROB_OCCUPIED_IF_DETECTED  0.85   ///< P(occupied | sensor detected obstacle)
#define PROB_FREE_IF_NOT_DETECTED  0.15   ///< P(occupied | sensor detected free space)
#define OCCUPANCY_THRESHOLD        0.7    ///< Probability above which a cell is impassable
#define GRID_UPDATE_FACTOR         0.3    ///< Soft-update weight α per Bayesian sensor step

/**
 * @class OccupancyGrid
 * @brief Probabilistic 2-D map of the Zaytun Quarter operating environment.
 *
 * Each cell stores a probability in [0, 1] representing the likelihood that
 * the cell is physically occupied by an obstacle.  Probabilities are updated
 * incrementally using a soft Bayesian rule so that a single noisy sensor hit
 * does not immediately block the path planner — the cell must accumulate
 * enough evidence to cross OCCUPANCY_THRESHOLD (0.7).
 *
 * Soft Bayesian update formula applied by UpdateCell():
 *   new_p = old_p * (1 - α) + target_p * α
 * where α = GRID_UPDATE_FACTOR = 0.30 and target_p is either
 * PROB_OCCUPIED_IF_DETECTED (0.85) or PROB_FREE_IF_NOT_DETECTED (0.15).
 *
 * Three consecutive obstacle detections raise a default cell from
 *   0.50 → 0.605 → 0.6785 → ~0.730  (exceeds OCCUPANCY_THRESHOLD).
 *
 * Probability semantics:
 *   0.0  — completely free space (confirmed clear)
 *   0.5  — unknown  (initial state for every cell)
 *   1.0  — completely occupied (confirmed obstacle)
 *
 * The path planners (AStarPlanner, DStarLitePlanner) call IsOccupied() to
 * treat any cell whose probability exceeds 0.7 as a hard obstacle.
 */
class OccupancyGrid : public IOccupancyGrid {
private:
    int                                width;    ///< Number of grid columns (x-axis)
    int                                length;   ///< Number of grid rows    (y-axis)
    std::vector<std::vector<double>>   grid;     ///< 2-D probability storage [row][col]

public:
    /**
     * @brief Constructs the occupancy grid with the given dimensions.
     *
     * All cells are initialised to 0.5 (maximum entropy / unknown state).
     * Default size matches the legacy 10×10 test grid; production missions
     * use dimensions parsed from config.json.
     *
     * @param width  Number of columns (x-axis extent).
     * @param length Number of rows    (y-axis extent).
     */
    OccupancyGrid(int width = 100, int length = 100);

    virtual ~OccupancyGrid() = default;

    /**
     * @brief Applies one Bayesian update to a single cell based on a sensor observation.
     *
     * Applies the soft update rule:
     *   new_p = old_p * (1 - GRID_UPDATE_FACTOR) + target_p * GRID_UPDATE_FACTOR
     *
     * Out-of-bounds coordinates are silently ignored (no update, no error).
     *
     * @param x                       Grid column index (0-based).
     * @param y                       Grid row index    (0-based).
     * @param sensorObservedObstacle  true  → target = PROB_OCCUPIED_IF_DETECTED
     *                                false → target = PROB_FREE_IF_NOT_DETECTED
     */
    void UpdateCell(int x, int y, bool sensorObservedObstacle);

    /**
     * @brief Returns true if the cell's probability exceeds OCCUPANCY_THRESHOLD.
     *
     * Out-of-bounds coordinates return false (treated as free by planners).
     *
     * @param x Grid column index.
     * @param y Grid row index.
     * @return  true when the cell is considered an obstacle.
     */
    bool IsOccupied(int x, int y) const;

    /**
     * @brief Returns the raw occupancy probability of a cell.
     *
     * @param x Grid column index.
     * @param y Grid row index.
     * @return  Probability in [0, 1], or -1.0 for out-of-bounds coordinates.
     */
    double GetCellProbability(int x, int y) const;

    /** @return Number of grid columns. */
    int GetWidth()  const { return width; }

    /** @return Number of grid rows. */
    int GetLength() const { return length; }

    /**
     * @brief IOccupancyGrid interface adapter: maps world coordinates to grid indices.
     *
     * Converts the floating-point Coordinates to integer cell indices (truncation),
     * then calls UpdateCell.  Any sensor reading > 0.5 is treated as an obstacle.
     *
     * @param cell       World coordinate of the target cell.
     * @param sensorData Vector of raw sensor readings; any value > 0.5 flags obstacle.
     */
    void UpdateGridProbability(const Coordinates&         cell,
                               const std::vector<double>& sensorData) override;

    /**
     * @brief Prints the full grid to stdout using ASCII symbols.
     *
     * Legend:  '.' free (p < 0.3)  |  'o' unknown (0.3–0.7)  |  'X' occupied (p > 0.7)
     * Rows are printed top-to-bottom (y=0 at the top).
     */
    void PrintGrid() const;

    /**
     * @brief Resets every cell to 0.5 (unknown state).
     *
     * Useful for re-running a simulation without re-constructing the grid object.
     */
    void ResetGrid();
};
