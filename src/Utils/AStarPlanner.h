#pragma once
#include <vector>
#include "../Common/Types.h"
#include "../Model/OccupancyGrid.h"
#include "CostCalculator.h"

namespace AIGD {

/**
 * A* static global path planner.
 *
 * Finds an optimal path from start to target on an OccupancyGrid using the
 * A* search algorithm.  Cells with occupancy probability > 0.7 are treated
 * as impassable obstacles.  Step costs are computed via CostCalculator so
 * the planner respects the active mission priority profile.
 *
 * Heuristic : Manhattan distance (admissible for 4-directional movement).
 * Closed set: std::unordered_set for O(1) membership checks.
 * No break or continue statements are used anywhere in the implementation.
 */
class AStarPlanner {
public:
    /**
     * Finds an optimal path from start to target using the A* algorithm.
     *
     * @param start     Source cell  (x, y components map to integer grid indices).
     * @param target    Destination cell.
     * @param grid      Occupancy grid consulted for obstacle detection and risk values.
     * @param costCalc  Cost engine used to compute g(n) movement costs.
     * @return          Ordered waypoints from start to target (inclusive),
     *                  or an empty vector when no path exists.
     */
    std::vector<Coordinates> findPath(
        const Coordinates&   start,
        const Coordinates&   target,
        const OccupancyGrid& grid,
        CostCalculator&      costCalc
    );
};

} // namespace AIGD
