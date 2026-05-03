#pragma once
#include "../Common/Types.h"
#include <vector>

/**
 * Abstract interface for the probabilistic occupancy grid.
 * Each cell holds a probability [0,1] of being occupied by an obstacle/threat.
 */
class IOccupancyGrid {
public:
    virtual ~IOccupancyGrid() = default;

    // Updates a cell's occupancy probability given new sensor data.
    // sensorData: raw sensor readings relevant to this cell
    virtual void UpdateGridProbability(const Coordinates&         cell,
                                       const std::vector<double>& sensorData) = 0;
};
