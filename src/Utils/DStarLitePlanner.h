#pragma once
#include <vector>
#include <unordered_map>
#include <queue>
#include <utility>
#include <limits>
#include "../Common/Types.h"
#include "../Model/OccupancyGrid.h"
#include "CostCalculator.h"

namespace AIGD {

/**
 * D* Lite incremental dynamic replanner
 *
 * Maintains a backward-search state (goal -> start) so that when a new
 * obstacle is discovered only the affected vertices are reprocessed, not
 * the entire graph.
 *
 * Key concepts:
 *  g(s)   – current cost estimate from s to goal.
 *  rhs(s) – one-step lookahead: min over successors s' of [c(s,s') + g(s')].
 *  A vertex is locally consistent when g(s) == rhs(s).
 *  Open list holds locally inconsistent vertices keyed by (k1, k2):
 *    k1 = min(g,rhs) + h(s, sStart) + km
 *    k2 = min(g,rhs)
 *
 */
class DStarLitePlanner {
public:
    using Key = std::pair<double, double>;

    /**
     * Initialises the planner and computes the initial shortest path.
     *
     * @param start    Starting cell (integer grid coordinates in x/y).
     * @param goal     Target cell.
     * @param grid     Occupancy grid consulted for obstacle detection.
     * @param costCalc Cost engine for movement step costs.
     */
    DStarLitePlanner(const Coordinates&   start,
                     const Coordinates&   goal,
                     const OccupancyGrid& grid,
                     CostCalculator&      costCalc);

    /** Returns the path computed at construction time (start → goal). */
    std::vector<Coordinates> getInitialPath() const;

    /**
     * Replan after a new obstacle is discovered.
     *
     * Marks newObstacle as occupied in the grid (3 sensor hits to exceed the
     * 0.7 threshold), propagates the cost change, and returns the replanned
     * path from currentPos to the original goal.
     *
     * @param currentPos   Drone's current grid cell when the obstacle is found.
     * @param newObstacle  Newly discovered obstacle cell.
     * @param grid         Occupancy grid (will be updated with the obstacle).
     * @param costCalc     Cost engine.
     * @return             Replanned path, or empty vector if goal is unreachable.
     */
    std::vector<Coordinates> updatePath(const Coordinates& currentPos,
                                        const Coordinates& newObstacle,
                                        OccupancyGrid&     grid,
                                        CostCalculator&    costCalc);

    /** Exposed for unit testing — runs the core D* Lite search loop. */
    void computeShortestPath(const OccupancyGrid& grid, CostCalculator& costCalc);

    /** Exposed for unit testing — updates rhs(x,y) and the open list. */
    void updateVertex(int x, int y, const OccupancyGrid& grid, CostCalculator& costCalc);

private:
    static constexpr double INF = std::numeric_limits<double>::infinity();

    int    width_,  length_;
    int    startX_, startY_;   // Current logical start (moves as drone advances)
    int    goalX_,  goalY_;
    double altitude_;          // Preserved for output Coordinates z-component
    double km_;                // Key modifier: compensates for start-position shifts

    std::unordered_map<int, double> g_;
    std::unordered_map<int, double> rhs_;

    struct PQEntry {
        Key key;
        int encoded;
        bool operator>(const PQEntry& o) const { return key > o.key; }
    };
    using MinHeap = std::priority_queue<PQEntry,
                                        std::vector<PQEntry>,
                                        std::greater<PQEntry>>;
    MinHeap                    openList_;
    std::unordered_map<int, Key> currentOpenKey_; // enc -> valid key for open nodes

    std::vector<Coordinates> initialPath_;  // Stored at construction

    // ── Encoding helpers ─────────────────────────────────────────────────────
    int encode (int x, int y) const { return x * length_ + y; }
    int decodeX(int enc)      const { return enc / length_; }
    int decodeY(int enc)      const { return enc % length_; }

    // ── Value accessors (return INF when absent from map) ────────────────────
    double getG  (int enc) const;
    double getRhs(int enc) const;

    // Manhattan distance heuristic from (x,y) to current start (backward search)
    double h(int x, int y) const;

    Key calculateKey(int x, int y) const;

    // ── Open-list management (lazy-deletion min-heap) ─────────────────────────
    void insertOpen   (int x, int y);
    void removeOpen   (int enc);
    bool isInOpen     (int enc) const;
    void cleanStaleTop();           // Pops stale heap entries until top is valid

    // ── Graph helpers ─────────────────────────────────────────────────────────
    std::vector<std::pair<int,int>> getNeighbors(int x, int y) const;

    // INF if destination is occupied, else calculateCost(1, risk, 1)
    double edgeCost(int x1, int y1, int x2, int y2,
                    const OccupancyGrid& grid,
                    const CostCalculator& costCalc) const;

    // Greedy path extraction following min(edgeCost + g) successors
    std::vector<Coordinates> extractPath(int fromX, int fromY,
                                         const OccupancyGrid&  grid,
                                         const CostCalculator& costCalc) const;
};

} // namespace AIGD
