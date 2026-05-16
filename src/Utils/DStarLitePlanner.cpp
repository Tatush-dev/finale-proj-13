#include "DStarLitePlanner.h"
#include <cmath>
#include <algorithm>

namespace AIGD {

// ── Value accessors ──────────────────────────────────────────────────────────
// Return INF when the encoded cell is absent from the cost map, meaning its
// g or rhs value has never been set (implicitly infinite — unvisited cell).
// Single-return form: result initialised to INF, overwritten on map hit.

double DStarLitePlanner::getG(int enc) const {
    auto it = g_.find(enc);
    double result = INF;
    if (it != g_.end()) {
        result = it->second;
    }
    return result;
}

double DStarLitePlanner::getRhs(int enc) const {
    auto it = rhs_.find(enc);
    double result = INF;
    if (it != rhs_.end()) {
        result = it->second;
    }
    return result;
}

// ── Heuristic ────────────────────────────────────────────────────────────────
// h = 0 (backward Dijkstra). CostCalculator returns values < 1 per hop while
// a Manhattan hop-count heuristic returns values ≥ 1 per hop, making it
// inadmissible and causing the D* Lite termination to fire too early — leaving
// stale g-values on neighbours of the start that confuse path extraction.
// h = 0 is trivially admissible and consistent, and correct for any cost model.
double DStarLitePlanner::h(int x, int y) const {
    (void)x; (void)y;
    return 0.0;
}

// ── Key calculation ──────────────────────────────────────────────────────────
// k1 = min(g, rhs) + h(s, sStart) + km
// k2 = min(g, rhs)
// Keys are compared lexicographically: k1 first, then k2.
DStarLitePlanner::Key DStarLitePlanner::calculateKey(int x, int y) const {
    int    enc    = encode(x, y);
    double minVal = std::min(getG(enc), getRhs(enc));
    return { minVal + h(x, y) + km_, minVal };
}

// ── Open-list management ─────────────────────────────────────────────────────

void DStarLitePlanner::insertOpen(int x, int y) {
    int enc      = encode(x, y);
    Key k        = calculateKey(x, y);
    currentOpenKey_[enc] = k;
    openList_.push({ k, enc });
}

void DStarLitePlanner::removeOpen(int enc) {
    // Lazy deletion: simply erase the canonical key entry.
    // The stale heap entry will be discarded when it surfaces at cleanStaleTop().
    currentOpenKey_.erase(enc);
}

bool DStarLitePlanner::isInOpen(int enc) const {
    return currentOpenKey_.count(enc) > 0;
}

// Discards stale heap entries from the top until the top is valid or the heap
// is empty. A stale entry is one whose stored key no longer matches
// currentOpenKey_ (node was removed or re-inserted with a newer key).
void DStarLitePlanner::cleanStaleTop() {
    bool foundValid = false;
    while (!openList_.empty() && !foundValid) {
        const PQEntry& top = openList_.top();
        auto it = currentOpenKey_.find(top.encoded);
        bool stale = (it == currentOpenKey_.end()) || (it->second != top.key);
        if (stale) {
            openList_.pop();
        } else {
            foundValid = true;
        }
    }
}

// ── Neighbor enumeration ─────────────────────────────────────────────────────

std::vector<std::pair<int,int>> DStarLitePlanner::getNeighbors(int x, int y) const {
    std::vector<std::pair<int,int>> result;
    const int dx[4] = {  1, -1,  0,  0 };
    const int dy[4] = {  0,  0,  1, -1 };
    for (int i = 0; i < 4; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (nx >= 0 && nx < width_ && ny >= 0 && ny < length_) {
            result.push_back({ nx, ny });
        }
    }
    return result;
}

// ── Edge cost ────────────────────────────────────────────────────────────────
// Returns INF when the destination cell is occupied (impassable).
// Otherwise delegates to the mission cost function: Cost = Distance + Risk + Energy.
double DStarLitePlanner::edgeCost(int x1, int y1, int x2, int y2,
                                  const OccupancyGrid& grid,
                                  const CostCalculator& costCalc) const {
    (void)x1; (void)y1; // source cell not needed for this cost model
    if (grid.IsOccupied(x2, y2)) {
        return INF;
    }
    return costCalc.calculateCost(
        /*distance=*/       1.0,
        /*riskFactor=*/     grid.GetCellProbability(x2, y2),
        /*energyConsumption=*/1.0
    );
}

// ── updateVertex ─────────────────────────────────────────────────────────────
// Recomputes rhs(x,y) from its successors (= neighbors in an undirected grid)
// and updates the vertex's membership in the open list.
void DStarLitePlanner::updateVertex(int x, int y,
                                    const OccupancyGrid& grid,
                                    CostCalculator& costCalc) {
    int  enc    = encode(x, y);
    bool isGoal = (x == goalX_ && y == goalY_);

    if (!isGoal) {
        // rhs(u) = min over neighbors s' of [c(u, s') + g(s')]
        double minRhs = INF;
        std::vector<std::pair<int,int>> neighbors = getNeighbors(x, y);
        for (int i = 0; i < static_cast<int>(neighbors.size()); ++i) {
            int nx = neighbors[i].first;
            int ny = neighbors[i].second;
            double c   = edgeCost(x, y, nx, ny, grid, costCalc);
            double val = (c < INF) ? (c + getG(encode(nx, ny))) : INF;
            if (val < minRhs) {
                minRhs = val;
            }
        }
        rhs_[enc] = minRhs;
    }

    // Remove from open list if currently present
    if (isInOpen(enc)) {
        removeOpen(enc);
    }

    // Reinsert if locally inconsistent (g ≠ rhs)
    if (getG(enc) != getRhs(enc)) {
        insertOpen(x, y);
    }
}

// ── computeShortestPath ───────────────────────────────────────────────────────
// Core D* Lite loop. Processes the open list until the start vertex is locally
// consistent and no open vertex has a key smaller than the start's key.
//
// Each iteration either:
//   a) Re-inserts a vertex whose key has increased since it was enqueued
//      (key was stale — the vertex is placed back with the correct key), or
//   b) Processes a truly inconsistent vertex:
//      - Overconsistent  (g > rhs): lower g to rhs, propagate to neighbours.
//      - Underconsistent (g < rhs): set g to INF, update vertex + neighbours.
void DStarLitePlanner::computeShortestPath(const OccupancyGrid& grid,
                                           CostCalculator& costCalc) {
    cleanStaleTop();

    int  startEnc      = encode(startX_, startY_);
    bool shouldContinue = true;

    while (shouldContinue) {
        bool heapEmpty = openList_.empty();

        if (heapEmpty) {
            shouldContinue = false;
        } else {
            Key topKey   = openList_.top().key;
            Key startKey = calculateKey(startX_, startY_);
            double gS    = getG(startEnc);
            double rhsS  = getRhs(startEnc);

            bool topBelowStart  = (topKey < startKey);
            bool startInconsist = (gS != rhsS);

            if (!topBelowStart && !startInconsist) {
                // Start is locally consistent and no better-keyed vertex exists
                shouldContinue = false;
            } else {
                // Pop the minimum-key vertex for processing
                PQEntry entry = openList_.top();
                openList_.pop();

                int enc = entry.encoded;
                int ux  = decodeX(enc);
                int uy  = decodeY(enc);

                Key knew = calculateKey(ux, uy);

                if (entry.key < knew) {
                    // Vertex key has increased since insertion; reinsert with correct key.
                    // Update currentOpenKey_ then push; the old entry is already popped.
                    currentOpenKey_[enc] = knew;
                    openList_.push({ knew, enc });
                } else {
                    // Remove from canonical map so updateVertex can reinsert if needed
                    currentOpenKey_.erase(enc);

                    double g_u   = getG(enc);
                    double rhs_u = getRhs(enc);

                    if (g_u > rhs_u) {
                        // Overconsistent: g too high, lower it to rhs
                        g_[enc] = rhs_u;
                        std::vector<std::pair<int,int>> nbrs = getNeighbors(ux, uy);
                        for (int i = 0; i < static_cast<int>(nbrs.size()); ++i) {
                            updateVertex(nbrs[i].first, nbrs[i].second, grid, costCalc);
                        }
                    } else {
                        // Underconsistent: g too low, set to INF and propagate
                        g_[enc] = INF;
                        // Update u itself (its rhs may also need correction)
                        updateVertex(ux, uy, grid, costCalc);
                        std::vector<std::pair<int,int>> nbrs = getNeighbors(ux, uy);
                        for (int i = 0; i < static_cast<int>(nbrs.size()); ++i) {
                            updateVertex(nbrs[i].first, nbrs[i].second, grid, costCalc);
                        }
                    }
                }

                // Refresh the top of the heap and re-evaluate the loop condition
                cleanStaleTop();
            }
        }
    }
}

// ── extractPath ───────────────────────────────────────────────────────────────
// Builds a path from (fromX, fromY) to goal by greedily following the
// neighbor with minimum [edgeCost + effectiveG] at each step, where
// effectiveG = max(g, rhs).  Using max guards against underconsistent vertices
// whose stale g < rhs (= true cost), which could otherwise appear cheaper than
// they really are and cause path-extraction cycles.
//
// Single-return form: the early-exit (unreachable start) is handled by wrapping
// the path-building loop in an else block rather than an early return statement.
std::vector<Coordinates> DStarLitePlanner::extractPath(int fromX, int fromY,
                                                        const OccupancyGrid&  grid,
                                                        const CostCalculator& costCalc) const {
    std::vector<Coordinates> path;

    int startEnc2 = encode(fromX, fromY);
    bool unreachable = (std::max(getG(startEnc2), getRhs(startEnc2)) >= INF &&
                        !(fromX == goalX_ && fromY == goalY_));

    // Only attempt path extraction if the start cell is reachable.
    if (!unreachable) {
        int cx = fromX;
        int cy = fromY;
        int maxSteps = width_ * length_; // Safety bound against cycles
        int steps    = 0;

        bool reachedGoal = (cx == goalX_ && cy == goalY_);

        while (!reachedGoal && steps < maxSteps) {
            path.push_back(Coordinates(static_cast<double>(cx),
                                       static_cast<double>(cy),
                                       altitude_));

            // Advance to the neighbor that minimises edgeCost(cx,cy,n) + effectiveG(n)
            std::vector<std::pair<int,int>> nbrs = getNeighbors(cx, cy);
            int    bestX  = -1, bestY = -1;
            double bestVal = INF;

            for (int i = 0; i < static_cast<int>(nbrs.size()); ++i) {
                int nx = nbrs[i].first;
                int ny = nbrs[i].second;
                double c          = edgeCost(cx, cy, nx, ny, grid, costCalc);
                int    nEnc       = encode(nx, ny);
                double effectiveG = std::max(getG(nEnc), getRhs(nEnc));
                double val        = (c < INF) ? (c + effectiveG) : INF;
                if (val < bestVal) {
                    bestVal = val;
                    bestX   = nx;
                    bestY   = ny;
                }
            }

            if (bestX == -1 || bestVal >= INF) {
                // No reachable successor: goal is blocked — clear path to signal failure.
                path.clear();
                reachedGoal = true;  // exit loop; empty path signals failure
            } else {
                cx = bestX;
                cy = bestY;
                reachedGoal = (cx == goalX_ && cy == goalY_);
                ++steps;
            }
        }

        // Append goal node if we reached it successfully
        if (cx == goalX_ && cy == goalY_) {
            path.push_back(Coordinates(static_cast<double>(goalX_),
                                       static_cast<double>(goalY_),
                                       altitude_));
        }
    }

    return path;
}

// ── Constructor ───────────────────────────────────────────────────────────────
// Initialises D* Lite state:
//  – All g and rhs values are implicitly INF (absent from maps).
//  – rhs(goal) = 0; goal is inserted into the open list.
//  – computeShortestPath() is run to build the initial g-value surface.
DStarLitePlanner::DStarLitePlanner(const Coordinates&   start,
                                   const Coordinates&   goal,
                                   const OccupancyGrid& grid,
                                   CostCalculator&      costCalc)
    : width_   (grid.GetWidth()),
      length_  (grid.GetLength()),
      startX_  (static_cast<int>(start.x)),
      startY_  (static_cast<int>(start.y)),
      goalX_   (static_cast<int>(goal.x)),
      goalY_   (static_cast<int>(goal.y)),
      altitude_(start.altitude),
      km_      (0.0)
{
    int goalEnc = encode(goalX_, goalY_);
    rhs_[goalEnc] = 0.0;
    insertOpen(goalX_, goalY_);

    computeShortestPath(grid, costCalc);

    // Store the initial path so getInitialPath() needs no grid/costCalc parameters
    initialPath_ = extractPath(startX_, startY_, grid, costCalc);
}

// ── getInitialPath ────────────────────────────────────────────────────────────

std::vector<Coordinates> DStarLitePlanner::getInitialPath() const {
    return initialPath_;
}

// ── updatePath ────────────────────────────────────────────────────────────────
// When a new obstacle is discovered the caller provides:
//   currentPos   – where the drone is right now
//   newObstacle  – the obstacle cell (will be marked in the grid)
//   grid         – occupancy grid (modified here)
//   costCalc     – cost engine
//
// Steps:
//  1. Advance the logical start to currentPos (update km_ accordingly).
//  2. Mark newObstacle in the grid (3 sensor hits exceed the 0.7 threshold).
//  3. Call updateVertex on the obstacle's neighbors — these are the vertices
//     whose rhs values may have been based on the now-impassable edge.
//  4. Run computeShortestPath() to restore consistency.
//  5. Extract and return the replanned path from currentPos.
std::vector<Coordinates> DStarLitePlanner::updatePath(const Coordinates& currentPos,
                                                       const Coordinates& newObstacle,
                                                       OccupancyGrid&     grid,
                                                       CostCalculator&    costCalc) {
    int newStartX = static_cast<int>(currentPos.x);
    int newStartY = static_cast<int>(currentPos.y);

    // km_ accumulates h(oldStart, newStart) each time the start position moves.
    // This compensates for the heuristic shift without rebuilding the open list.
    km_ += h(newStartX, newStartY);  // h uses old (startX_, startY_) here
    startX_ = newStartX;
    startY_ = newStartY;

    int obstX = static_cast<int>(newObstacle.x);
    int obstY = static_cast<int>(newObstacle.y);

    // Mark obstacle: 3 UpdateCell(true) calls move probability
    // 0.5 → 0.605 → 0.6785 → ~0.730, exceeding the 0.7 occupied threshold.
    grid.UpdateCell(obstX, obstY, true);
    grid.UpdateCell(obstX, obstY, true);
    grid.UpdateCell(obstX, obstY, true);

    // Update all neighbors of the new obstacle: their rhs values may have been
    // based on an edge through the now-occupied cell.
    std::vector<std::pair<int,int>> obstNbrs = getNeighbors(obstX, obstY);
    for (int i = 0; i < static_cast<int>(obstNbrs.size()); ++i) {
        updateVertex(obstNbrs[i].first, obstNbrs[i].second, grid, costCalc);
    }
    // Also update the obstacle cell itself in case it was a relay node
    updateVertex(obstX, obstY, grid, costCalc);

    computeShortestPath(grid, costCalc);

    return extractPath(startX_, startY_, grid, costCalc);
}

} // namespace AIGD
