#include "AStarPlanner.h"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <algorithm>

namespace AIGD {

// ── Internal node type (used only within this translation unit) ──────────────
namespace {

struct AStarNode {
    int x;
    int y;
    double g;       // Accumulated movement cost from start
    double h;       // Heuristic estimate to target (Manhattan distance)
    std::shared_ptr<AStarNode> parent;

    double f() const { return g + h; }
};

// Min-heap comparator: node with lower f has higher priority
struct NodeComparator {
    bool operator()(const std::shared_ptr<AStarNode>& a,
                    const std::shared_ptr<AStarNode>& b) const {
        return a->f() > b->f();
    }
};

} // anonymous namespace

// ── AStarPlanner::findPath ───────────────────────────────────────────────────
std::vector<Coordinates> AStarPlanner::findPath(
    const Coordinates&   start,
    const Coordinates&   target,
    const OccupancyGrid& grid,
    CostCalculator&      costCalc
) {
    const int startX  = static_cast<int>(start.x);
    const int startY  = static_cast<int>(start.y);
    const int targetX = static_cast<int>(target.x);
    const int targetY = static_cast<int>(target.y);

    // Encode (x, y) to a unique integer key for O(1) set/map lookups.
    // grid.GetLength() is the y-dimension, so the result is collision-free.
    auto encodePos = [&](int x, int y) -> int {
        return x * grid.GetLength() + y;
    };

    // Admissible Manhattan distance heuristic for 4-directional movement
    auto heuristic = [&](int x, int y) -> double {
        return static_cast<double>(std::abs(x - targetX) + std::abs(y - targetY));
    };

    // Open set (min-heap by f = g + h)
    std::priority_queue<
        std::shared_ptr<AStarNode>,
        std::vector<std::shared_ptr<AStarNode>>,
        NodeComparator
    > openSet;

    // Closed set — nodes whose optimal cost has been confirmed (O(1) lookup)
    std::unordered_set<int> closedSet;

    // Tracks the best g cost discovered so far for each open node to prune
    // stale heap entries without an explicit decrease-key operation
    std::unordered_map<int, double> bestGInOpen;

    auto startNode = std::make_shared<AStarNode>(AStarNode{
        startX, startY, 0.0, heuristic(startX, startY), nullptr
    });
    openSet.push(startNode);
    bestGInOpen[encodePos(startX, startY)] = 0.0;

    // Main A* loop 
    bool targetFound = false;
    std::shared_ptr<AStarNode> foundNode = nullptr;

    while (!openSet.empty() && !targetFound) {
        auto current = openSet.top();
        openSet.pop();

        const int key = encodePos(current->x, current->y);

        // Skip stale heap entries: node already settled via a cheaper path
        if (closedSet.find(key) == closedSet.end()) {
            closedSet.insert(key);

            if (current->x == targetX && current->y == targetY) {
                targetFound = true;
                foundNode   = current;
            } else {
                // 4-directional movement: right, left, down, up
                const int dx[4] = { 1, -1,  0,  0 };
                const int dy[4] = { 0,  0,  1, -1 };

                for (int i = 0; i < 4; ++i) {
                    const int nx = current->x + dx[i];
                    const int ny = current->y + dy[i];

                    const bool inBounds = (nx >= 0 && nx < grid.GetWidth() &&
                                           ny >= 0 && ny < grid.GetLength());
                    if (inBounds) {
                        const int  nKey      = encodePos(nx, ny);
                        const bool inClosed  = closedSet.find(nKey) != closedSet.end();
                        // Cells with probability > 0.7 are treated as obstacles
                        const bool isWall    = grid.IsOccupied(nx, ny);

                        if (!isWall && !inClosed) {
                            // g(n): weighted cost via CostCalculator using
                            // unit distance, cell risk, and unit energy per step
                            const double stepCost = costCalc.calculateCost(
                                /*distance=*/1.0,
                                /*riskFactor=*/grid.GetCellProbability(nx, ny),
                                /*energyConsumption=*/1.0
                            );
                            const double newG = current->g + stepCost;

                            const auto   it       = bestGInOpen.find(nKey);
                            const bool   improves = (it == bestGInOpen.end() ||
                                                     newG < it->second);
                            if (improves) {
                                bestGInOpen[nKey] = newG;
                                auto neighbor = std::make_shared<AStarNode>(AStarNode{
                                    nx, ny, newG, heuristic(nx, ny), current
                                });
                                openSet.push(neighbor);
                            }
                        }
                    }
                }
            }
        }
    }

    //  Reconstruct path by tracing parent pointers from goal to start 
    std::vector<Coordinates> path;
    if (targetFound) {
        auto node = foundNode;
        while (node != nullptr) {
            path.push_back(Coordinates(
                static_cast<double>(node->x),
                static_cast<double>(node->y),
                start.altitude
            ));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
    }
    return path;
}

} // namespace AIGD
