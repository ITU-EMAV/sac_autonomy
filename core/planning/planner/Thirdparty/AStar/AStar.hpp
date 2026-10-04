#ifndef ASTAR_HPP
#define ASTAR_HPP

#define MAX_ITER 50000

#include <vector>
#include <cmath>
#include <functional>
#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <memory> 
#include <unordered_set>
#include <chrono>


//--------------------------------------------------------------------------------------
// Vec2i: Represents a 2-dimensional vector or coordinate on a grid
//--------------------------------------------------------------------------------------
struct Vec2i {
    int x;  // x-coordinate (column)
    int y;  // y-coordinate (row)

    // Default constructor: initializes coordinates to (0,0)
    Vec2i() : x(0), y(0) {}

    // Parameterized constructor: initializes coordinates to (_x, _y)
    Vec2i(int _x, int _y) : x(_x), y(_y) {}

    // Overload the addition operator to add two vectors coordinate-wise
    Vec2i operator + (const Vec2i& rhs) const {
        return Vec2i{x + rhs.x, y + rhs.y};
    }

    // Equality operator: returns true if both coordinates match exactly
    bool operator == (const Vec2i& other) const {
        return (x == other.x && y == other.y);
    }
    
    // Inequality operator: returns true if either coordinate differs
    bool operator != (const Vec2i& other) const {
        return !(*this == other);
    }
};

//--------------------------------------------------------------------------------------
// Vec2iHash: Hash function for Vec2i to enable its use in unordered_map and unordered_set
//--------------------------------------------------------------------------------------
struct Vec2iHash {
    std::size_t operator()(const Vec2i& v) const {
        // Combines the hash values of x and y coordinates.
        // The bit shifting helps reduce collisions.
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 1);
    }
};

namespace AStar
{
    using uint = unsigned int; // alias for unsigned int

    //----------------------------------------------------------------------------------
    // Node: Represents a state in the A* search with position and orientation (theta)
    //----------------------------------------------------------------------------------

    struct Node {
        Vec2i coordinates;
        double theta;
        uint G;
        uint H;
        std::shared_ptr<Node> parent;

        Node(const Vec2i& coords, double heading, std::shared_ptr<Node> p = nullptr)
            : coordinates(coords), theta(heading), G(0), H(0), parent(p)
        {}

        inline uint getScore() const {
            return G + H;
        }
    };

    //----------------------------------------------------------------------------------
    // NodeKey: Unique key for a node, combining x, y, and discretized theta
    //----------------------------------------------------------------------------------
    struct NodeKey {
        int x;
        int y;
        int theta; // Discretized angle (in degrees)

        // Equality operator to check if two NodeKey instances represent the same state.
        bool operator==(const NodeKey& other) const {
            return x == other.x && y == other.y && theta == other.theta;
        }
    };

    //----------------------------------------------------------------------------------
    // NodeKeyHash: Hash function for NodeKey, necessary for using it in unordered_map.
    //----------------------------------------------------------------------------------
    struct NodeKeyHash {
        std::size_t operator()(const NodeKey& key) const {
            // Combines the hash values for x, y, and theta.
            std::size_t hx = std::hash<int>()(key.x);
            std::size_t hy = std::hash<int>()(key.y);
            std::size_t ht = std::hash<int>()(key.theta);
            return hx ^ (hy << 1) ^ (ht << 2);
        }
    };

    //----------------------------------------------------------------------------------
    // makeNodeKey: Utility function to generate a NodeKey from a coordinate and theta value.
    //              The theta is rounded and normalized to [0, 360) degrees.
    //----------------------------------------------------------------------------------
    inline NodeKey makeNodeKey(const Vec2i& coord, double theta) {
        NodeKey key;
        key.x = coord.x;
        key.y = coord.y;
        // Round theta to the nearest integer and normalize within the range 0-359
        key.theta = static_cast<int>(std::round(theta)) % 360;
        if (key.theta < 0)
            key.theta += 360;
        return key;
    }

    //----------------------------------------------------------------------------------
    // Heuristic functions namespace: Provides different heuristics for A* search.
    //----------------------------------------------------------------------------------
    namespace Heuristic {

        // getDelta: Computes the absolute difference (delta) between two points along each axis.
        inline Vec2i getDelta(const Vec2i& source, const Vec2i& target) {
            return {std::abs(source.x - target.x), std::abs(source.y - target.y)};
        }

        // manhattan: Manhattan distance heuristic (scaled by a factor of 10).
        //            Suitable for grid-based movement.
        inline uint manhattan(const Vec2i& source, const Vec2i& target) {
            auto delta = getDelta(source, target);
            return 10 * (delta.x + delta.y);
        }

        // euclidean: Euclidean distance heuristic (scaled by a factor of 10).
        //            Provides a straight-line distance between two points.
        inline uint euclidean(const Vec2i& source, const Vec2i& target) {
            auto delta = getDelta(source, target);
            double dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
            return static_cast<uint>(10.0 * dist);
        }

        // nonholonomic: A heuristic for systems with nonholonomic constraints.
        //                Considers both position and orientation (theta) differences.
        inline uint nonholonomic(const Vec2i& sourcePos, double sourceTheta,
                                 const Vec2i& targetPos, double targetTheta)
        {
            // Compute the positional difference using Euclidean distance scaled by 10.
            auto deltaPos = getDelta(sourcePos, targetPos);
            uint costPos = static_cast<uint>(10.0 *
                           std::sqrt(deltaPos.x * deltaPos.x + deltaPos.y * deltaPos.y));

            // Compute the orientation difference (in degrees) and normalize it.
            double diffTheta = std::fabs(sourceTheta - targetTheta);
            if (diffTheta > 180.0) {
                diffTheta = 360.0 - diffTheta;
            }
            // The cost for orientation is taken as the degree difference.
            uint costTheta = static_cast<uint>(diffTheta);

            // Total heuristic cost is the sum of positional and orientation costs.
            return costPos + costTheta;
        }
    }

    //----------------------------------------------------------------------------------
    // NodeComparator: Functor used to compare two nodes based on their total cost (F score).
    //                 This is required for the priority queue in the A* algorithm.
    //----------------------------------------------------------------------------------
    struct NodeComparator {
        bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
            // The node with the smaller F score (G+H) should have higher priority.
            return a->getScore() > b->getScore();
        }
    };

    //----------------------------------------------------------------------------------
    // Generator: Main class implementing the A* search algorithm.
    //            It encapsulates world configuration, collision handling, and path finding.
    //----------------------------------------------------------------------------------
    class Generator {
    public:
        // Constructor: Initializes default world size, allowed heading changes,
        // default heuristic function, and clearance (safety margin).
        Generator() {
            worldSize = {27, 27}; // Default grid size

            // Allowed heading changes in degrees for each move.
            headingChanges = {-20, -15, -10, 0, 10, 15, 20};

            // Set default heuristic to nonholonomic which accounts for both position and orientation.
            heuristicFunc = [](const Vec2i& sPos, double sTheta,
                               const Vec2i& tPos, double tTheta)
            {
                return Heuristic::nonholonomic(sPos, sTheta, tPos, tTheta);
            };

            // Clearance: the number of cells around obstacles to avoid.
            clearance = 0;

            // Cost (in score units) charged per cell of lateral distance from the centerline.
            centerline_contributing_factor = 1.0;

            // Length of a single expansion, in grid cells.
            curvature_contributing_factor = 1.0;

            // Cost charged per degree of heading change.
            turn_penalty = 0.5;

            // Wall-clock budget for one search, in seconds. Reaching it returns the
            // best node found so far instead of stalling the caller.
            time_budget = 0.05;
        }

        // setWorldSize: Configures the size of the world/grid.
        void setWorldSize(const Vec2i& ws) {
            worldSize = ws;
            const std::size_t cells =
                static_cast<std::size_t>(worldSize.x) * static_cast<std::size_t>(worldSize.y);
            traversalCosts.assign(cells, 0.0);
            centerlineDistance.assign(cells, 0);
            hasCenterline = false;
        }

        // removeCollision: Removes a collision (obstacle) at a specific coordinate.
        void removeCollision(const Vec2i& coord) {
            walls.erase(coord);
        }

        // clearCollisions: Clears all obstacles from the world.
        void clearCollisions() {
            walls.clear();
        }

        void clearTraversalCosts() {
            std::fill(traversalCosts.begin(), traversalCosts.end(), 0.0);
        }

        // setTraversalCost: Cost of *occupying* a cell, expressed per cell of travel.
        //                   0 means "perfectly fine", larger values are progressively
        //                   less desirable. Lethal cells must be added as collisions.
        void setTraversalCost(const Vec2i& coord, double cost) {
            if (coord.x >= 0 && coord.x < worldSize.x &&
                coord.y >= 0 && coord.y < worldSize.y) {
                traversalCosts[coord.y * worldSize.x + coord.x] = std::max(0.0, cost);
            }
        }

        // addCollision: Adds an obstacle at a specific coordinate.
        void addCollision(const Vec2i& coord) {
            walls.insert(coord);
        }

        // setClearance: Sets the safety clearance (buffer zone) around obstacles.
        void setClearance(int c) {
            clearance = c;
        }

        // setCurbatureContributingFactor: Sets the factor that increases radius of curvature
        void setCurvatureContributingFactor(double factor) {
            curvature_contributing_factor = factor;
        }

        // Define a function type for 3D heuristic (considering x, y, and theta)
        using Heuristic3D = std::function<uint(const Vec2i&, double,
                                               const Vec2i&, double)>;
        // setHeuristic: Allows setting a custom heuristic function.
        void setHeuristic(Heuristic3D func) {
            heuristicFunc = func;
        }

        // setCenterline: Stores the centerline and precomputes, once per plan, the
        //                distance from every grid cell to the nearest centerline cell.
        //                This turns the per-expansion centerline lookup into O(1).
        void setCenterline(const std::vector<Vec2i>& centerline_){
            this->centerline = centerline_;
            buildCenterlineField();
        }

        void setCenterlineContributingFactor(double factor){
            this->centerline_contributing_factor = factor;
        }

        // setTurnPenalty: Cost charged per degree of heading change on a step.
        void setTurnPenalty(double penalty){
            this->turn_penalty = std::max(0.0, penalty);
        }

        // setTimeBudget: Wall-clock limit for a single findPath call, in seconds.
        //                Zero or less disables the limit. An unreachable goal would
        //                otherwise run all MAX_ITER expansions before giving up.
        void setTimeBudget(double seconds){
            this->time_budget = seconds;
        }

        //----------------------------------------------------------------------------------
        // findPath: Executes the A* search algorithm to find a path from the source state
        //           to the target state.
        //
        // Parameters:
        // - sourcePos: Starting grid position.
        // - sourceTheta: Starting orientation (in degrees).
        // - targetPos: Goal grid position.
        // - targetTheta: Desired goal orientation (in degrees).
        //
        // Returns:
        // - A vector of Node pointers representing the path from start to goal.
        //   If an exact path is not found, returns the best available path.
        //----------------------------------------------------------------------------------
        std::vector<std::shared_ptr<AStar::Node>> findPath(const Vec2i& sourcePos, double sourceTheta,
                                    const Vec2i& targetPos, double targetTheta)
        {
            std::priority_queue<std::shared_ptr<AStar::Node>, std::vector<std::shared_ptr<AStar::Node>>, NodeComparator> openQueue;
            std::unordered_map<NodeKey, std::shared_ptr<AStar::Node>, NodeKeyHash> openMap;
            std::unordered_set<NodeKey, NodeKeyHash> closedSet;

            std::shared_ptr<AStar::Node> startNode = std::make_shared<Node>(sourcePos, sourceTheta);
            startNode->G = 0;
            startNode->H = heuristicFunc(sourcePos, sourceTheta, targetPos, targetTheta);
            NodeKey startKey = makeNodeKey(sourcePos, sourceTheta);
            openQueue.push(startNode);
            openMap[startKey] = startNode;

            std::shared_ptr<AStar::Node> bestNode = startNode;
            uint bestHeuristic = startNode->H;
            std::shared_ptr<AStar::Node> goalNode = nullptr;

            iter = 0;

            // Length of one expansion, in grid cells. The heuristic charges 10 score
            // units per cell, so the step cost must do the same or A* degenerates into
            // a greedy best-first search that ignores traversal/centerline costs.
            const double stepLength = std::max(curvature_contributing_factor, 1e-3);
            const uint moveCost = static_cast<uint>(std::lround(10.0 * stepLength));

            const auto searchStart = std::chrono::steady_clock::now();

            while (!openQueue.empty() && iter < MAX_ITER) {
                iter++;

                // Checked every 256 expansions so the clock read stays negligible.
                if (time_budget > 0.0 && (iter & 0xFF) == 0) {
                    const double elapsed = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - searchStart).count();
                    if (elapsed > time_budget)
                        break;
                }

                std::shared_ptr<AStar::Node> current = openQueue.top();
                openQueue.pop();

                NodeKey currentKey = makeNodeKey(current->coordinates, current->theta);
                if (closedSet.find(currentKey) != closedSet.end())
                    continue;

                closedSet.insert(currentKey);

                if (current->H < bestHeuristic) {
                    bestHeuristic = current->H;
                    bestNode = current;
                }

                if (current->coordinates == targetPos) {
                    goalNode = current;
                    break;
                }

                for (double dTheta : headingChanges) {
                    double newTheta = current->theta + dTheta;
                    newTheta = std::fmod(newTheta, 360.0);
                    if (newTheta < 0)
                        newTheta += 360.0;

                    double rad = newTheta * M_PI / 180.0;
                    double newX = current->coordinates.x + curvature_contributing_factor * std::cos(rad);
                    double newY = current->coordinates.y + curvature_contributing_factor * std::sin(rad);
                    Vec2i newCoord(static_cast<int>(std::round(newX)),
                                static_cast<int>(std::round(newY)));

                    if ((newCoord != targetPos) && detectCollision(newCoord))
                        continue;

                    // The goal is allowed to sit outside the grid check above; guard the
                    // cost lookups so we never index outside the cost fields.
                    if (newCoord.x < 0 || newCoord.x >= worldSize.x ||
                        newCoord.y < 0 || newCoord.y >= worldSize.y)
                        continue;

                    const std::size_t cellIdx =
                        static_cast<std::size_t>(newCoord.y) * worldSize.x + newCoord.x;

                    // Distance term, on the same 10-units-per-cell scale as the heuristic.
                    uint stepCost = moveCost;
                    // Steering effort, proportional to how hard the step turns.
                    stepCost += static_cast<uint>(
                        std::lround(turn_penalty * std::fabs(dTheta)));

                    // Per-cell penalties are costs *per cell of travel*, so integrating
                    // them over the step keeps tuning independent of the step length.
                    double cellCost = traversalCosts[cellIdx];
                    if (centerline_contributing_factor > 0.0 && hasCenterline) {
                        cellCost += centerline_contributing_factor *
                                    (centerlineDistance[cellIdx] / 10.0);
                    }
                    uint newG = current->G + stepCost +
                                static_cast<uint>(std::lround(cellCost * stepLength));

                    NodeKey neighborKey = makeNodeKey(newCoord, newTheta);
                    if (closedSet.find(neighborKey) != closedSet.end())
                        continue;

                    auto it = openMap.find(neighborKey);
                    if (it == openMap.end() || newG < it->second->G) {
                        std::shared_ptr<AStar::Node> neighbor = std::make_shared<Node>(newCoord, newTheta, current);
                        neighbor->G = newG;
                        neighbor->H = heuristicFunc(newCoord, newTheta, targetPos, targetTheta);
                        openQueue.push(neighbor);
                        openMap[neighborKey] = neighbor;
                    }
                }
            }

            if (!goalNode) {
                // std::cout << "Exact goal not reached, using closest node as fallback." << std::endl;
                goalNode = bestNode;
            }

            std::vector<std::shared_ptr<AStar::Node>> path;
            if (goalNode) {
                std::shared_ptr<AStar::Node> current = goalNode;
                while (current) {
                    path.push_back(current);
                    current = current->parent;
                }
                std::reverse(path.begin(), path.end());
            }

            return path;
        }


    private:
        //----------------------------------------------------------------------------------
        // detectCollision: Checks if a given coordinate is in collision.
        //
        // The function performs two checks:
        // 1) Grid Boundary: Ensures the coordinate is within the world boundaries.
        // 2) Clearance Check: If clearance > 0, checks surrounding cells for obstacles.
        //----------------------------------------------------------------------------------
        bool detectCollision(const Vec2i& coord) const {
            // Check if the coordinate is outside the grid boundaries.
            if (coord.x < 0 || coord.x >= worldSize.x ||
                coord.y < 0 || coord.y >= worldSize.y)
            {
                return true;
            }
            // If clearance is specified, check neighboring cells within the clearance radius.
            for (int dx = -clearance; dx <= clearance; ++dx) {
                for (int dy = -clearance; dy <= clearance; ++dy) {
                    Vec2i checkCell{coord.x + dx, coord.y + dy};
                    // Skip checking if the cell is outside grid boundaries.
                    if (checkCell.x < 0 || checkCell.x >= worldSize.x ||
                        checkCell.y < 0 || checkCell.y >= worldSize.y)
                        return true; // If the cell is out of the bounds that means there can be a collision.
                        //continue;
                    // If an obstacle exists in the surrounding cell, a collision is detected.
                    if (walls.find(checkCell) != walls.end())
                        return true;
                }
            }
            return false; // No collision detected.
        }

        //----------------------------------------------------------------------------------
        // buildCenterlineField: Two-pass chamfer distance transform seeded on the
        //   centerline cells. Afterwards centerlineDistance[i] holds the distance from
        //   cell i to the nearest centerline cell, in tenths of a grid cell (the 3-4
        //   chamfer approximates the Euclidean distance to within a few percent).
        //   Running this once per plan replaces an O(centerline) scan per expansion.
        //----------------------------------------------------------------------------------
        void buildCenterlineField() {
            const std::size_t cells =
                static_cast<std::size_t>(worldSize.x) * static_cast<std::size_t>(worldSize.y);
            centerlineDistance.assign(cells, kUnreachable);
            hasCenterline = false;

            for (const auto& point : centerline) {
                if (point.x < 0 || point.x >= worldSize.x ||
                    point.y < 0 || point.y >= worldSize.y)
                    continue;
                centerlineDistance[static_cast<std::size_t>(point.y) * worldSize.x + point.x] = 0;
                hasCenterline = true;
            }

            if (!hasCenterline) {
                // No usable centerline in this grid: charge no lateral penalty at all.
                std::fill(centerlineDistance.begin(), centerlineDistance.end(), 0);
                return;
            }

            // Forward pass: propagate from the top-left neighbours.
            for (int y = 0; y < worldSize.y; ++y) {
                for (int x = 0; x < worldSize.x; ++x) {
                    int best = centerlineDistance[static_cast<std::size_t>(y) * worldSize.x + x];
                    best = std::min(best, neighbourDistance(x - 1, y,     kNear));
                    best = std::min(best, neighbourDistance(x - 1, y - 1, kDiag));
                    best = std::min(best, neighbourDistance(x,     y - 1, kNear));
                    best = std::min(best, neighbourDistance(x + 1, y - 1, kDiag));
                    centerlineDistance[static_cast<std::size_t>(y) * worldSize.x + x] = best;
                }
            }

            // Backward pass: propagate from the bottom-right neighbours.
            for (int y = worldSize.y - 1; y >= 0; --y) {
                for (int x = worldSize.x - 1; x >= 0; --x) {
                    int best = centerlineDistance[static_cast<std::size_t>(y) * worldSize.x + x];
                    best = std::min(best, neighbourDistance(x + 1, y,     kNear));
                    best = std::min(best, neighbourDistance(x + 1, y + 1, kDiag));
                    best = std::min(best, neighbourDistance(x,     y + 1, kNear));
                    best = std::min(best, neighbourDistance(x - 1, y + 1, kDiag));
                    centerlineDistance[static_cast<std::size_t>(y) * worldSize.x + x] = best;
                }
            }
        }

        // neighbourDistance: Distance already assigned to (x, y) plus the chamfer weight,
        //                    or kUnreachable when (x, y) is outside the grid.
        int neighbourDistance(int x, int y, int weight) const {
            if (x < 0 || x >= worldSize.x || y < 0 || y >= worldSize.y)
                return kUnreachable;
            return centerlineDistance[static_cast<std::size_t>(y) * worldSize.x + x] + weight;
        }

    private:
        // Chamfer weights for the centerline distance transform, in tenths of a cell.
        static constexpr int kNear = 10;          // orthogonal neighbour
        static constexpr int kDiag = 14;          // diagonal neighbour (~sqrt(2))
        static constexpr int kUnreachable = 1 << 28;  // large, but safe to add kDiag to

        Vec2i worldSize;  // Dimensions of the grid world.
        // Set of cells that contain obstacles.
        std::unordered_set<Vec2i, Vec2iHash> walls;
        // Cost of occupying a cell, per cell of travel. 0 = fully preferred.
        std::vector<double> traversalCosts;
        // Allowed heading changes (in degrees) for each movement step.
        std::vector<double> headingChanges;
        // The selected heuristic function used for cost estimation.
        Heuristic3D heuristicFunc;
        // Clearance value: the safety distance (in cells) around obstacles.
        int clearance;
        // Factor that increases radius of curvature
        double curvature_contributing_factor;
        // Iteration counter for debugging and limiting the search.
        uint iter;
        // Cost charged per degree of heading change.
        double turn_penalty;
        // Wall-clock budget for a single search, in seconds (<= 0 disables it).
        double time_budget;
        // Centerline point
        std::vector<Vec2i> centerline = {};
        double centerline_contributing_factor;
        // Distance from every cell to the nearest centerline cell, in tenths of a cell.
        std::vector<int> centerlineDistance;
        bool hasCenterline = false;
    };

} // end namespace AStar

#endif // ASTAR_HPP
