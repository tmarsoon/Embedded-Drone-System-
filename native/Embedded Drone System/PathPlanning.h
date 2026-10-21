#ifndef EMBEDDED_DRONE_PATH_PLANNING_H
#define EMBEDDED_DRONE_PATH_PLANNING_H

#include <set>
#include <utility>
#include <vector>

struct Node {
    int x;
    int y;
    double cost;
    // Compatibility only: returned paths own their nodes; parents stay null.
    Node* parent;
    Node(int x, int y, double cost, Node* parent = nullptr)
        : x(x), y(y), cost(cost), parent(parent) {}
    bool operator>(const Node& other) const { return cost > other.cost; }
};

class AStarPlanner {
public:
    // grid[x][y] uses zero for traversable cells. Invalid dimensions throw
    // std::invalid_argument.
    AStarPlanner(int width, int height, const std::vector<std::vector<int>>& grid);
    virtual ~AStarPlanner() = default;
    // Four-neighbor, unit-cost shortest path in goal-to-start order. Invalid or
    // blocked endpoints and unreachable goals return empty. Cost is distance
    // from start. No returned node contains an owning or dangling pointer.
    virtual std::vector<Node> plan(int startX, int startY, int goalX, int goalY);

protected:
    int width;
    int height;
    std::vector<std::vector<int>> grid;
    bool inBounds(int x, int y) const;
    double heuristic(int x1, int y1, int x2, int y2) const;
    virtual bool isBlocked(int x, int y) const;
};

struct DynamicObstacle {
    int x;
    int y;
    DynamicObstacle(int x, int y) : x(x), y(y) {}
    void move(int dx, int dy) { x += dx; y += dy; }
};

class DynamicAStarPlanner : public AStarPlanner {
public:
    DynamicAStarPlanner(int width, int height,
                        const std::vector<std::vector<int>>& grid);
    // Invalid coordinates throw std::out_of_range before changing obstacles.
    void addDynamicObstacle(int x, int y);
    void removeDynamicObstacle(int x, int y);
    void moveObstacle(int oldX, int oldY, int newX, int newY);

protected:
    bool isBlocked(int x, int y) const override;

private:
    std::set<std::pair<int, int>> dynamicObstacles;
};

#endif
