#include "PathPlanning.h"

#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

AStarPlanner::AStarPlanner(int w, int h, const std::vector<std::vector<int>>& g)
    : width(w), height(h), grid(g) {
    if (width <= 0 || height <= 0 || grid.size() != static_cast<std::size_t>(width)) {
        throw std::invalid_argument("planner dimensions must match a nonempty grid");
    }
    for (const auto& column : grid) {
        if (column.size() != static_cast<std::size_t>(height)) {
            throw std::invalid_argument("planner grid must be rectangular");
        }
    }
}

bool AStarPlanner::inBounds(int x, int y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) const {
    return std::abs(static_cast<double>(x1) - x2)
         + std::abs(static_cast<double>(y1) - y2);
}

bool AStarPlanner::isBlocked(int x, int y) const {
    return !inBounds(x, y) || grid[x][y] != 0;
}

std::vector<Node> AStarPlanner::plan(int startX, int startY, int goalX, int goalY) {
    if (isBlocked(startX, startY) || isBlocked(goalX, goalY)) {
        return {};
    }

    // Coordinate tie-breaks make equally short routes reproducible.
    using Entry = std::tuple<double, double, int, int>;  // f, g, x, y
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> distance(
        width, std::vector<double>(height, infinity));
    std::vector<std::vector<std::pair<int, int>>> parent(
        width, std::vector<std::pair<int, int>>(height, {-1, -1}));

    distance[startX][startY] = 0.0;
    open.emplace(heuristic(startX, startY, goalX, goalY), 0.0, startX, startY);
    constexpr int neighbors[][2] = {{-1, 0}, {0, -1}, {0, 1}, {1, 0}};

    while (!open.empty()) {
        const auto [estimate, cost, x, y] = open.top();
        (void)estimate;
        open.pop();
        if (cost != distance[x][y]) {
            continue;  // A shorter route superseded this queued entry.
        }
        if (x == goalX && y == goalY) {
            std::vector<Node> path;
            int px = goalX;
            int py = goalY;
            while (true) {
                path.emplace_back(px, py, distance[px][py]);
                if (px == startX && py == startY) {
                    return path;
                }
                const auto predecessor = parent[px][py];
                px = predecessor.first;
                py = predecessor.second;
            }
        }

        for (const auto& offset : neighbors) {
            const int nx = x + offset[0];
            const int ny = y + offset[1];
            if (isBlocked(nx, ny)) {
                continue;
            }
            const double nextCost = cost + 1.0;
            if (nextCost < distance[nx][ny]) {
                distance[nx][ny] = nextCost;
                parent[nx][ny] = {x, y};
                open.emplace(nextCost + heuristic(nx, ny, goalX, goalY),
                             nextCost, nx, ny);
            }
        }
    }
    return {};
}

DynamicAStarPlanner::DynamicAStarPlanner(
    int w, int h, const std::vector<std::vector<int>>& g)
    : AStarPlanner(w, h, g) {}

void DynamicAStarPlanner::addDynamicObstacle(int x, int y) {
    if (!inBounds(x, y)) {
        throw std::out_of_range("dynamic obstacle is outside the grid");
    }
    dynamicObstacles.emplace(x, y);
}

void DynamicAStarPlanner::removeDynamicObstacle(int x, int y) {
    if (!inBounds(x, y)) {
        throw std::out_of_range("dynamic obstacle is outside the grid");
    }
    dynamicObstacles.erase({x, y});
}

void DynamicAStarPlanner::moveObstacle(int oldX, int oldY, int newX, int newY) {
    if (!inBounds(oldX, oldY) || !inBounds(newX, newY)) {
        throw std::out_of_range("dynamic obstacle is outside the grid");
    }
    dynamicObstacles.erase({oldX, oldY});
    dynamicObstacles.emplace(newX, newY);
}

bool DynamicAStarPlanner::isBlocked(int x, int y) const {
    return AStarPlanner::isBlocked(x, y)
        || dynamicObstacles.count({x, y}) != 0;
}
