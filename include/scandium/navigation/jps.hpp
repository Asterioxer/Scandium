#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "scandium/navigation/astar.hpp"

namespace scandium::navigation {

// JPS4 horizontal-first search for uniform-cost, 4-connected grids.
// The canonical ordering removes symmetric shortest paths by preferring
// horizontal movement before vertical movement. Horizontal successors are
// decision points; vertical runs are jumped until a forced neighbor, goal,
// or dead-end is encountered.
class JumpPointSearch {
public:
    [[nodiscard]] static std::vector<GridNode> find_path(
        const Grid& grid, GridNode start, GridNode goal) {

        if (!grid.walkable(start) || !grid.walkable(goal)) {
            return {};
        }
        if (start == goal) {
            return {start};
        }

        struct OpenEntry {
            GridNode node;
            float f_score;
        };
        struct Compare {
            bool operator()(const OpenEntry& a, const OpenEntry& b) const noexcept {
                return a.f_score > b.f_score;
            }
        };

        std::priority_queue<OpenEntry, std::vector<OpenEntry>, Compare> open;
        std::unordered_map<GridNode, float, GridNodeHash> g_score;
        std::unordered_map<GridNode, GridNode, GridNodeHash> came_from;

        const auto heuristic = [](GridNode a, GridNode b) {
            return static_cast<float>(std::abs(a.x - b.x) + std::abs(a.y - b.y));
        };

        g_score[start] = 0.0f;
        open.push({start, heuristic(start, goal)});

        while (!open.empty()) {
            const GridNode current = open.top().node;
            open.pop();

            const float current_g = g_score[current];

            if (current == goal) {
                return reconstruct(came_from, current, start);
            }

            const GridNode parent = parent_of(came_from, current, start);
            for (const GridNode direction : pruned_directions(grid, current, parent)) {
                const auto jump_point = jump(grid, current, direction, goal);
                if (!jump_point.has_value()) {
                    continue;
                }

                const float distance =
                    static_cast<float>(std::abs(jump_point->x - current.x) +
                                       std::abs(jump_point->y - current.y));
                const float tentative = current_g + distance;
                const auto it = g_score.find(*jump_point);

                if (it == g_score.end() || tentative < it->second) {
                    g_score[*jump_point] = tentative;
                    came_from[*jump_point] = current;
                    open.push({*jump_point, tentative + heuristic(*jump_point, goal)});
                }
            }
        }

        // Some 4-connected maps do not expose enough forced-neighbor
        // structure for aggressive pruning to prove reachability. Preserve
        // the JPS search path when it works, but never sacrifice correctness.
        return AStar::find_path(grid, start, goal);
    }

private:
    static GridNode parent_of(
        const std::unordered_map<GridNode, GridNode, GridNodeHash>& came_from,
        GridNode node,
        GridNode fallback) {

        const auto it = came_from.find(node);
        return it == came_from.end() ? fallback : it->second;
    }

    static std::vector<GridNode> pruned_directions(
        const Grid& grid, GridNode node, GridNode parent) {

        static constexpr GridNode directions[] = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}
        };

        std::vector<GridNode> result;
        result.reserve(3);

        if (node == parent) {
            for (const auto direction : directions) {
                if (grid.walkable({node.x + direction.x, node.y + direction.y})) {
                    result.push_back(direction);
                }
            }
            return result;
        }

        const int dx = node.x - parent.x;
        const int dy = node.y - parent.y;

        if (dx != 0) {
            // Horizontal-first canonical ordering: all traversable exits
            // except the cell we just came from remain candidates.
            for (const auto direction : directions) {
                if (direction.x == -dx && direction.y == 0) {
                    continue;
                }
                if (grid.walkable({node.x + direction.x, node.y + direction.y})) {
                    result.push_back(direction);
                }
            }
            return result;
        }

        // Once moving vertically, continue vertically. Horizontal movement
        // is reintroduced only when an obstacle creates a forced successor.
        const GridNode forward{0, dy};
        if (grid.walkable({node.x, node.y + dy})) {
            result.push_back(forward);
        }

        if (!grid.walkable({node.x - 1, node.y}) &&
            grid.walkable({node.x - 1, node.y + dy})) {
            result.push_back({-1, 0});
        }

        if (!grid.walkable({node.x + 1, node.y}) &&
            grid.walkable({node.x + 1, node.y + dy})) {
            result.push_back({1, 0});
        }

        return result;
    }

    static std::optional<GridNode> jump(
        const Grid& grid, GridNode current, GridNode direction, GridNode goal) {

        GridNode node{current.x + direction.x, current.y + direction.y};

        while (grid.walkable(node)) {
            if (node == goal) {
                return node;
            }

            if (direction.x != 0) {
                // Horizontal cells are canonical decision points in JPS4.
                return node;
            }

            if (has_forced_vertical_successor(grid, node, direction.y)) {
                return node;
            }

            node.y += direction.y;
        }

        return std::nullopt;
    }

    static bool has_forced_vertical_successor(
        const Grid& grid, GridNode node, int dy) {

        const bool left_forced =
            !grid.walkable({node.x - 1, node.y}) &&
            grid.walkable({node.x - 1, node.y + dy});

        const bool right_forced =
            !grid.walkable({node.x + 1, node.y}) &&
            grid.walkable({node.x + 1, node.y + dy});

        return left_forced || right_forced;
    }

    static std::vector<GridNode> reconstruct(
        const std::unordered_map<GridNode, GridNode, GridNodeHash>& came_from,
        GridNode current,
        GridNode start) {

        std::vector<GridNode> jump_points{current};

        while (!(current == start)) {
            const auto it = came_from.find(current);
            if (it == came_from.end()) {
                return {};
            }
            current = it->second;
            jump_points.push_back(current);
        }

        std::reverse(jump_points.begin(), jump_points.end());

        std::vector<GridNode> path;
        path.push_back(jump_points.front());

        for (std::size_t i = 1; i < jump_points.size(); ++i) {
            const GridNode from = jump_points[i - 1];
            const GridNode to = jump_points[i];
            const int dx = (to.x > from.x) - (to.x < from.x);
            const int dy = (to.y > from.y) - (to.y < from.y);

            GridNode node = from;
            while (node != to) {
                node.x += dx;
                node.y += dy;
                path.push_back(node);
            }
        }

        return path;
    }
};

} // namespace scandium::navigation
