#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "scandium/navigation/astar.hpp"

namespace scandium::navigation {

// Jump Point Search for the project's 4-connected uniform-cost grid.
// JPS prunes symmetric straight-line paths and expands only jump points.
// Returned paths are expanded back into individual grid cells so callers
// retain the same contract as A*.
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

            if (current == goal) {
                return reconstruct(came_from, current, start);
            }

            const float current_g = g_score[current];

            for (const auto direction : directions_from(grid, current,
                                                        parent_of(came_from, current, start))) {
                const auto jump = jump(grid, current, direction, goal);
                if (!jump.has_value()) {
                    continue;
                }

                const float distance =
                    static_cast<float>(std::abs(jump->x - current.x) +
                                       std::abs(jump->y - current.y));
                const float tentative = current_g + distance;
                const auto it = g_score.find(*jump);

                if (it == g_score.end() || tentative < it->second) {
                    g_score[*jump] = tentative;
                    came_from[*jump] = current;
                    open.push({*jump, tentative + heuristic(*jump, goal)});
                }
            }
        }

        return {};
    }

private:
    static constexpr GridNode kDirections[] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };

    static GridNode parent_of(
        const std::unordered_map<GridNode, GridNode, GridNodeHash>& came_from,
        GridNode node,
        GridNode fallback) {

        const auto it = came_from.find(node);
        return it == came_from.end() ? fallback : it->second;
    }

    static std::vector<GridNode> directions_from(
        const Grid& grid, GridNode node, GridNode parent) {

        std::vector<GridNode> result;
        result.reserve(4);

        const int dx = node.x - parent.x;
        const int dy = node.y - parent.y;

        if (node == parent) {
            for (const auto direction : kDirections) {
                if (grid.walkable({node.x + direction.x, node.y + direction.y})) {
                    result.push_back(direction);
                }
            }
            return result;
        }

        // Natural direction.
        const GridNode forward{dx, dy};
        if (grid.walkable({node.x + dx, node.y + dy})) {
            result.push_back(forward);
        }

        // Forced neighbors for a 4-connected grid.
        if (dx != 0) {
            if (!grid.walkable({node.x, node.y + 1}) &&
                grid.walkable({node.x - dx, node.y + 1})) {
                result.push_back({0, 1});
            }
            if (!grid.walkable({node.x, node.y - 1}) &&
                grid.walkable({node.x - dx, node.y - 1})) {
                result.push_back({0, -1});
            }
        } else {
            if (!grid.walkable({node.x + 1, node.y}) &&
                grid.walkable({node.x + 1, node.y - dy})) {
                result.push_back({1, 0});
            }
            if (!grid.walkable({node.x - 1, node.y}) &&
                grid.walkable({node.x - 1, node.y - dy})) {
                result.push_back({-1, 0});
            }
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

            const int dx = direction.x;
            const int dy = direction.y;

            // A straight cardinal jump point has a forced neighbor.
            if (dx != 0) {
                if ((!grid.walkable({node.x, node.y + 1}) &&
                     grid.walkable({node.x - dx, node.y + 1})) ||
                    (!grid.walkable({node.x, node.y - 1}) &&
                     grid.walkable({node.x - dx, node.y - 1}))) {
                    return node;
                }
            } else {
                if ((!grid.walkable({node.x + 1, node.y}) &&
                     grid.walkable({node.x + 1, node.y - dy})) ||
                    (!grid.walkable({node.x - 1, node.y}) &&
                     grid.walkable({node.x - 1, node.y - dy}))) {
                    return node;
                }
            }

            // A perpendicular jump point makes this node a useful turning point.
            if (dx != 0) {
                if (has_jump_ahead(grid, node, {0, 1}, goal) ||
                    has_jump_ahead(grid, node, {0, -1}, goal)) {
                    return node;
                }
            } else {
                if (has_jump_ahead(grid, node, {1, 0}, goal) ||
                    has_jump_ahead(grid, node, {-1, 0}, goal)) {
                    return node;
                }
            }

            node.x += dx;
            node.y += dy;
        }

        return std::nullopt;
    }

    static bool has_jump_ahead(
        const Grid& grid, GridNode start, GridNode direction, GridNode goal) {

        GridNode node{start.x + direction.x, start.y + direction.y};
        while (grid.walkable(node)) {
            if (node == goal) {
                return true;
            }

            const int dx = direction.x;
            const int dy = direction.y;
            if (dx != 0) {
                if ((!grid.walkable({node.x, node.y + 1}) &&
                     grid.walkable({node.x - dx, node.y + 1})) ||
                    (!grid.walkable({node.x, node.y - 1}) &&
                     grid.walkable({node.x - dx, node.y - 1}))) {
                    return true;
                }
            } else {
                if ((!grid.walkable({node.x + 1, node.y}) &&
                     grid.walkable({node.x + 1, node.y - dy})) ||
                    (!grid.walkable({node.x - 1, node.y}) &&
                     grid.walkable({node.x - 1, node.y - dy}))) {
                    return true;
                }
            }

            node.x += dx;
            node.y += dy;
        }
        return false;
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
