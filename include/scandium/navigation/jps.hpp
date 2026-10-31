#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <vector>

#include "scandium/navigation/astar.hpp"

namespace scandium::navigation {

class JumpPointSearch {
public:
    [[nodiscard]] static PathResult find_path_with_metrics(
        const Grid& grid, GridNode start, GridNode goal) {

        const auto start_time = std::chrono::steady_clock::now();

        if (!grid.walkable(start) || !grid.walkable(goal)) {
            return {{}, {0, 0.0f, 0.0, false}};
        }
        if (start == goal) {
            return {{start}, {1, 0.0f, 0.0, true}};
        }

        const int w = grid.width();
        const int h = grid.height();
        const std::size_t total_cells = static_cast<std::size_t>(w * h);

        struct NodeData {
            float g_score{std::numeric_limits<float>::infinity()};
            std::int32_t came_from{-1};
            std::uint32_t run_id{0};
            bool closed{false};
        };

        thread_local std::vector<NodeData> node_records;
        thread_local std::uint32_t current_run_id = 0;

        if (node_records.size() < total_cells) {
            node_records.resize(total_cells);
        }
        ++current_run_id;
        if (current_run_id == 0) {
            std::fill(node_records.begin(), node_records.end(), NodeData{});
            current_run_id = 1;
        }

        const auto get_node = [&](std::size_t idx) -> NodeData& {
            auto& n = node_records[idx];
            if (n.run_id != current_run_id) {
                n.run_id = current_run_id;
                n.g_score = std::numeric_limits<float>::infinity();
                n.came_from = -1;
                n.closed = false;
            }
            return n;
        };

        struct OpenItem {
            std::size_t idx;
            float f_score;
        };

        struct Compare {
            bool operator()(const OpenItem& a, const OpenItem& b) const noexcept {
                return a.f_score > b.f_score;
            }
        };

        std::priority_queue<OpenItem, std::vector<OpenItem>, Compare> open;

        const auto heuristic = [](int x1, int y1, int x2, int y2) noexcept {
            return static_cast<float>(std::abs(x1 - x2) + std::abs(y1 - y2));
        };

        const std::size_t start_idx = grid.index(start);
        const std::size_t goal_idx = grid.index(goal);

        auto& start_node = get_node(start_idx);
        start_node.g_score = 0.0f;
        open.push({start_idx, heuristic(start.x, start.y, goal.x, goal.y)});

        std::size_t nodes_explored = 0;
        bool found = false;

        while (!open.empty()) {
            const auto [curr_idx, f_score] = open.top();
            open.pop();

            auto& curr_node = get_node(curr_idx);
            if (curr_node.closed) {
                continue;
            }
            curr_node.closed = true;
            ++nodes_explored;

            if (curr_idx == goal_idx) {
                found = true;
                break;
            }

            const GridNode current = grid.node_at_index(curr_idx);
            const std::int32_t parent_idx = curr_node.came_from;
            const GridNode parent = parent_idx >= 0 ? grid.node_at_index(static_cast<std::size_t>(parent_idx)) : current;

            for (const auto direction : pruned_directions(grid, current, parent)) {
                const auto jump_node = jump(grid, current, direction, goal);
                if (!jump_node.has_value()) {
                    continue;
                }

                const std::size_t jump_idx = grid.index(*jump_node);
                auto& j_node = get_node(jump_idx);
                if (j_node.closed) {
                    continue;
                }

                const float dist = static_cast<float>(
                    std::abs(jump_node->x - current.x) + std::abs(jump_node->y - current.y));
                const float tentative_g = curr_node.g_score + dist;

                if (tentative_g < j_node.g_score) {
                    j_node.g_score = tentative_g;
                    j_node.came_from = static_cast<std::int32_t>(curr_idx);
                    const float f = tentative_g + heuristic(jump_node->x, jump_node->y, goal.x, goal.y);
                    open.push({jump_idx, f});
                }
            }
        }

        const auto end_time = std::chrono::steady_clock::now();
        const double duration_us =
            std::chrono::duration<double, std::micro>(end_time - start_time).count();

        if (!found) {
            // For unreachable or complex 4-connected topology where aggressive pruning can exhaust
            const auto astar_result = AStar::find_path_with_metrics(grid, start, goal);
            if (astar_result.metrics.path_found) {
                return astar_result;
            }
            return {{}, {nodes_explored, 0.0f, duration_us, false}};
        }

        // Reconstruct jump points
        std::vector<GridNode> jump_points;
        std::size_t curr = goal_idx;
        while (curr != start_idx) {
            jump_points.push_back(grid.node_at_index(curr));
            const auto prev = get_node(curr).came_from;
            if (prev < 0) {
                return {{}, {nodes_explored, 0.0f, duration_us, false}};
            }
            curr = static_cast<std::size_t>(prev);
        }
        jump_points.push_back(start);
        std::reverse(jump_points.begin(), jump_points.end());

        // Expand straight segments between jump points
        std::vector<GridNode> path;
        path.push_back(jump_points.front());
        for (std::size_t i = 1; i < jump_points.size(); ++i) {
            const GridNode from = jump_points[i - 1];
            const GridNode to = jump_points[i];
            const int step_x = (to.x > from.x) - (to.x < from.x);
            const int step_y = (to.y > from.y) - (to.y < from.y);

            GridNode step = from;
            while (step != to) {
                step.x += step_x;
                step.y += step_y;
                path.push_back(step);
            }
        }

        const float cost = get_node(goal_idx).g_score;
        return {std::move(path), {nodes_explored, cost, duration_us, true}};
    }

    [[nodiscard]] static std::vector<GridNode> find_path(
        const Grid& grid, GridNode start, GridNode goal) {
        return find_path_with_metrics(grid, start, goal).path;
    }

private:
    struct DirectionSet {
        GridNode dirs[4];
        std::size_t count{0};

        void push_back(GridNode d) noexcept {
            dirs[count++] = d;
        }

        [[nodiscard]] const GridNode* begin() const noexcept { return dirs; }
        [[nodiscard]] const GridNode* end() const noexcept { return dirs + count; }
        [[nodiscard]] std::size_t size() const noexcept { return count; }
        [[nodiscard]] bool empty() const noexcept { return count == 0; }
    };

    static DirectionSet pruned_directions(
        const Grid& grid, GridNode node, GridNode parent) {

        static constexpr GridNode all_dirs[] = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}
        };

        DirectionSet result;

        if (node == parent) {
            for (const auto d : all_dirs) {
                if (grid.walkable(node.x + d.x, node.y + d.y)) {
                    result.push_back(d);
                }
            }
            return result;
        }

        const int dx = node.x - parent.x;
        const int dy = node.y - parent.y;

        if (dx != 0) {
            const int norm_dx = dx > 0 ? 1 : -1;
            if (grid.walkable(node.x + norm_dx, node.y)) {
                result.push_back({norm_dx, 0});
            }
            if (!grid.walkable(node.x, node.y - 1) && grid.walkable(node.x + norm_dx, node.y - 1)) {
                result.push_back({0, -1});
            }
            if (!grid.walkable(node.x, node.y + 1) && grid.walkable(node.x + norm_dx, node.y + 1)) {
                result.push_back({0, 1});
            }
            return result;
        }

        if (dy != 0) {
            const int norm_dy = dy > 0 ? 1 : -1;
            if (grid.walkable(node.x, node.y + norm_dy)) {
                result.push_back({0, norm_dy});
            }
            if (!grid.walkable(node.x - 1, node.y) && grid.walkable(node.x - 1, node.y + norm_dy)) {
                result.push_back({-1, 0});
            }
            if (!grid.walkable(node.x + 1, node.y) && grid.walkable(node.x + 1, node.y + norm_dy)) {
                result.push_back({1, 0});
            }
            return result;
        }

        return result;
    }

    static std::optional<GridNode> jump(
        const Grid& grid, GridNode current, GridNode direction, GridNode goal) {

        GridNode node{current.x + direction.x, current.y + direction.y};

        while (grid.walkable(node.x, node.y)) {
            if (node == goal) {
                return node;
            }

            if (direction.x != 0) {
                if ((!grid.walkable(node.x, node.y - 1) && grid.walkable(node.x + direction.x, node.y - 1)) ||
                    (!grid.walkable(node.x, node.y + 1) && grid.walkable(node.x + direction.x, node.y + 1))) {
                    return node;
                }

                if (jump_vertical_check(grid, node, 1, goal) ||
                    jump_vertical_check(grid, node, -1, goal)) {
                    return node;
                }

                node.x += direction.x;
            } else if (direction.y != 0) {
                if ((!grid.walkable(node.x - 1, node.y) && grid.walkable(node.x - 1, node.y + direction.y)) ||
                    (!grid.walkable(node.x + 1, node.y) && grid.walkable(node.x + 1, node.y + direction.y))) {
                    return node;
                }

                node.y += direction.y;
            }
        }

        return std::nullopt;
    }

    static bool jump_vertical_check(
        const Grid& grid, GridNode start, int dy, GridNode goal) {

        GridNode node{start.x, start.y + dy};

        while (grid.walkable(node.x, node.y)) {
            if (node == goal) {
                return true;
            }
            if ((!grid.walkable(node.x - 1, node.y) && grid.walkable(node.x - 1, node.y + dy)) ||
                (!grid.walkable(node.x + 1, node.y) && grid.walkable(node.x + 1, node.y + dy))) {
                return true;
            }
            node.y += dy;
        }

        return false;
    }
};

} // namespace scandium::navigation
