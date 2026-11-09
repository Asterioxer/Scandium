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
        const Grid& grid, GridNode start, GridNode goal,
        GridConnectivity connectivity = GridConnectivity::FourWay) {

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

        const auto heuristic = [connectivity](int x1, int y1, int x2, int y2) noexcept -> float {
            const int dx = std::abs(x1 - x2);
            const int dy = std::abs(y1 - y2);
            if (connectivity == GridConnectivity::EightWay) {
                const float min_d = static_cast<float>(std::min(dx, dy));
                const float max_d = static_cast<float>(std::max(dx, dy));
                return 0.41421356f * min_d + max_d;
            }
            return static_cast<float>(dx + dy);
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

            if (connectivity == GridConnectivity::FourWay) {
                explore_neighbors_4way(grid, current, parent, goal, get_node, open, heuristic, curr_idx, curr_node.g_score);
            } else {
                explore_neighbors_8way(grid, current, parent, goal, get_node, open, heuristic, curr_idx, curr_node.g_score);
            }
        }

        const auto end_time = std::chrono::steady_clock::now();
        const double duration_us =
            std::chrono::duration<double, std::micro>(end_time - start_time).count();

        if (!found) {
            return {{}, {nodes_explored, 0.0f, duration_us, false}};
        }

        // Reconstruct path through jump points
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

        // Interpolate straight / diagonal segments between jump points
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
        const Grid& grid, GridNode start, GridNode goal,
        GridConnectivity connectivity = GridConnectivity::FourWay) {
        return find_path_with_metrics(grid, start, goal, connectivity).path;
    }

private:
    struct DirList {
        GridNode dirs[8];
        std::size_t count{0};
        void push(GridNode d) noexcept { dirs[count++] = d; }
        const GridNode* begin() const noexcept { return dirs; }
        const GridNode* end() const noexcept { return dirs + count; }
    };

    template <typename GetNodeFn, typename OpenQueue, typename HeuristicFn>
    static void explore_neighbors_4way(
        const Grid& grid, GridNode current, GridNode parent, GridNode goal,
        GetNodeFn& get_node, OpenQueue& open, HeuristicFn& heuristic,
        std::size_t curr_idx, float curr_g) {

        DirList dirs;
        if (current == parent) {
            static constexpr GridNode cardinals[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
            for (const auto d : cardinals) {
                if (grid.walkable(current.x + d.x, current.y + d.y)) {
                    dirs.push(d);
                }
            }
        } else {
            const int dx = (current.x > parent.x) - (current.x < parent.x);
            const int dy = (current.y > parent.y) - (current.y < parent.y);

            if (dx != 0) {
                if (grid.walkable(current.x + dx, current.y)) {
                    dirs.push({dx, 0});
                }
                if (grid.walkable(current.x, current.y + 1)) {
                    dirs.push({0, 1});
                }
                if (grid.walkable(current.x, current.y - 1)) {
                    dirs.push({0, -1});
                }
            } else if (dy != 0) {
                if (grid.walkable(current.x, current.y + dy)) {
                    dirs.push({0, dy});
                }
                if (grid.walkable(current.x + 1, current.y)) {
                    dirs.push({1, 0});
                }
                if (grid.walkable(current.x - 1, current.y)) {
                    dirs.push({-1, 0});
                }
            }
        }

        for (const auto d : dirs) {
            const auto jump_node = jump_4way(grid, current, d, goal);
            if (!jump_node.has_value()) {
                continue;
            }

            const std::size_t jump_idx = grid.index(*jump_node);
            auto& j_node = get_node(jump_idx);
            if (j_node.closed) {
                continue;
            }

            const float dist = static_cast<float>(std::abs(jump_node->x - current.x) + std::abs(jump_node->y - current.y));
            const float tentative_g = curr_g + dist;

            if (tentative_g < j_node.g_score) {
                j_node.g_score = tentative_g;
                j_node.came_from = static_cast<std::int32_t>(curr_idx);
                const float f = tentative_g + heuristic(jump_node->x, jump_node->y, goal.x, goal.y);
                open.push({jump_idx, f});
            }
        }
    }

    static std::optional<GridNode> jump_4way(
        const Grid& grid, GridNode current, GridNode dir, GridNode goal) {

        GridNode node{current.x + dir.x, current.y + dir.y};

        while (grid.walkable(node.x, node.y)) {
            if (node == goal) {
                return node;
            }

            if (dir.x != 0) {
                if ((!grid.walkable(node.x - dir.x, node.y - 1) && grid.walkable(node.x, node.y - 1)) ||
                    (!grid.walkable(node.x - dir.x, node.y + 1) && grid.walkable(node.x, node.y + 1))) {
                    return node;
                }

                if (jump_4way_orthogonal(grid, node, {0, 1}, goal) ||
                    jump_4way_orthogonal(grid, node, {0, -1}, goal)) {
                    return node;
                }

                node.x += dir.x;
            } else {
                if ((!grid.walkable(node.x - 1, node.y - dir.y) && grid.walkable(node.x - 1, node.y)) ||
                    (!grid.walkable(node.x + 1, node.y - dir.y) && grid.walkable(node.x + 1, node.y))) {
                    return node;
                }

                if (jump_4way_orthogonal(grid, node, {1, 0}, goal) ||
                    jump_4way_orthogonal(grid, node, {-1, 0}, goal)) {
                    return node;
                }

                node.y += dir.y;
            }
        }

        return std::nullopt;
    }

    static bool jump_4way_orthogonal(
        const Grid& grid, GridNode start, GridNode dir, GridNode goal) {

        GridNode node{start.x + dir.x, start.y + dir.y};

        while (grid.walkable(node.x, node.y)) {
            if (node == goal) {
                return true;
            }

            if (dir.x != 0) {
                if ((!grid.walkable(node.x - dir.x, node.y - 1) && grid.walkable(node.x, node.y - 1)) ||
                    (!grid.walkable(node.x - dir.x, node.y + 1) && grid.walkable(node.x, node.y + 1))) {
                    return true;
                }
                node.x += dir.x;
            } else {
                if ((!grid.walkable(node.x - 1, node.y - dir.y) && grid.walkable(node.x - 1, node.y)) ||
                    (!grid.walkable(node.x + 1, node.y - dir.y) && grid.walkable(node.x + 1, node.y))) {
                    return true;
                }
                node.y += dir.y;
            }
        }

        return false;
    }

    template <typename GetNodeFn, typename OpenQueue, typename HeuristicFn>
    static void explore_neighbors_8way(
        const Grid& grid, GridNode current, GridNode parent, GridNode goal,
        GetNodeFn& get_node, OpenQueue& open, HeuristicFn& heuristic,
        std::size_t curr_idx, float curr_g) {

        DirList dirs;
        if (current == parent) {
            static constexpr GridNode all_8[] = {
                {1, 0}, {-1, 0}, {0, 1}, {0, -1},
                {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
            };
            for (const auto d : all_8) {
                if (grid.walkable(current.x + d.x, current.y + d.y)) {
                    if (d.x != 0 && d.y != 0) {
                        if (!grid.walkable(current.x + d.x, current.y) || !grid.walkable(current.x, current.y + d.y)) {
                            continue;
                        }
                    }
                    dirs.push(d);
                }
            }
        } else {
            const int dx = (current.x > parent.x) - (current.x < parent.x);
            const int dy = (current.y > parent.y) - (current.y < parent.y);

            if (dx != 0 && dy != 0) {
                if (grid.walkable(current.x, current.y + dy)) dirs.push({0, dy});
                if (grid.walkable(current.x + dx, current.y)) dirs.push({dx, 0});
                if (grid.walkable(current.x + dx, current.y + dy)) {
                    if (grid.walkable(current.x + dx, current.y) && grid.walkable(current.x, current.y + dy)) {
                        dirs.push({dx, dy});
                    }
                }
                if (!grid.walkable(current.x - dx, current.y) && grid.walkable(current.x - dx, current.y + dy)) {
                    if (grid.walkable(current.x, current.y + dy)) dirs.push({-dx, dy});
                }
                if (!grid.walkable(current.x, current.y - dy) && grid.walkable(current.x + dx, current.y - dy)) {
                    if (grid.walkable(current.x + dx, current.y)) dirs.push({dx, -dy});
                }
            } else if (dx != 0) {
                if (grid.walkable(current.x + dx, current.y)) dirs.push({dx, 0});
                if (!grid.walkable(current.x, current.y - 1) && grid.walkable(current.x + dx, current.y - 1)) {
                    if (grid.walkable(current.x + dx, current.y)) dirs.push({dx, -1});
                }
                if (!grid.walkable(current.x, current.y + 1) && grid.walkable(current.x + dx, current.y + 1)) {
                    if (grid.walkable(current.x + dx, current.y)) dirs.push({dx, 1});
                }
            } else {
                if (grid.walkable(current.x, current.y + dy)) dirs.push({0, dy});
                if (!grid.walkable(current.x - 1, current.y) && grid.walkable(current.x - 1, current.y + dy)) {
                    if (grid.walkable(current.x, current.y + dy)) dirs.push({-1, dy});
                }
                if (!grid.walkable(current.x + 1, current.y) && grid.walkable(current.x + 1, current.y + dy)) {
                    if (grid.walkable(current.x, current.y + dy)) dirs.push({1, dy});
                }
            }
        }

        for (const auto d : dirs) {
            const auto jump_node = jump_8way(grid, current, d, goal);
            if (!jump_node.has_value()) {
                continue;
            }

            const std::size_t jump_idx = grid.index(*jump_node);
            auto& j_node = get_node(jump_idx);
            if (j_node.closed) {
                continue;
            }

            const int step_dx = std::abs(jump_node->x - current.x);
            const int step_dy = std::abs(jump_node->y - current.y);
            float dist = 0.0f;
            if (step_dx != 0 && step_dy != 0) {
                dist = static_cast<float>(step_dx) * 1.41421356f;
            } else {
                dist = static_cast<float>(step_dx + step_dy);
            }

            const float tentative_g = curr_g + dist;
            if (tentative_g < j_node.g_score) {
                j_node.g_score = tentative_g;
                j_node.came_from = static_cast<std::int32_t>(curr_idx);
                const float f = tentative_g + heuristic(jump_node->x, jump_node->y, goal.x, goal.y);
                open.push({jump_idx, f});
            }
        }
    }

    static std::optional<GridNode> jump_8way(
        const Grid& grid, GridNode current, GridNode dir, GridNode goal) {

        const GridNode node{current.x + dir.x, current.y + dir.y};
        if (!grid.walkable(node.x, node.y)) {
            return std::nullopt;
        }

        if (dir.x != 0 && dir.y != 0) {
            if (!grid.walkable(current.x + dir.x, current.y) || !grid.walkable(current.x, current.y + dir.y)) {
                return std::nullopt; // Corner cutting disallowed
            }
        }

        if (node == goal) {
            return node;
        }

        if (dir.x != 0 && dir.y != 0) {
            if ((!grid.walkable(node.x - dir.x, node.y) && grid.walkable(node.x - dir.x, node.y + dir.y)) ||
                (!grid.walkable(node.x, node.y - dir.y) && grid.walkable(node.x + dir.x, node.y - dir.y))) {
                return node;
            }
            if (jump_8way(grid, node, {dir.x, 0}, goal).has_value() ||
                jump_8way(grid, node, {0, dir.y}, goal).has_value()) {
                return node;
            }
        } else if (dir.x != 0) {
            if ((!grid.walkable(node.x, node.y - 1) && grid.walkable(node.x + dir.x, node.y - 1)) ||
                (!grid.walkable(node.x, node.y + 1) && grid.walkable(node.x + dir.x, node.y + 1))) {
                return node;
            }
        } else {
            if ((!grid.walkable(node.x - 1, node.y) && grid.walkable(node.x - 1, node.y + dir.y)) ||
                (!grid.walkable(node.x + 1, node.y) && grid.walkable(node.x + 1, node.y + dir.y))) {
                return node;
            }
        }

        return jump_8way(grid, node, dir, goal);
    }
};

} // namespace scandium::navigation
