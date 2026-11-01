#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace scandium::navigation {

enum class GridConnectivity {
    FourWay,  // Cardinal movement (N, S, E, W), cost 1.0, Manhattan heuristic
    EightWay  // Diagonal movement (N, S, E, W, NE, NW, SE, SW), cost sqrt(2), Octile heuristic
};

struct GridNode {
    int x{};
    int y{};

    friend bool operator==(const GridNode&, const GridNode&) = default;
};

struct GridNodeHash {
    std::size_t operator()(const GridNode& node) const noexcept {
        const auto x = std::hash<int>{}(node.x);
        const auto y = std::hash<int>{}(node.y);
        return x ^ (y + 0x9e3779b9U + (x << 6) + (x >> 2));
    }
};

struct SearchMetrics {
    std::size_t nodes_explored{0};
    float path_cost{0.0f};
    double execution_microseconds{0.0};
    bool path_found{false};
};

struct PathResult {
    std::vector<GridNode> path;
    SearchMetrics metrics;
};

class Grid {
public:
    Grid(int width, int height)
        : width_(width),
          height_(height),
          blocked_(static_cast<std::size_t>(width * height), false) {}

    [[nodiscard]] int width() const noexcept { return width_; }
    [[nodiscard]] int height() const noexcept { return height_; }

    [[nodiscard]] bool in_bounds(GridNode node) const noexcept {
        return node.x >= 0 && node.x < width_ && node.y >= 0 && node.y < height_;
    }

    [[nodiscard]] bool in_bounds(int x, int y) const noexcept {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    [[nodiscard]] bool walkable(GridNode node) const noexcept {
        return in_bounds(node) && !blocked_[index(node.x, node.y)];
    }

    [[nodiscard]] bool walkable(int x, int y) const noexcept {
        return in_bounds(x, y) && !blocked_[index(x, y)];
    }

    void set_blocked(GridNode node, bool blocked) {
        if (in_bounds(node)) {
            blocked_[index(node.x, node.y)] = blocked;
        }
    }

    void set_blocked(int x, int y, bool blocked) {
        if (in_bounds(x, y)) {
            blocked_[index(x, y)] = blocked;
        }
    }

    void set_rect_blocked(int start_x, int start_y, int w, int h, bool blocked) {
        for (int y = start_y; y < start_y + h; ++y) {
            for (int x = start_x; x < start_x + w; ++x) {
                set_blocked(x, y, blocked);
            }
        }
    }

    void clear_obstacles() noexcept {
        std::fill(blocked_.begin(), blocked_.end(), false);
    }

    [[nodiscard]] std::vector<GridNode> neighbors(GridNode node) const {
        static constexpr GridNode offsets[] = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}
        };

        std::vector<GridNode> result;
        result.reserve(4);
        for (const auto offset : offsets) {
            GridNode next{node.x + offset.x, node.y + offset.y};
            if (walkable(next)) {
                result.push_back(next);
            }
        }
        return result;
    }

    [[nodiscard]] std::vector<GridNode> neighbors_8way(GridNode node) const {
        static constexpr struct { int dx; int dy; } offsets[8] = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},
            {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
        };

        std::vector<GridNode> result;
        result.reserve(8);
        for (const auto [dx, dy] : offsets) {
            const int nx = node.x + dx;
            const int ny = node.y + dy;
            if (walkable(nx, ny)) {
                if (dx != 0 && dy != 0) {
                    if (!walkable(node.x + dx, node.y) || !walkable(node.x, node.y + dy)) {
                        continue; // Corner-cutting disallowed
                    }
                }
                result.push_back({nx, ny});
            }
        }
        return result;
    }

    [[nodiscard]] std::size_t index(int x, int y) const noexcept {
        return static_cast<std::size_t>(y * width_ + x);
    }

    [[nodiscard]] std::size_t index(GridNode node) const noexcept {
        return static_cast<std::size_t>(node.y * width_ + node.x);
    }

    [[nodiscard]] GridNode node_at_index(std::size_t idx) const noexcept {
        return {static_cast<int>(idx % static_cast<std::size_t>(width_)),
                static_cast<int>(idx / static_cast<std::size_t>(width_))};
    }

    [[nodiscard]] bool line_of_sight(GridNode a, GridNode b) const noexcept {
        int x0 = a.x;
        int y0 = a.y;
        const int x1 = b.x;
        const int y1 = b.y;

        const int dx = std::abs(x1 - x0);
        const int dy = -std::abs(y1 - y0);
        const int sx = x0 < x1 ? 1 : -1;
        const int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true) {
            if (!walkable(x0, y0)) {
                return false;
            }
            if (x0 == x1 && y0 == y1) {
                break;
            }
            const int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
        return true;
    }

private:
    int width_;
    int height_;
    std::vector<bool> blocked_;
};

class AStar {
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

        // Thread-local reusable node buffer with run-stamp to avoid per-search heap allocations
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
                // Octile distance (admissible and consistent for 8-connected grid)
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
        struct Step { int dx; int dy; float cost; };
        static constexpr Step steps_4way[4] = {
            {1, 0, 1.0f}, {-1, 0, 1.0f}, {0, 1, 1.0f}, {0, -1, 1.0f}
        };
        static constexpr Step steps_8way[8] = {
            {1, 0, 1.0f}, {-1, 0, 1.0f}, {0, 1, 1.0f}, {0, -1, 1.0f},
            {1, 1, 1.41421356f}, {-1, 1, 1.41421356f}, {1, -1, 1.41421356f}, {-1, -1, 1.41421356f}
        };

        const Step* steps = (connectivity == GridConnectivity::EightWay) ? steps_8way : steps_4way;
        const int step_count = (connectivity == GridConnectivity::EightWay) ? 8 : 4;

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

            const int cx = static_cast<int>(curr_idx % static_cast<std::size_t>(w));
            const int cy = static_cast<int>(curr_idx / static_cast<std::size_t>(w));
            const float current_g = curr_node.g_score;

            for (int i = 0; i < step_count; ++i) {
                const int nx = cx + steps[i].dx;
                const int ny = cy + steps[i].dy;

                if (grid.walkable(nx, ny)) {
                    // Prevent corner cutting across diagonal walls
                    if (steps[i].dx != 0 && steps[i].dy != 0) {
                        if (!grid.walkable(cx + steps[i].dx, cy) || !grid.walkable(cx, cy + steps[i].dy)) {
                            continue;
                        }
                    }

                    const std::size_t n_idx = grid.index(nx, ny);
                    auto& neighbor_node = get_node(n_idx);
                    if (neighbor_node.closed) {
                        continue;
                    }

                    const float tentative_g = current_g + steps[i].cost;
                    if (tentative_g < neighbor_node.g_score) {
                        neighbor_node.g_score = tentative_g;
                        neighbor_node.came_from = static_cast<std::int32_t>(curr_idx);
                        const float f = tentative_g + heuristic(nx, ny, goal.x, goal.y);
                        open.push({n_idx, f});
                    }
                }
            }
        }

        const auto end_time = std::chrono::steady_clock::now();
        const double duration_us =
            std::chrono::duration<double, std::micro>(end_time - start_time).count();

        if (!found) {
            return {{}, {nodes_explored, 0.0f, duration_us, false}};
        }

        // Reconstruct path
        std::vector<GridNode> path;
        std::size_t curr = goal_idx;
        while (curr != start_idx) {
            path.push_back(grid.node_at_index(curr));
            const auto prev = node_records[curr].came_from;
            if (prev < 0) {
                return {{}, {nodes_explored, 0.0f, duration_us, false}};
            }
            curr = static_cast<std::size_t>(prev);
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());

        const float cost = node_records[goal_idx].g_score;
        return {std::move(path), {nodes_explored, cost, duration_us, true}};
    }

    [[nodiscard]] static std::vector<GridNode> find_path(
        const Grid& grid, GridNode start, GridNode goal,
        GridConnectivity connectivity = GridConnectivity::FourWay) {
        return find_path_with_metrics(grid, start, goal, connectivity).path;
    }

    [[nodiscard]] static std::vector<GridNode> smooth_path(
        const Grid& grid, const std::vector<GridNode>& path) {
        if (path.size() <= 2) {
            return path;
        }

        std::vector<GridNode> smoothed;
        smoothed.push_back(path.front());
        std::size_t current_idx = 0;

        while (current_idx < path.size() - 1) {
            std::size_t furthest = current_idx + 1;
            for (std::size_t candidate = current_idx + 2; candidate < path.size(); ++candidate) {
                if (grid.line_of_sight(path[current_idx], path[candidate])) {
                    furthest = candidate;
                }
            }
            smoothed.push_back(path[furthest]);
            current_idx = furthest;
        }

        return smoothed;
    }
};

} // namespace scandium::navigation
