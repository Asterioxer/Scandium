#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace scandium::navigation {

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

class Grid {
public:
    Grid(int width, int height) : width_(width), height_(height),
        blocked_(static_cast<std::size_t>(width * height), false) {}

    [[nodiscard]] bool in_bounds(GridNode node) const noexcept {
        return node.x >= 0 && node.x < width_ && node.y >= 0 && node.y < height_;
    }

    [[nodiscard]] bool walkable(GridNode node) const noexcept {
        return in_bounds(node) && !blocked_[index(node)];
    }

    void set_blocked(GridNode node, bool blocked) {
        if (in_bounds(node)) {
            blocked_[index(node)] = blocked;
        }
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

private:
    int width_;
    int height_;
    std::vector<bool> blocked_;

    [[nodiscard]] std::size_t index(GridNode node) const noexcept {
        return static_cast<std::size_t>(node.y * width_ + node.x);
    }
};

class AStar {
public:
    [[nodiscard]] static std::vector<GridNode> find_path(
        const Grid& grid, GridNode start, GridNode goal) {

        if (!grid.walkable(start) || !grid.walkable(goal)) {
            return {};
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
            const auto current = open.top().node;
            open.pop();

            if (current == goal) {
                return reconstruct(came_from, current, start);
            }

            const float current_g = g_score[current];

            for (const auto neighbor : grid.neighbors(current)) {
                const float tentative = current_g + 1.0f;
                const auto it = g_score.find(neighbor);

                if (it == g_score.end() || tentative < it->second) {
                    g_score[neighbor] = tentative;
                    came_from[neighbor] = current;
                    open.push({neighbor, tentative + heuristic(neighbor, goal)});
                }
            }
        }

        return {};
    }

private:
    static std::vector<GridNode> reconstruct(
        const std::unordered_map<GridNode, GridNode, GridNodeHash>& came_from,
        GridNode current,
        GridNode start) {

        std::vector<GridNode> path{current};

        while (!(current == start)) {
            const auto it = came_from.find(current);
            if (it == came_from.end()) {
                return {};
            }
            current = it->second;
            path.push_back(current);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
};

} // namespace scandium::navigation
