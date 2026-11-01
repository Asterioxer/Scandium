#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#include "scandium/math/vec3.hpp"
#include "scandium/navigation/astar.hpp"

namespace scandium::navigation {

/**
 * 2D Flow Field (Vector Field) Crowd Navigation.
 * Precomputes an integration potential field from goal cells and generates
 * continuous downhill steering vectors for unlimited concurrent agents in O(1) time.
 */
class FlowField {
public:
    static constexpr float UNREACHABLE = std::numeric_limits<float>::infinity();

    FlowField(int width, int height, float cell_size = 1.0f, math::Vec3f origin = {})
        : width_(width),
          height_(height),
          cell_size_(cell_size),
          origin_(origin),
          cost_field_(static_cast<std::size_t>(width * height), 1.0f),
          integration_field_(static_cast<std::size_t>(width * height), UNREACHABLE),
          flow_field_(static_cast<std::size_t>(width * height), math::Vec3f{0.0f, 0.0f, 0.0f}) {}

    [[nodiscard]] int width() const noexcept { return width_; }
    [[nodiscard]] int height() const noexcept { return height_; }
    [[nodiscard]] float cell_size() const noexcept { return cell_size_; }

    void set_blocked(int x, int y, bool blocked) {
        if (in_bounds(x, y)) {
            cost_field_[to_index(x, y)] = blocked ? 255.0f : 1.0f;
        }
    }

    void set_cost(int x, int y, float cost) {
        if (in_bounds(x, y)) {
            cost_field_[to_index(x, y)] = std::max(1.0f, cost);
        }
    }

    [[nodiscard]] float get_cost(int x, int y) const noexcept {
        if (!in_bounds(x, y)) return 255.0f;
        return cost_field_[to_index(x, y)];
    }

    [[nodiscard]] bool is_blocked(int x, int y) const noexcept {
        return get_cost(x, y) >= 254.0f;
    }

    /**
     * Generates the complete integration and vector flow field towards one or more goal cells.
     */
    void compute(const std::vector<GridNode>& goals) {
        const std::size_t total_cells = static_cast<std::size_t>(width_ * height_);
        std::fill(integration_field_.begin(), integration_field_.end(), UNREACHABLE);
        std::fill(flow_field_.begin(), flow_field_.end(), math::Vec3f{0.0f, 0.0f, 0.0f});

        if (goals.empty()) return;

        struct NodeCost {
            int x;
            int y;
            float cost;

            bool operator>(const NodeCost& other) const noexcept {
                return cost > other.cost;
            }
        };

        std::priority_queue<NodeCost, std::vector<NodeCost>, std::greater<>> open_set;

        for (const auto& g : goals) {
            if (in_bounds(g.x, g.y) && !is_blocked(g.x, g.y)) {
                const std::size_t idx = to_index(g.x, g.y);
                integration_field_[idx] = 0.0f;
                open_set.push({g.x, g.y, 0.0f});
            }
        }

        constexpr float sqrt2 = 1.41421356f;
        constexpr int dx[8] = { 0,  1, 0, -1,  1, -1,  1, -1};
        constexpr int dy[8] = {-1,  0, 1,  0, -1, -1,  1,  1};
        constexpr float step_cost[8] = {1.0f, 1.0f, 1.0f, 1.0f, sqrt2, sqrt2, sqrt2, sqrt2};

        // 1. Integration Field Dijkstra Wave Propagation
        while (!open_set.empty()) {
            const auto current = open_set.top();
            open_set.pop();

            const std::size_t curr_idx = to_index(current.x, current.y);
            if (current.cost > integration_field_[curr_idx]) {
                continue;
            }

            for (int i = 0; i < 8; ++i) {
                const int nx = current.x + dx[i];
                const int ny = current.y + dy[i];

                if (!in_bounds(nx, ny)) continue;
                const std::size_t neighbor_idx = to_index(nx, ny);
                const float cell_weight = cost_field_[neighbor_idx];
                if (cell_weight >= 254.0f) continue; // Blocked

                // Corner-cutting check for diagonals
                if (i >= 4) {
                    if (is_blocked(current.x + dx[i], current.y) || is_blocked(current.x, current.y + dy[i])) {
                        continue;
                    }
                }

                const float new_cost = current.cost + cell_weight * step_cost[i];
                if (new_cost < integration_field_[neighbor_idx]) {
                    integration_field_[neighbor_idx] = new_cost;
                    open_set.push({nx, ny, new_cost});
                }
            }
        }

        // 2. Vector Field Downhill Gradient Generation
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                const std::size_t idx = to_index(x, y);
                if (integration_field_[idx] == UNREACHABLE || is_blocked(x, y)) {
                    flow_field_[idx] = {0.0f, 0.0f, 0.0f};
                    continue;
                }

                float lowest_cost = integration_field_[idx];
                math::Vec3f best_dir{0.0f, 0.0f, 0.0f};

                for (int i = 0; i < 8; ++i) {
                    const int nx = x + dx[i];
                    const int ny = y + dy[i];
                    if (!in_bounds(nx, ny)) continue;

                    if (i >= 4 && (is_blocked(x + dx[i], y) || is_blocked(x, y + dy[i]))) {
                        continue;
                    }

                    const float neighbor_cost = integration_field_[to_index(nx, ny)];
                    if (neighbor_cost < lowest_cost) {
                        lowest_cost = neighbor_cost;
                        best_dir = math::Vec3f{static_cast<float>(dx[i]), 0.0f, static_cast<float>(dy[i])};
                    }
                }

                if (best_dir.length_squared() > 1e-6f) {
                    flow_field_[idx] = best_dir.normalized();
                } else {
                    flow_field_[idx] = {0.0f, 0.0f, 0.0f}; // At goal
                }
            }
        }
    }

    /**
     * O(1) smooth bilinear sampling of the flow direction vector for an agent at continuous world position.
     */
    [[nodiscard]] math::Vec3f sample(const math::Vec3f& world_pos) const noexcept {
        const float gx = (world_pos.x - origin_.x) / cell_size_ - 0.5f;
        const float gz = (world_pos.z - origin_.z) / cell_size_ - 0.5f;

        const int x0 = static_cast<int>(std::floor(gx));
        const int z0 = static_cast<int>(std::floor(gz));
        const int x1 = x0 + 1;
        const int z1 = z0 + 1;

        const float fx = gx - static_cast<float>(x0);
        const float fz = gz - static_cast<float>(z0);

        const auto get_vector = [this](int x, int z) -> math::Vec3f {
            if (!in_bounds(x, z)) return {};
            return flow_field_[to_index(x, z)];
        };

        const math::Vec3f v00 = get_vector(x0, z0);
        const math::Vec3f v10 = get_vector(x1, z0);
        const math::Vec3f v01 = get_vector(x0, z1);
        const math::Vec3f v11 = get_vector(x1, z1);

        // Bilinear blend
        const math::Vec3f v0 = v00 * (1.0f - fx) + v10 * fx;
        const math::Vec3f v1 = v01 * (1.0f - fx) + v11 * fx;
        const math::Vec3f blended = v0 * (1.0f - fz) + v1 * fz;

        if (blended.length_squared() > 1e-4f) {
            return blended.normalized();
        }
        return get_vector(static_cast<int>(std::round(gx + 0.5f)), static_cast<int>(std::round(gz + 0.5f)));
    }

    [[nodiscard]] float get_distance_to_goal(int x, int y) const noexcept {
        if (!in_bounds(x, y)) return UNREACHABLE;
        return integration_field_[to_index(x, y)];
    }

    [[nodiscard]] math::Vec3f get_flow_at_cell(int x, int y) const noexcept {
        if (!in_bounds(x, y)) return {};
        return flow_field_[to_index(x, y)];
    }

private:
    [[nodiscard]] bool in_bounds(int x, int y) const noexcept {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    [[nodiscard]] std::size_t to_index(int x, int y) const noexcept {
        return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
    }

    int width_{0};
    int height_{0};
    float cell_size_{1.0f};
    math::Vec3f origin_{};

    std::vector<float> cost_field_;
    std::vector<float> integration_field_;
    std::vector<math::Vec3f> flow_field_;
};

} // namespace scandium::navigation
