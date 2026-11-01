#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "scandium/math/ray.hpp"
#include "scandium/math/vec3.hpp"
#include "scandium/physics/aabb.hpp"
#include "scandium/physics/ray_intersection.hpp"

namespace scandium::spatial {

struct RaycastHit {
    std::uint32_t primitive_id{0};
    float distance{0.0f};
};

// High-performance Linear Bounding Volume Hierarchy (LBVH)
// Uses 63-bit Morton code spatial sorting for multi-scale entity broadphase and logarithmic raycasting.
class LinearBVH {
public:
    struct alignas(32) Node {
        physics::Aabb aabb{};
        std::uint32_t left_child{0};
        std::uint32_t right_child{0};
        std::uint32_t primitive_id{0};
        bool is_leaf{false};
    };

    LinearBVH() = default;

    void build(const std::vector<physics::Aabb>& boxes) {
        clear();
        if (boxes.empty()) {
            return;
        }

        const std::size_t n = boxes.size();
        if (n == 1) {
            nodes_.push_back(Node{boxes[0], 0, 0, 0, true});
            return;
        }

        // 1. Calculate global scene bounds
        physics::Aabb global_bounds = boxes[0];
        for (std::size_t i = 1; i < n; ++i) {
            global_bounds = physics::merge(global_bounds, boxes[i]);
        }

        const math::Vec3f extent = global_bounds.max - global_bounds.min;
        const float inv_extent_x = extent.x > 1e-6f ? 1.0f / extent.x : 0.0f;
        const float inv_extent_y = extent.y > 1e-6f ? 1.0f / extent.y : 0.0f;
        const float inv_extent_z = extent.z > 1e-6f ? 1.0f / extent.z : 0.0f;

        // 2. Generate 63-bit Morton codes for all primitive centers
        struct PrimitiveMorton {
            std::uint64_t code{0};
            std::uint32_t id{0};

            bool operator<(const PrimitiveMorton& rhs) const noexcept {
                return code < rhs.code;
            }
        };

        std::vector<PrimitiveMorton> primitives;
        primitives.reserve(n);

        for (std::size_t i = 0; i < n; ++i) {
            const math::Vec3f c = boxes[i].center();
            const float nx = std::clamp((c.x - global_bounds.min.x) * inv_extent_x, 0.0f, 1.0f);
            const float ny = std::clamp((c.y - global_bounds.min.y) * inv_extent_y, 0.0f, 1.0f);
            const float nz = std::clamp((c.z - global_bounds.min.z) * inv_extent_z, 0.0f, 1.0f);

            const auto qx = static_cast<std::uint64_t>(nx * 2097151.0f);
            const auto qy = static_cast<std::uint64_t>(ny * 2097151.0f);
            const auto qz = static_cast<std::uint64_t>(nz * 2097151.0f);

            primitives.push_back({morton3d(qx, qy, qz), static_cast<std::uint32_t>(i)});
        }

        // 3. Sort primitives along Z-order space-filling curve
        std::sort(primitives.begin(), primitives.end());

        // 4. Construct binary hierarchy over sorted array
        nodes_.reserve(2 * n);
        build_recursive(boxes, primitives, 0, n);
    }

    void clear() noexcept {
        nodes_.clear();
    }

    [[nodiscard]] bool empty() const noexcept {
        return nodes_.empty();
    }

    [[nodiscard]] std::size_t node_count() const noexcept {
        return nodes_.size();
    }

    [[nodiscard]] const std::vector<Node>& nodes() const noexcept {
        return nodes_;
    }

    // Fast non-recursive AABB spatial query
    template <typename Callback>
    void query(const physics::Aabb& search_box, Callback&& cb) const {
        if (nodes_.empty()) {
            return;
        }

        std::uint32_t stack[64];
        int stack_ptr = 0;
        stack[stack_ptr++] = 0;

        while (stack_ptr > 0) {
            const std::uint32_t curr_idx = stack[--stack_ptr];
            const auto& node = nodes_[curr_idx];

            if (!node.aabb.intersects(search_box)) {
                continue;
            }

            if (node.is_leaf) {
                cb(node.primitive_id);
            } else {
                stack[stack_ptr++] = node.right_child;
                stack[stack_ptr++] = node.left_child;
            }
        }
    }

    [[nodiscard]] std::vector<std::uint32_t> query(const physics::Aabb& search_box) const {
        std::vector<std::uint32_t> result;
        result.reserve(16);
        query(search_box, [&result](std::uint32_t id) {
            result.push_back(id);
        });
        return result;
    }

    // Fast logarithmic raycast against bounding volume hierarchy
    [[nodiscard]] std::optional<RaycastHit> raycast(
        const math::Ray& ray,
        float max_distance = std::numeric_limits<float>::infinity()) const noexcept {

        if (nodes_.empty()) {
            return std::nullopt;
        }

        std::uint32_t stack[64];
        int stack_ptr = 0;
        stack[stack_ptr++] = 0;

        std::optional<RaycastHit> closest_hit;
        float current_max = max_distance;

        while (stack_ptr > 0) {
            const std::uint32_t curr_idx = stack[--stack_ptr];
            const auto& node = nodes_[curr_idx];

            const auto hit_t = physics::intersect_ray_aabb(ray, node.aabb, current_max);
            if (!hit_t.has_value()) {
                continue;
            }

            if (node.is_leaf) {
                if (*hit_t < current_max) {
                    current_max = *hit_t;
                    closest_hit = RaycastHit{node.primitive_id, *hit_t};
                }
            } else {
                stack[stack_ptr++] = node.right_child;
                stack[stack_ptr++] = node.left_child;
            }
        }

        return closest_hit;
    }

private:
    static constexpr std::uint64_t expand_bits(std::uint64_t v) noexcept {
        v &= 0x1FFFFFULL;
        v = (v | (v << 32)) & 0x001F00000000FFFFULL;
        v = (v | (v << 16)) & 0x001F0000FF0000FFULL;
        v = (v | (v << 8))  & 0x010F00F00F00F00FULL;
        v = (v | (v << 4))  & 0x10C30C30C30C30C3ULL;
        v = (v | (v << 2))  & 0x1249249249249249ULL;
        return v;
    }

    static constexpr std::uint64_t morton3d(std::uint64_t x, std::uint64_t y, std::uint64_t z) noexcept {
        return (expand_bits(x) << 2) | (expand_bits(y) << 1) | expand_bits(z);
    }

    template <typename PrimitiveMorton>
    std::uint32_t build_recursive(
        const std::vector<physics::Aabb>& boxes,
        const std::vector<PrimitiveMorton>& primitives,
        std::size_t start,
        std::size_t end) {

        const std::size_t count = end - start;
        const auto node_idx = static_cast<std::uint32_t>(nodes_.size());
        nodes_.push_back(Node{});

        if (count == 1) {
            const auto prim_id = primitives[start].id;
            nodes_[node_idx] = Node{
                boxes[prim_id],
                0,
                0,
                prim_id,
                true
            };
            return node_idx;
        }

        const std::size_t mid = start + count / 2;
        const std::uint32_t left = build_recursive(boxes, primitives, start, mid);
        const std::uint32_t right = build_recursive(boxes, primitives, mid, end);

        nodes_[node_idx] = Node{
            physics::merge(nodes_[left].aabb, nodes_[right].aabb),
            left,
            right,
            0,
            false
        };

        return node_idx;
    }

    std::vector<Node> nodes_;
};

} // namespace scandium::spatial
