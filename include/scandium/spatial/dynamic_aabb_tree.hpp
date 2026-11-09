#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include "scandium/math/ray.hpp"
#include "scandium/math/vec3.hpp"
#include "scandium/physics/aabb.hpp"
#include "scandium/physics/ray_intersection.hpp"

namespace scandium::spatial {

/**
 * @brief Dynamic self-balancing bounding volume hierarchy (AABB Tree).
 *
 * Implements:
 *   - Surface Area Heuristic (SAH) greedy insertion.
 *   - Tree rotations (AVL-style) to preserve O(log N) query depth.
 *   - Fat-AABB margins to avoid tree rebuilds/reinsertions for small object movements.
 *   - Velocity-predictive bounds expansion.
 *   - Non-recursive stack-based AABB overlap queries, raycasts, and self-collision pair generation.
 *   - Pre-allocated pooled nodes with free-list recycling.
 */
class DynamicAABBTree {
public:
    static constexpr std::int32_t NullNode = -1;

    struct alignas(32) Node {
        physics::Aabb aabb{};
        std::uint32_t user_data{0};
        std::int32_t parent{NullNode};
        std::int32_t left{NullNode};
        std::int32_t right{NullNode};
        std::int32_t height{0};

        [[nodiscard]] bool is_leaf() const noexcept {
            return left == NullNode;
        }
    };

    explicit DynamicAABBTree(float fat_margin = 0.2f, std::size_t initial_capacity = 64)
        : fat_margin_(fat_margin) {
        nodes_.resize(initial_capacity);
        for (std::size_t i = 0; i < initial_capacity - 1; ++i) {
            nodes_[i].parent = static_cast<std::int32_t>(i + 1);
            nodes_[i].height = -1;
        }
        nodes_[initial_capacity - 1].parent = NullNode;
        nodes_[initial_capacity - 1].height = -1;
        free_head_ = 0;
    }

    ~DynamicAABBTree() = default;

    DynamicAABBTree(const DynamicAABBTree&) = default;
    DynamicAABBTree& operator=(const DynamicAABBTree&) = default;
    DynamicAABBTree(DynamicAABBTree&&) noexcept = default;
    DynamicAABBTree& operator=(DynamicAABBTree&&) noexcept = default;

    /**
     * @brief Computes half surface area of an AABB.
     */
    [[nodiscard]] static constexpr float surface_area(const physics::Aabb& a) noexcept {
        const math::Vec3f d = a.max - a.min;
        return d.x * d.y + d.y * d.z + d.z * d.x;
    }

    /**
     * @brief Inserts a new leaf node for an entity.
     * @param user_data Identifier or entity ID.
     * @param tight_aabb Exact bounding box.
     * @param velocity Optional velocity for predictive fat bounding box expansion.
     * @return Unique node ID.
     */
    std::int32_t insert(std::uint32_t user_data, const physics::Aabb& tight_aabb,
                        const math::Vec3f& velocity = {}) {
        const std::int32_t node_id = allocate_node();
        auto& node = nodes_[node_id];
        node.user_data = user_data;
        node.aabb = make_fat_aabb(tight_aabb, velocity);
        node.height = 0;

        insert_leaf(node_id);
        ++leaf_count_;
        return node_id;
    }

    /**
     * @brief Removes a leaf node from the tree.
     */
    void remove(std::int32_t node_id) {
        assert(node_id >= 0 && static_cast<std::size_t>(node_id) < nodes_.size());
        assert(nodes_[node_id].is_leaf());

        remove_leaf(node_id);
        free_node(node_id);
        --leaf_count_;
    }

    /**
     * @brief Updates an existing leaf node.
     * Fast-path: If the tight AABB is still contained inside the fat AABB, returns false
     * and performs NO tree modification ($O(1)$).
     * @return true if tree was rebalanced, false if tight AABB was still within fat margin.
     */
    bool move(std::int32_t node_id, const physics::Aabb& new_tight_aabb,
              const math::Vec3f& velocity = {}) {
        assert(node_id >= 0 && static_cast<std::size_t>(node_id) < nodes_.size());
        assert(nodes_[node_id].is_leaf());

        const auto& fat = nodes_[node_id].aabb;
        if (fat.min.x <= new_tight_aabb.min.x && fat.max.x >= new_tight_aabb.max.x &&
            fat.min.y <= new_tight_aabb.min.y && fat.max.y >= new_tight_aabb.max.y &&
            fat.min.z <= new_tight_aabb.min.z && fat.max.z >= new_tight_aabb.max.z) {
            return false; // Fast path: Still within fat AABB boundary
        }

        remove_leaf(node_id);
        nodes_[node_id].aabb = make_fat_aabb(new_tight_aabb, velocity);
        insert_leaf(node_id);
        return true;
    }

    /**
     * @brief Queries all primitives whose fat AABB overlaps with the search AABB.
     */
    template <typename Callback>
    void query(const physics::Aabb& query_aabb, Callback&& cb) const {
        if (root_ == NullNode) {
            return;
        }

        thread_local std::vector<std::int32_t> stack;
        stack.clear();
        stack.push_back(root_);

        while (!stack.empty()) {
            const std::int32_t curr = stack.back();
            stack.pop_back();

            const auto& node = nodes_[curr];
            if (!node.aabb.intersects(query_aabb)) {
                continue;
            }

            if (node.is_leaf()) {
                if constexpr (std::is_invocable_r_v<bool, Callback, std::uint32_t>) {
                    if (!cb(node.user_data)) {
                        return; // Early termination requested
                    }
                } else {
                    cb(node.user_data);
                }
            } else {
                stack.push_back(node.left);
                stack.push_back(node.right);
            }
        }
    }

    /**
     * @brief Raycasts against the dynamic tree.
     */
    template <typename Callback>
    void raycast(const math::Ray& ray, Callback&& cb) const {
        if (root_ == NullNode) {
            return;
        }

        thread_local std::vector<std::int32_t> stack;
        stack.clear();
        stack.push_back(root_);

        while (!stack.empty()) {
            const std::int32_t curr = stack.back();
            stack.pop_back();

            const auto& node = nodes_[curr];
            const auto hit = physics::intersect_ray_aabb(ray, node.aabb);
            if (!hit) {
                continue;
            }
            const float t_hit = *hit;

            if (node.is_leaf()) {
                if constexpr (std::is_invocable_r_v<bool, Callback, std::uint32_t, float>) {
                    if (!cb(node.user_data, t_hit)) {
                        return;
                    }
                } else {
                    cb(node.user_data, t_hit);
                }
            } else {
                stack.push_back(node.left);
                stack.push_back(node.right);
            }
        }
    }

    /**
     * @brief Finds all overlapping pairs among leaf nodes in the tree.
     */
    template <typename PairCallback>
    void query_pairs(PairCallback&& cb) const {
        if (root_ == NullNode || nodes_[root_].is_leaf()) {
            return;
        }

        thread_local std::vector<std::pair<std::int32_t, std::int32_t>> stack;
        stack.clear();
        stack.push_back({nodes_[root_].left, nodes_[root_].right});

        while (!stack.empty()) {
            const auto [n1, n2] = stack.back();
            stack.pop_back();

            const auto& node1 = nodes_[n1];
            const auto& node2 = nodes_[n2];

            if (!node1.aabb.intersects(node2.aabb)) {
                continue;
            }

            if (node1.is_leaf() && node2.is_leaf()) {
                cb(node1.user_data, node2.user_data);
            } else if (node1.is_leaf()) {
                stack.push_back({n1, node2.left});
                stack.push_back({n1, node2.right});
            } else if (node2.is_leaf()) {
                stack.push_back({node1.left, n2});
                stack.push_back({node1.right, n2});
            } else {
                // Descend larger volume
                if (surface_area(node1.aabb) > surface_area(node2.aabb)) {
                    stack.push_back({node1.left, n2});
                    stack.push_back({node1.right, n2});
                } else {
                    stack.push_back({n1, node2.left});
                    stack.push_back({n1, node2.right});
                }
            }
        }
    }

    void clear() noexcept {
        const std::size_t cap = nodes_.size();
        for (std::size_t i = 0; i < cap - 1; ++i) {
            nodes_[i].parent = static_cast<std::int32_t>(i + 1);
            nodes_[i].height = -1;
        }
        nodes_[cap - 1].parent = NullNode;
        nodes_[cap - 1].height = -1;
        free_head_ = 0;
        root_ = NullNode;
        leaf_count_ = 0;
    }

    [[nodiscard]] std::size_t leaf_count() const noexcept {
        return leaf_count_;
    }

    [[nodiscard]] bool empty() const noexcept {
        return leaf_count_ == 0;
    }

    [[nodiscard]] std::int32_t root() const noexcept {
        return root_;
    }

    [[nodiscard]] const Node& get_node(std::int32_t id) const noexcept {
        return nodes_[id];
    }

    [[nodiscard]] std::int32_t height() const noexcept {
        return root_ == NullNode ? 0 : nodes_[root_].height;
    }

private:
    [[nodiscard]] physics::Aabb make_fat_aabb(const physics::Aabb& a, const math::Vec3f& v) const noexcept {
        math::Vec3f margin_vec{fat_margin_, fat_margin_, fat_margin_};
        physics::Aabb fat{a.min - margin_vec, a.max + margin_vec};

        // Predictive velocity expansion
        if (v.x < 0.0f) fat.min.x += v.x * 2.0f;
        else fat.max.x += v.x * 2.0f;

        if (v.y < 0.0f) fat.min.y += v.y * 2.0f;
        else fat.max.y += v.y * 2.0f;

        if (v.z < 0.0f) fat.min.z += v.z * 2.0f;
        else fat.max.z += v.z * 2.0f;

        return fat;
    }

    std::int32_t allocate_node() {
        if (free_head_ == NullNode) {
            // Expand node capacity
            const std::size_t old_cap = nodes_.size();
            const std::size_t new_cap = old_cap * 2;
            nodes_.resize(new_cap);

            for (std::size_t i = old_cap; i < new_cap - 1; ++i) {
                nodes_[i].parent = static_cast<std::int32_t>(i + 1);
                nodes_[i].height = -1;
            }
            nodes_[new_cap - 1].parent = NullNode;
            nodes_[new_cap - 1].height = -1;
            free_head_ = static_cast<std::int32_t>(old_cap);
        }

        const std::int32_t id = free_head_;
        free_head_ = nodes_[id].parent;
        nodes_[id].parent = NullNode;
        nodes_[id].left = NullNode;
        nodes_[id].right = NullNode;
        nodes_[id].height = 0;
        return id;
    }

    void free_node(std::int32_t id) {
        assert(id >= 0 && static_cast<std::size_t>(id) < nodes_.size());
        nodes_[id].parent = free_head_;
        nodes_[id].height = -1;
        free_head_ = id;
    }

    void insert_leaf(std::int32_t leaf) {
        if (root_ == NullNode) {
            root_ = leaf;
            nodes_[root_].parent = NullNode;
            return;
        }

        // 1. Surface Area Heuristic (SAH) traversal to select optimal sibling
        const physics::Aabb leaf_aabb = nodes_[leaf].aabb;
        const float leaf_area = surface_area(leaf_aabb);
        std::int32_t curr = root_;

        while (!nodes_[curr].is_leaf()) {
            const std::int32_t left = nodes_[curr].left;
            const std::int32_t right = nodes_[curr].right;

            const float combined_area = surface_area(physics::merge(nodes_[curr].aabb, leaf_aabb));
            const float cost = 2.0f * combined_area;
            const float inheritance_cost = 2.0f * (combined_area - surface_area(nodes_[curr].aabb));

            // Cost of descending into left child
            float cost_left = 0.0f;
            if (nodes_[left].is_leaf()) {
                cost_left = surface_area(physics::merge(nodes_[left].aabb, leaf_aabb)) + inheritance_cost;
            } else {
                const float old_area = surface_area(nodes_[left].aabb);
                const float new_area = surface_area(physics::merge(nodes_[left].aabb, leaf_aabb));
                cost_left = (new_area - old_area) + inheritance_cost;
            }

            // Cost of descending into right child
            float cost_right = 0.0f;
            if (nodes_[right].is_leaf()) {
                cost_right = surface_area(physics::merge(nodes_[right].aabb, leaf_aabb)) + inheritance_cost;
            } else {
                const float old_area = surface_area(nodes_[right].aabb);
                const float new_area = surface_area(physics::merge(nodes_[right].aabb, leaf_aabb));
                cost_right = (new_area - old_area) + inheritance_cost;
            }

            // Descend if either child path is cheaper than creating parent here
            if (cost < cost_left && cost < cost_right) {
                break;
            }

            curr = (cost_left < cost_right) ? left : right;
        }

        const std::int32_t sibling = curr;

        // 2. Create new internal parent node
        const std::int32_t old_parent = nodes_[sibling].parent;
        const std::int32_t new_parent = allocate_node();

        nodes_[new_parent].parent = old_parent;
        nodes_[new_parent].aabb = physics::merge(leaf_aabb, nodes_[sibling].aabb);
        nodes_[new_parent].height = nodes_[sibling].height + 1;

        if (old_parent != NullNode) {
            if (nodes_[old_parent].left == sibling) {
                nodes_[old_parent].left = new_parent;
            } else {
                nodes_[old_parent].right = new_parent;
            }
        } else {
            root_ = new_parent;
        }

        nodes_[new_parent].left = sibling;
        nodes_[new_parent].right = leaf;
        nodes_[sibling].parent = new_parent;
        nodes_[leaf].parent = new_parent;

        // 3. Walk back up the tree refitting AABBs and balancing heights
        curr = nodes_[leaf].parent;
        while (curr != NullNode) {
            curr = balance(curr);

            const std::int32_t l = nodes_[curr].left;
            const std::int32_t r = nodes_[curr].right;
            assert(l != NullNode && r != NullNode);

            nodes_[curr].height = 1 + std::max(nodes_[l].height, nodes_[r].height);
            nodes_[curr].aabb = physics::merge(nodes_[l].aabb, nodes_[r].aabb);

            curr = nodes_[curr].parent;
        }
    }

    void remove_leaf(std::int32_t leaf) {
        if (leaf == root_) {
            root_ = NullNode;
            return;
        }

        const std::int32_t parent = nodes_[leaf].parent;
        const std::int32_t grand_parent = nodes_[parent].parent;
        const std::int32_t sibling = (nodes_[parent].left == leaf) ? nodes_[parent].right : nodes_[parent].left;

        if (grand_parent != NullNode) {
            if (nodes_[grand_parent].left == parent) {
                nodes_[grand_parent].left = sibling;
            } else {
                nodes_[grand_parent].right = sibling;
            }
            nodes_[sibling].parent = grand_parent;
            free_node(parent);

            // Refit and balance upwards
            std::int32_t curr = grand_parent;
            while (curr != NullNode) {
                curr = balance(curr);

                const std::int32_t l = nodes_[curr].left;
                const std::int32_t r = nodes_[curr].right;

                nodes_[curr].aabb = physics::merge(nodes_[l].aabb, nodes_[r].aabb);
                nodes_[curr].height = 1 + std::max(nodes_[l].height, nodes_[r].height);

                curr = nodes_[curr].parent;
            }
        } else {
            root_ = sibling;
            nodes_[sibling].parent = NullNode;
            free_node(parent);
        }
    }

    // AVL tree rotation to maintain logarithmic balance
    std::int32_t balance(std::int32_t i) {
        assert(i != NullNode);
        auto& a = nodes_[i];
        if (a.is_leaf() || a.height < 2) {
            return i;
        }

        const std::int32_t b_idx = a.left;
        const std::int32_t c_idx = a.right;
        auto& b = nodes_[b_idx];
        auto& c = nodes_[c_idx];

        const std::int32_t balance_factor = c.height - b.height;

        // Rotate C up
        if (balance_factor > 1) {
            const std::int32_t f_idx = c.left;
            const std::int32_t g_idx = c.right;
            auto& f = nodes_[f_idx];
            auto& g = nodes_[g_idx];

            c.left = i;
            c.parent = a.parent;
            a.parent = c_idx;

            if (c.parent != NullNode) {
                if (nodes_[c.parent].left == i) {
                    nodes_[c.parent].left = c_idx;
                } else {
                    nodes_[c.parent].right = c_idx;
                }
            } else {
                root_ = c_idx;
            }

            // Rotate
            if (f.height > g.height) {
                c.right = f_idx;
                a.right = g_idx;
                g.parent = i;
                a.aabb = physics::merge(b.aabb, g.aabb);
                c.aabb = physics::merge(a.aabb, f.aabb);

                a.height = 1 + std::max(b.height, g.height);
                c.height = 1 + std::max(a.height, f.height);
            } else {
                c.right = g_idx;
                a.right = f_idx;
                f.parent = i;
                a.aabb = physics::merge(b.aabb, f.aabb);
                c.aabb = physics::merge(a.aabb, g.aabb);

                a.height = 1 + std::max(b.height, f.height);
                c.height = 1 + std::max(a.height, g.height);
            }

            return c_idx;
        }

        // Rotate B up
        if (balance_factor < -1) {
            const std::int32_t d_idx = b.left;
            const std::int32_t e_idx = b.right;
            auto& d = nodes_[d_idx];
            auto& e = nodes_[e_idx];

            b.left = i;
            b.parent = a.parent;
            a.parent = b_idx;

            if (b.parent != NullNode) {
                if (nodes_[b.parent].left == i) {
                    nodes_[b.parent].left = b_idx;
                } else {
                    nodes_[b.parent].right = b_idx;
                }
            } else {
                root_ = b_idx;
            }

            // Rotate
            if (d.height > e.height) {
                b.right = d_idx;
                a.left = e_idx;
                e.parent = i;
                a.aabb = physics::merge(c.aabb, e.aabb);
                b.aabb = physics::merge(a.aabb, d.aabb);

                a.height = 1 + std::max(c.height, e.height);
                b.height = 1 + std::max(a.height, d.height);
            } else {
                b.right = e_idx;
                a.left = d_idx;
                d.parent = i;
                a.aabb = physics::merge(c.aabb, d.aabb);
                b.aabb = physics::merge(a.aabb, e.aabb);

                a.height = 1 + std::max(c.height, d.height);
                b.height = 1 + std::max(a.height, e.height);
            }

            return b_idx;
        }

        return i;
    }

    std::vector<Node> nodes_{};
    std::int32_t root_{NullNode};
    std::int32_t free_head_{NullNode};
    std::size_t leaf_count_{0};
    float fat_margin_{0.2f};
};

} // namespace scandium::spatial
