#include <cassert>
#include <vector>

#include "scandium/physics/aabb.hpp"
#include "scandium/spatial/dynamic_aabb_tree.hpp"

int main() {
    using scandium::math::Vec3f;
    using scandium::math::Ray;
    using scandium::physics::Aabb;
    using scandium::physics::merge;
    using scandium::spatial::DynamicAABBTree;

    // Test 1: Basic AABB math
    const Aabb player{{-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}};
    const Aabb obstacle{{0.5f, 0.5f, 0.5f}, {2.0f, 2.0f, 2.0f}};
    const Aabb distant{{5.0f, 5.0f, 5.0f}, {6.0f, 6.0f, 6.0f}};

    assert(player.intersects(obstacle));
    assert(!player.intersects(distant));

    assert(player.contains(Vec3f{0.0f, 0.0f, 0.0f}));
    assert(!player.contains(Vec3f{1.1f, 0.0f, 0.0f}));

    const auto combined = merge(player, distant);
    assert(combined.min.x == -1.0f);
    assert(combined.min.y == -1.0f);
    assert(combined.max.z == 6.0f);

    // Test 2: DynamicAABBTree lifecycle, SAH insertion & removal
    {
        DynamicAABBTree tree(0.2f);
        assert(tree.empty());
        assert(tree.leaf_count() == 0);

        const auto n1 = tree.insert(101, player);
        const auto n2 = tree.insert(102, obstacle);
        const auto n3 = tree.insert(103, distant);

        assert(!tree.empty());
        assert(tree.leaf_count() == 3);
        assert(tree.height() >= 1);

        // Query overlapping with origin
        std::vector<std::uint32_t> hit_ids;
        tree.query(Aabb{{-0.1f, -0.1f, -0.1f}, {0.1f, 0.1f, 0.1f}}, [&](std::uint32_t id) {
            hit_ids.push_back(id);
        });
        assert(hit_ids.size() == 1);
        assert(hit_ids[0] == 101);

        // Fat AABB fast-path test: small movement within fat margin (0.2f)
        const Aabb slight_move{{-0.95f, -1.0f, -1.0f}, {1.05f, 1.0f, 1.0f}};
        const bool rebalanced = tree.move(n1, slight_move);
        assert(!rebalanced); // Stayed within fat margin, zero tree modifications!

        // Movement outside fat margin triggers tree update
        const Aabb large_move{{10.0f, 10.0f, 10.0f}, {12.0f, 12.0f, 12.0f}};
        const bool moved = tree.move(n1, large_move);
        assert(moved); // Successfully rebalanced!

        // Removal
        tree.remove(n2);
        assert(tree.leaf_count() == 2);
    }

    // Test 3: Raycasting
    {
        DynamicAABBTree tree(0.1f);
        tree.insert(1, Aabb{{10.0f, -1.0f, -1.0f}, {12.0f, 1.0f, 1.0f}});
        tree.insert(2, Aabb{{20.0f, -1.0f, -1.0f}, {22.0f, 1.0f, 1.0f}});
        tree.insert(3, Aabb{{-10.0f, 5.0f, 5.0f}, {-8.0f, 7.0f, 7.0f}}); // Off to side

        Ray ray{Vec3f{0.0f, 0.0f, 0.0f}, Vec3f{1.0f, 0.0f, 0.0f}}; // Pointing along +X
        std::vector<std::uint32_t> ray_hits;
        tree.raycast(ray, [&](std::uint32_t id, float dist) {
            ray_hits.push_back(id);
            assert(dist > 0.0f);
        });

        assert(ray_hits.size() == 2);
        // Both boxes on +X axis were hit, box 3 was pruned
        assert((ray_hits[0] == 1 && ray_hits[1] == 2) || (ray_hits[0] == 2 && ray_hits[1] == 1));
    }

    // Test 4: Pair queries (Broadphase collision detection)
    {
        DynamicAABBTree tree(0.0f); // 0 margin for exact box pairs
        tree.insert(1, Aabb{{0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f}});
        tree.insert(2, Aabb{{1.0f, 1.0f, 1.0f}, {3.0f, 3.0f, 3.0f}}); // Overlaps with 1
        tree.insert(3, Aabb{{10.0f, 10.0f, 10.0f}, {12.0f, 12.0f, 12.0f}}); // Disjoint

        std::size_t pair_count = 0;
        tree.query_pairs([&](std::uint32_t a, std::uint32_t b) {
            assert((a == 1 && b == 2) || (a == 2 && b == 1));
            ++pair_count;
        });
        assert(pair_count == 1);
    }

    // Test 5: Balanced Tree Height with 500 Random Entities
    {
        DynamicAABBTree tree(0.1f);
        constexpr int count = 500;
        std::vector<std::int32_t> nodes;
        nodes.reserve(count);

        for (int i = 0; i < count; ++i) {
            const float x = static_cast<float>((i * 37) % 500);
            const float y = static_cast<float>((i * 17) % 500);
            const float z = static_cast<float>((i * 53) % 500);
            nodes.push_back(tree.insert(i, Aabb{{x, y, z}, {x + 2.0f, y + 2.0f, z + 2.0f}}));
        }

        assert(tree.leaf_count() == count);
        // For 500 nodes, log2(500) ~ 9. With AVL balancing, tree height should remain <= 20
        assert(tree.height() <= 20);

        // Remove half
        for (int i = 0; i < count; i += 2) {
            tree.remove(nodes[i]);
        }
        assert(tree.leaf_count() == count / 2);
        assert(tree.height() <= 18);
    }

    return 0;
}
