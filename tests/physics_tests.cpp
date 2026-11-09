#include <cassert>
#include <cmath>

#include "scandium/math/quaternion.hpp"
#include "scandium/math/ray.hpp"
#include "scandium/physics/ray_intersection.hpp"
#include "scandium/spatial/spatial_hash.hpp"
#include "scandium/spatial/linear_bvh.hpp"


#include "scandium/physics/ccd.hpp"
#include "scandium/physics/simd_ray_intersection.hpp"

void test_continuous_collision_detection() {
    using namespace scandium::physics;
    using scandium::math::Vec3f;

    // 1. Swept sphere vs sphere head-on
    {
        const Vec3f pos_a{0.0f, 0.0f, 0.0f};
        const Vec3f vel_a{10.0f, 0.0f, 0.0f};
        const float rad_a = 1.0f;

        const Vec3f pos_b{10.0f, 0.0f, 0.0f};
        const Vec3f vel_b{-10.0f, 0.0f, 0.0f};
        const float rad_b = 1.0f;

        const auto contact = swept_sphere_vs_sphere(pos_a, vel_a, rad_a, pos_b, vel_b, rad_b, 1.0f);
        assert(contact.has_contact);
        // Distance decreases from 10 to 2 (8 units of relative travel at relative speed 20 -> toi = 8/20 = 0.4)
        assert(std::fabs(contact.toi - 0.4f) < 1e-4f);
        assert(contact.normal.x < -0.99f);
    }

    // 2. Swept sphere vs sphere missing (parallel trajectories)
    {
        const Vec3f pos_a{0.0f, 5.0f, 0.0f};
        const Vec3f vel_a{10.0f, 0.0f, 0.0f};
        const Vec3f pos_b{0.0f, -5.0f, 0.0f};
        const Vec3f vel_b{10.0f, 0.0f, 0.0f};

        const auto contact = swept_sphere_vs_sphere(pos_a, vel_a, 1.0f, pos_b, vel_b, 1.0f, 1.0f);
        assert(!contact.has_contact);
    }

    // 3. Swept sphere vs stationary AABB
    {
        const Vec3f sphere_pos{0.0f, 0.0f, 0.0f};
        const Vec3f sphere_vel{20.0f, 0.0f, 0.0f};
        const float sphere_radius = 0.5f;

        const Aabb box{{8.0f, -2.0f, -2.0f}, {12.0f, 2.0f, 2.0f}};
        const auto contact = swept_sphere_vs_aabb(sphere_pos, sphere_vel, sphere_radius, box, 1.0f);
        assert(contact.has_contact);
        // Distance to box min.x (8.0) minus radius (0.5) is 7.5. Speed is 20 -> toi = 7.5/20 = 0.375
        assert(std::fabs(contact.toi - 0.375f) < 1e-4f);
        assert(contact.normal.x < -0.99f);
    }

    // 4. Collision velocity response
    {
        Vec3f vel{10.0f, 5.0f, 0.0f};
        const Vec3f normal{-1.0f, 0.0f, 0.0f};
        resolve_collision_response(vel, normal, 0.5f, 0.0f);
        // Normal component should rebound to -5.0
        assert(std::fabs(vel.x - (-5.0f)) < 1e-4f);
        assert(std::fabs(vel.y - 5.0f) < 1e-4f);
    }
}

void test_simd_ray_intersection() {
    using namespace scandium::physics;
    using scandium::math::Vec3f;
    using scandium::math::Ray;

    std::vector<Aabb> boxes = {
        Aabb{{10.0f, -1.0f, -1.0f}, {12.0f, 1.0f, 1.0f}}, // Hit (dist ~10)
        Aabb{{20.0f, -1.0f, -1.0f}, {22.0f, 1.0f, 1.0f}}, // Hit (dist ~20)
        Aabb{{5.0f, 5.0f, 5.0f}, {7.0f, 7.0f, 7.0f}},     // Miss
        Aabb{{30.0f, -0.5f, -0.5f}, {35.0f, 0.5f, 0.5f}}, // Hit (dist ~30)
        Aabb{{-5.0f, -1.0f, -1.0f}, {-3.0f, 1.0f, 1.0f}}, // Behind ray origin
        Aabb{{50.0f, -2.0f, -2.0f}, {60.0f, 2.0f, 2.0f}}, // Hit (dist ~50)
        Aabb{{0.0f, 10.0f, 0.0f}, {2.0f, 12.0f, 2.0f}},   // Miss
        Aabb{{80.0f, -1.0f, -1.0f}, {85.0f, 1.0f, 1.0f}}  // Hit (dist ~80)
    };

    Ray ray{Vec3f{0.0f, 0.0f, 0.0f}, Vec3f{1.0f, 0.0f, 0.0f}};

    const auto soa8 = AABB8::from_aabbs(boxes.data(), boxes.size());

    // 1. Scalar baseline
    const auto scalar_res = intersect_ray_8aabb_scalar(ray, soa8);
    // Expected hits: 0, 1, 3, 5, 7 -> mask: (1 | 2 | 8 | 32 | 128) = 171
    assert((scalar_res.hit_mask & (1u << 0)) != 0);
    assert((scalar_res.hit_mask & (1u << 1)) != 0);
    assert((scalar_res.hit_mask & (1u << 2)) == 0); // Miss
    assert((scalar_res.hit_mask & (1u << 3)) != 0);
    assert((scalar_res.hit_mask & (1u << 4)) == 0); // Behind
    assert((scalar_res.hit_mask & (1u << 5)) != 0);
    assert((scalar_res.hit_mask & (1u << 6)) == 0); // Miss
    assert((scalar_res.hit_mask & (1u << 7)) != 0);

    // 2. Hardware-dispatched SIMD
    const auto simd_res = intersect_ray_8aabb(ray, soa8);
    assert(simd_res.hit_mask == scalar_res.hit_mask);

    for (int i = 0; i < 8; ++i) {
        if ((simd_res.hit_mask & (1u << i)) != 0) {
            assert(std::fabs(simd_res.hit_distances[i] - scalar_res.hit_distances[i]) < 1e-4f);
        }
    }

#if defined(_MSC_VER) || defined(__x86_64__) || defined(_M_X64)
    // 3. Explicit AVX2 test
    if (scandium::core::CPUFeatures::has_avx2()) {
        const auto avx_res = intersect_ray_8aabb_avx2(ray, soa8);
        assert(avx_res.hit_mask == scalar_res.hit_mask);
        for (int i = 0; i < 8; ++i) {
            if ((avx_res.hit_mask & (1u << i)) != 0) {
                assert(std::fabs(avx_res.hit_distances[i] - scalar_res.hit_distances[i]) < 1e-4f);
            }
        }
    }

    // 4. SSE 4-wide test
    const auto soa4 = AABB4::from_aabbs(boxes.data(), 4);
    const auto sse_res = intersect_ray_4aabb_sse(ray, soa4);
    // Lanes 0, 1, 3 hit, 2 miss -> mask = 11 (1 | 2 | 8)
    assert((sse_res.hit_mask & 0x0F) == (scalar_res.hit_mask & 0x0F));
#endif
}

int main() {
    using scandium::math::Quaternion;
    using scandium::math::Ray;
    using scandium::math::Vec3f;
    using scandium::physics::Aabb;
    using scandium::physics::intersect_ray_aabb;
    using scandium::spatial::SpatialHash;

    const float half_pi = 1.57079632679f;
    const auto rotation = Quaternion::from_axis_angle({0.0f, 1.0f, 0.0f}, half_pi);
    const auto rotated = rotation.rotate({1.0f, 0.0f, 0.0f});
    assert(std::fabs(rotated.x) < 1e-4f);
    assert(std::fabs(rotated.z + 1.0f) < 1e-4f);

    const Aabb target{{2.0f, -1.0f, -1.0f}, {4.0f, 1.0f, 1.0f}};
    const Ray ray{{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
    const auto hit = intersect_ray_aabb(ray, target);
    assert(hit.has_value());
    assert(std::fabs(*hit - 2.0f) < 1e-5f);

    SpatialHash hash{10.0f};
    hash.insert(7, {1.0f, 1.0f, 1.0f});
    hash.insert(9, {8.0f, 1.0f, 1.0f});
    hash.insert(42, {100.0f, 100.0f, 100.0f});

    const auto nearby = hash.query({2.0f, 2.0f, 2.0f});
    bool found7 = false;
    bool found9 = false;
    for (const auto id : nearby) {
        found7 |= id == 7;
        found9 |= id == 9;
    }
    assert(found7 && found9);

    // Test clear and re-insert
    hash.clear();
    assert(hash.size() == 0);
    assert(hash.query({2.0f, 2.0f, 2.0f}).empty());

    // Test AdaptiveBroadphase selection logic
    using scandium::spatial::AdaptiveBroadphase;
    using scandium::spatial::BroadphaseStrategy;

    AdaptiveBroadphase broadphase{5.0f, 150};
    assert(broadphase.strategy() == BroadphaseStrategy::Auto);
    assert(!broadphase.should_use_spatial_hash(50));
    assert(!broadphase.should_use_spatial_hash(149));
    assert(broadphase.should_use_spatial_hash(150));
    assert(broadphase.should_use_spatial_hash(1000));

    broadphase.set_strategy(BroadphaseStrategy::Naive);
    assert(!broadphase.should_use_spatial_hash(1000));

    broadphase.set_strategy(BroadphaseStrategy::SpatialHash);
    assert(broadphase.should_use_spatial_hash(10));

    broadphase.set_strategy(BroadphaseStrategy::Auto);
    broadphase.set_crossover_threshold(300);
    assert(broadphase.crossover_threshold() == 300);
    assert(!broadphase.should_use_spatial_hash(250));
    assert(broadphase.should_use_spatial_hash(300));

    // Regression Test: SpatialHashDiagnostics and pack/unpack verification
    {
        SpatialHash diag_hash{5.0f, 2048};
        for (std::uint32_t i = 0; i < 500; ++i) {
            const float x = static_cast<float>(i % 25) * 5.0f - 50.0f;
            const float z = static_cast<float>(i / 25) * 5.0f - 50.0f;
            diag_hash.insert(i, {x, 0.0f, z});
        }
        const auto diag = diag_hash.diagnostics();
        assert(diag.total_entries == 500);
        assert(diag.total_buckets == 2048);
        assert(diag.occupied_buckets > 0);
        assert(diag.memory_bytes > 0);
        assert(diag.empty_bucket_pct > 0.0 && diag.empty_bucket_pct < 100.0);
    }

    // Regression Test: SpatialHash vs Naive query equivalence at scale (500 entities)
    {
        constexpr std::size_t entity_count = 500;
        const float cell_size = 5.0f;
        std::vector<Vec3f> positions;
        positions.reserve(entity_count);
        for (std::size_t i = 0; i < entity_count; ++i) {
            const float x = std::sin(static_cast<float>(i) * 0.13f) * 40.0f;
            const float y = std::cos(static_cast<float>(i) * 0.17f) * 40.0f;
            const float z = std::sin(static_cast<float>(i) * 0.23f) * 40.0f;
            positions.push_back({x, y, z});
        }

        SpatialHash hash{cell_size, entity_count * 2};
        for (std::size_t i = 0; i < entity_count; ++i) {
            hash.insert(static_cast<std::uint32_t>(i), positions[i]);
        }

        for (std::size_t q = 0; q < 50; ++q) {
            const auto& qpos = positions[q];
            auto hash_results = hash.query(qpos);
            std::sort(hash_results.begin(), hash_results.end());

            // Naive 1-ring neighbor search
            const int qcx = static_cast<int>(std::floor(qpos.x / cell_size));
            const int qcy = static_cast<int>(std::floor(qpos.y / cell_size));
            const int qcz = static_cast<int>(std::floor(qpos.z / cell_size));

            std::vector<std::uint32_t> naive_results;
            for (std::size_t i = 0; i < entity_count; ++i) {
                const auto& p = positions[i];
                const int pcx = static_cast<int>(std::floor(p.x / cell_size));
                const int pcy = static_cast<int>(std::floor(p.y / cell_size));
                const int pcz = static_cast<int>(std::floor(p.z / cell_size));
                if (std::abs(pcx - qcx) <= 1 && std::abs(pcy - qcy) <= 1 && std::abs(pcz - qcz) <= 1) {
                    naive_results.push_back(static_cast<std::uint32_t>(i));
                }
            }
            std::sort(naive_results.begin(), naive_results.end());
            assert(hash_results == naive_results);
        }
    }

    // Advancement Verification: LinearBVH spatial indexing and logarithmic raycasting
    {
        using scandium::spatial::LinearBVH;
        LinearBVH bvh;
        assert(bvh.empty());
        assert(bvh.node_count() == 0);

        // 1. Single box
        bvh.build({Aabb{{0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}}});
        assert(!bvh.empty());
        assert(bvh.node_count() == 1);
        auto single_hits = bvh.query(Aabb{{0.5f, 0.5f, 0.5f}, {1.5f, 1.5f, 1.5f}});
        assert(single_hits.size() == 1);
        assert(single_hits[0] == 0);

        // 2. Multi-box hierarchy (100 boxes in 3D grid)
        std::vector<Aabb> scene_boxes;
        scene_boxes.reserve(100);
        for (int i = 0; i < 100; ++i) {
            const float x = static_cast<float>(i % 10) * 4.0f;
            const float z = static_cast<float>(i / 10) * 4.0f;
            scene_boxes.push_back(Aabb{{x, 0.0f, z}, {x + 2.0f, 2.0f, z + 2.0f}});
        }

        bvh.build(scene_boxes);
        assert(!bvh.empty());
        assert(bvh.node_count() > 100);

        // Test spatial query against naive
        const Aabb query_volume{{5.0f, 0.0f, 5.0f}, {15.0f, 2.0f, 15.0f}};
        auto bvh_results = bvh.query(query_volume);
        std::sort(bvh_results.begin(), bvh_results.end());

        std::vector<std::uint32_t> naive_bvh_results;
        for (std::size_t i = 0; i < scene_boxes.size(); ++i) {
            if (scene_boxes[i].intersects(query_volume)) {
                naive_bvh_results.push_back(static_cast<std::uint32_t>(i));
            }
        }
        std::sort(naive_bvh_results.begin(), naive_bvh_results.end());
        assert(bvh_results == naive_bvh_results);

        // Test raycast against naive
        const Ray test_ray{{-10.0f, 1.0f, 4.0f}, {1.0f, 0.0f, 0.0f}};
        const auto bvh_hit = bvh.raycast(test_ray);
        assert(bvh_hit.has_value());

        // Naive closest raycast
        std::optional<std::uint32_t> closest_naive_id;
        float closest_naive_dist = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < scene_boxes.size(); ++i) {
            const auto hit = intersect_ray_aabb(test_ray, scene_boxes[i]);
            if (hit.has_value() && *hit < closest_naive_dist) {
                closest_naive_dist = *hit;
                closest_naive_id = static_cast<std::uint32_t>(i);
            }
        }

        assert(closest_naive_id.has_value());
        assert(bvh_hit->primitive_id == *closest_naive_id);
        assert(std::fabs(bvh_hit->distance - closest_naive_dist) < 1e-5f);
    }

    test_simd_ray_intersection();
    test_continuous_collision_detection();
    (void)Vec3f{};
    return 0;
}
