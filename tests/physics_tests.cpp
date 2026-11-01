#include <cassert>
#include <cmath>

#include "scandium/math/quaternion.hpp"
#include "scandium/math/ray.hpp"
#include "scandium/physics/ray_intersection.hpp"
#include "scandium/spatial/spatial_hash.hpp"

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

    (void)Vec3f{};
    return 0;
}
