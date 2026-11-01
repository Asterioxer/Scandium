#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "scandium/math/vec3.hpp"
#include "scandium/physics/aabb.hpp"
#include "scandium/spatial/spatial_hash.hpp"
#include "scandium/profiling/benchmark_harness.hpp"

using scandium::bench::compute_stats;
using scandium::bench::do_not_optimize;
using scandium::math::Vec3f;
using scandium::physics::Aabb;
using scandium::spatial::SpatialHash;
using scandium::spatial::SpatialHashDiagnostics;
using Clock = std::chrono::steady_clock;

enum class DistributionType {
    Uniform,
    Clustered,
    HighlyDense,
    Sparse,
    Adversarial
};

std::string dist_name(DistributionType d) {
    switch (d) {
        case DistributionType::Uniform: return "Uniform";
        case DistributionType::Clustered: return "Clustered";
        case DistributionType::HighlyDense: return "HighlyDense";
        case DistributionType::Sparse: return "Sparse";
        case DistributionType::Adversarial: return "Adversarial";
    }
    return "Unknown";
}

std::vector<Vec3f> generate_points(DistributionType dist, std::size_t count, std::uint32_t seed = 42) {
    std::mt19937 rng{seed};
    std::vector<Vec3f> pts;
    pts.reserve(count);

    if (dist == DistributionType::Uniform) {
        std::uniform_real_distribution<float> d{-100.0f, 100.0f};
        for (std::size_t i = 0; i < count; ++i) {
            pts.push_back({d(rng), d(rng), d(rng)});
        }
    } else if (dist == DistributionType::Clustered) {
        constexpr int num_clusters = 10;
        std::uniform_real_distribution<float> center_dist{-80.0f, 80.0f};
        std::vector<Vec3f> centers;
        centers.reserve(num_clusters);
        for (int c = 0; c < num_clusters; ++c) {
            centers.push_back({center_dist(rng), center_dist(rng), center_dist(rng)});
        }
        std::normal_distribution<float> cluster_offset{0.0f, 5.0f}; // stddev matches cell size
        std::uniform_int_distribution<int> c_pick{0, num_clusters - 1};
        for (std::size_t i = 0; i < count; ++i) {
            const auto& center = centers[c_pick(rng)];
            pts.push_back({center.x + cluster_offset(rng),
                           center.y + cluster_offset(rng),
                           center.z + cluster_offset(rng)});
        }
    } else if (dist == DistributionType::HighlyDense) {
        // High density in a tight box [-5, 5]^3 (cell_size=5 means 8 cells total)
        std::uniform_real_distribution<float> d{-5.0f, 5.0f};
        for (std::size_t i = 0; i < count; ++i) {
            pts.push_back({d(rng), d(rng), d(rng)});
        }
    } else if (dist == DistributionType::Sparse) {
        std::uniform_real_distribution<float> d{-1000.0f, 1000.0f};
        for (std::size_t i = 0; i < count; ++i) {
            pts.push_back({d(rng), d(rng), d(rng)});
        }
    } else if (dist == DistributionType::Adversarial) {
        // All points concentrated along a single coordinate line (x in [0, 50], y=0, z=0)
        // or aliasing cell hashes mod bucket_mask
        std::uniform_real_distribution<float> d{0.0f, 50.0f};
        for (std::size_t i = 0; i < count; ++i) {
            pts.push_back({d(rng), 0.0f, 0.0f});
        }
    }

    return pts;
}

// Naive broadphase verification
std::vector<std::uint32_t> naive_query(const std::vector<Vec3f>& positions,
                                       const Vec3f& query_pos, float cell_size) {
    const float range_x = cell_size;
    const float range_y = cell_size;
    const float range_z = cell_size;

    // cell contains query_pos; neighbor cells are within 1 cell in each axis
    const int qcx = static_cast<int>(std::floor(query_pos.x / cell_size));
    const int qcy = static_cast<int>(std::floor(query_pos.y / cell_size));
    const int qcz = static_cast<int>(std::floor(query_pos.z / cell_size));

    std::vector<std::uint32_t> results;
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const auto& p = positions[i];
        const int pcx = static_cast<int>(std::floor(p.x / cell_size));
        const int pcy = static_cast<int>(std::floor(p.y / cell_size));
        const int pcz = static_cast<int>(std::floor(p.z / cell_size));
        if (std::abs(pcx - qcx) <= 1 && std::abs(pcy - qcy) <= 1 && std::abs(pcz - qcz) <= 1) {
            results.push_back(static_cast<std::uint32_t>(i));
        }
    }
    std::sort(results.begin(), results.end());
    return results;
}

void profile_spatial_hash_distribution(DistributionType dist, std::size_t entity_count, std::size_t query_count) {
    const auto positions = generate_points(dist, entity_count);
    const float cell_size = 5.0f;

    // Correctness verification against naive for 100 sample points
    {
        SpatialHash test_hash{cell_size, entity_count * 2};
        for (std::size_t i = 0; i < entity_count; ++i) {
            test_hash.insert(static_cast<std::uint32_t>(i), positions[i]);
        }
        for (std::size_t q = 0; q < std::min<std::size_t>(100, entity_count); ++q) {
            auto hash_res = test_hash.query(positions[q]);
            std::sort(hash_res.begin(), hash_res.end());
            const auto naive_res = naive_query(positions, positions[q], cell_size);
            if (hash_res != naive_res) {
                std::cerr << "CORRECTNESS MISMATCH in distribution " << dist_name(dist)
                          << " at query " << q << ": hash found " << hash_res.size()
                          << ", naive found " << naive_res.size() << "\n";
                std::abort();
            }
        }
    }

    constexpr int warmup_trials = 2;
    constexpr int measured_trials = 5;

    std::vector<double> insert_times;
    insert_times.reserve(measured_trials);

    SpatialHash final_hash{cell_size, entity_count * 2};

    for (int t = 0; t < warmup_trials + measured_trials; ++t) {
        SpatialHash h{cell_size, entity_count * 2};
        const auto t0 = Clock::now();
        for (std::size_t i = 0; i < entity_count; ++i) {
            h.insert(static_cast<std::uint32_t>(i), positions[i]);
        }
        const auto t1 = Clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (t >= warmup_trials) {
            insert_times.push_back(ms);
        }
        if (t == warmup_trials + measured_trials - 1) {
            final_hash = std::move(h);
        }
    }

    const auto diag = final_hash.diagnostics();

    // Query profiling: measure individual query latency distribution across query_count queries
    std::vector<double> query_latencies_us;
    query_latencies_us.reserve(query_count);

    std::size_t total_candidates = 0;
    std::size_t max_candidates = 0;

    for (std::size_t q = 0; q < query_count; ++q) {
        const auto& qpos = positions[q % entity_count];
        std::size_t query_hits = 0;

        const auto q_start = Clock::now();
        final_hash.query(qpos, [&](std::uint32_t id) {
            ++query_hits;
            do_not_optimize(id);
        });
        const auto q_end = Clock::now();
        const double q_us = std::chrono::duration<double, std::micro>(q_end - q_start).count();
        query_latencies_us.push_back(q_us);

        total_candidates += query_hits;
        if (query_hits > max_candidates) {
            max_candidates = query_hits;
        }
    }

    const auto insert_stats = compute_stats(insert_times);
    const auto q_stats = compute_stats(query_latencies_us);
    const double avg_candidates = query_count > 0 ? static_cast<double>(total_candidates) / query_count : 0.0;
    const double total_query_ms = (q_stats.mean_ms * query_count) / 1000.0;
    const double k_qps = total_query_ms > 0.0 ? (static_cast<double>(query_count) / total_query_ms) : 0.0;

    std::cout << dist_name(dist) << ","
              << entity_count << ","
              << diag.total_buckets << ","
              << diag.occupied_buckets << ","
              << std::fixed << std::setprecision(1) << diag.empty_bucket_pct << "%,"
              << std::setprecision(2) << diag.avg_bucket_occupancy << ","
              << diag.max_bucket_occupancy << ","
              << diag.hash_collisions << ","
              << (diag.memory_bytes / 1024) << "KB,"
              << std::setprecision(2) << avg_candidates << ","
              << max_candidates << ","
              << std::setprecision(3) << insert_stats.median_ms << ","
              << std::setprecision(2) << q_stats.median_ms << "us,"
              << std::setprecision(2) << q_stats.p90_ms << "us,"
              << std::setprecision(2) << q_stats.p99_ms << "us,"
              << std::setprecision(1) << k_qps << "\n";
}

int main() {
    std::cout << "SCANDIUM SPATIAL HASH DEEP PROFILING MATRIX\n";
    std::cout << "===========================================\n";
    std::cout << "distribution,entities,buckets,occupied,empty_pct,avg_occupancy,max_occupancy,hash_collisions,memory,avg_candidates,max_candidates,insert_median_ms,query_p50,query_p90,query_p99,k_qps\n";

    for (const auto dist : {DistributionType::Uniform,
                            DistributionType::Clustered,
                            DistributionType::HighlyDense,
                            DistributionType::Sparse,
                            DistributionType::Adversarial}) {
        for (const std::size_t count : {1000ULL, 5000ULL, 10000ULL, 25000ULL}) {
            profile_spatial_hash_distribution(dist, count, 20000);
        }
    }

    return 0;
}
