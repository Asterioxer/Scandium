#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "scandium/math/vec3.hpp"
#include "scandium/spatial/spatial_hash.hpp"

using Clock = std::chrono::steady_clock;
using scandium::math::Vec3f;
using scandium::spatial::SpatialHash;

// Prevent compiler elimination of computation
template <typename T>
void do_not_optimize(T&& val) {
    volatile auto sink = val;
    (void)sink;
}

double median(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t mid = v.size() / 2;
    return (v.size() % 2 == 0) ? (v[mid - 1] + v[mid]) * 0.5 : v[mid];
}

int main() {
    std::cout << "SCANDIUM SPATIAL HASH RIGOROUS BENCHMARK\n";
    std::cout << "========================================\n";
    std::cout << "entities,queries,warmup_trials,measured_trials,insert_median_ms,query_median_ms,total_ms,k_queries_sec\n";

    std::mt19937 rng{1337};
    std::uniform_real_distribution<float> dist{-100.0f, 100.0f};

    constexpr int warmup_trials = 2;
    constexpr int measured_trials = 5;
    constexpr std::size_t query_count = 50'000;

    for (const std::size_t count : {1000ULL, 5000ULL, 10000ULL, 25000ULL}) {
        std::vector<Vec3f> positions;
        positions.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            positions.push_back({dist(rng), dist(rng), dist(rng)});
        }

        std::vector<double> insert_times;
        std::vector<double> query_times;
        insert_times.reserve(measured_trials);
        query_times.reserve(measured_trials);

        std::size_t global_sink = 0;

        for (int trial = 0; trial < warmup_trials + measured_trials; ++trial) {
            SpatialHash hash{5.0f, count * 2};

            // Insertion phase
            const auto t_insert_start = Clock::now();
            for (std::size_t i = 0; i < count; ++i) {
                hash.insert(static_cast<std::uint32_t>(i), positions[i]);
            }
            const auto t_insert_end = Clock::now();
            const double insert_ms = std::chrono::duration<double, std::milli>(t_insert_end - t_insert_start).count();

            // Query phase (50,000 queries)
            std::size_t total_hits = 0;
            const auto t_query_start = Clock::now();
            for (std::size_t q = 0; q < query_count; ++q) {
                const auto& qpos = positions[q % count];
                hash.query(qpos, [&](std::uint32_t id) {
                    total_hits += id;
                });
            }
            const auto t_query_end = Clock::now();
            const double query_ms = std::chrono::duration<double, std::milli>(t_query_end - t_query_start).count();

            do_not_optimize(total_hits);
            global_sink += total_hits;

            if (trial >= warmup_trials) {
                insert_times.push_back(insert_ms);
                query_times.push_back(query_ms);
            }
        }

        const double ins_med = median(insert_times);
        const double qry_med = median(query_times);
        const double tot_ms = ins_med + qry_med;
        const double safe_qry_ms = std::max(qry_med, 1e-6);
        const double k_qps = (static_cast<double>(query_count) / (safe_qry_ms * 1e-3)) / 1000.0;

        std::cout << count << ","
                  << query_count << ","
                  << warmup_trials << ","
                  << measured_trials << ","
                  << std::fixed << std::setprecision(4)
                  << ins_med << ","
                  << qry_med << ","
                  << tot_ms << ","
                  << std::setprecision(1) << k_qps << "\n";

        do_not_optimize(global_sink);
    }

    return 0;
}
