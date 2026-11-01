#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "scandium/ai/simd_steering.hpp"
#include "scandium/core/job_system.hpp"
#include "scandium/navigation/astar.hpp"
#include "scandium/physics/aabb.hpp"
#include "scandium/physics/ray_intersection.hpp"
#include "scandium/profiling/benchmark_harness.hpp"
#include "scandium/simulation/compute_pipeline.hpp"
#include "scandium/spatial/linear_bvh.hpp"

using Clock = std::chrono::steady_clock;
using scandium::bench::compute_stats;
using scandium::bench::do_not_optimize;

void benchmark_lbvh_vs_naive_raycast() {
    std::cout << "\n=======================================================\n";
    std::cout << "  SCANDIUM ADVANCEMENTS: LINEAR BVH VS NAIVE RAYCAST   \n";
    std::cout << "=======================================================\n";

    constexpr std::size_t num_primitives = 2000;
    constexpr std::size_t num_rays = 5000;

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> pos_dist(-100.0f, 100.0f);
    std::uniform_real_distribution<float> size_dist(0.5f, 2.0f);

    std::vector<scandium::physics::Aabb> boxes;
    boxes.reserve(num_primitives);
    for (std::size_t i = 0; i < num_primitives; ++i) {
        const float x = pos_dist(rng);
        const float y = pos_dist(rng);
        const float z = pos_dist(rng);
        const float s = size_dist(rng);
        boxes.push_back(scandium::physics::Aabb{
            {x - s, y - s, z - s},
            {x + s, y + s, z + s}
        });
    }

    // Build Linear BVH
    scandium::spatial::LinearBVH bvh;
    const auto t_build_start = Clock::now();
    bvh.build(boxes);
    const auto t_build_end = Clock::now();
    const double build_ms = std::chrono::duration<double, std::milli>(t_build_end - t_build_start).count();
    std::cout << "LBVH Build Time (" << num_primitives << " boxes): " << std::fixed << std::setprecision(3) << build_ms << " ms\n";

    // Generate random rays
    std::vector<scandium::math::Ray> rays;
    rays.reserve(num_rays);
    std::uniform_real_distribution<float> dir_dist(-1.0f, 1.0f);
    for (std::size_t i = 0; i < num_rays; ++i) {
        scandium::math::Vec3f orig{pos_dist(rng), pos_dist(rng), pos_dist(rng)};
        scandium::math::Vec3f dir{dir_dist(rng), dir_dist(rng), dir_dist(rng)};
        if (dir.length_squared() < 1e-4f) dir = {0.0f, 0.0f, 1.0f};
        rays.push_back({orig, dir.normalized()});
    }

    // 1. Naive O(N) Raycasting
    const auto t_naive_start = Clock::now();
    std::size_t naive_hits = 0;
    for (const auto& ray : rays) {
        float closest_t = 1000.0f;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const auto hit_t = scandium::physics::intersect_ray_aabb(ray, boxes[i], closest_t);
            if (hit_t.has_value() && *hit_t < closest_t) {
                closest_t = *hit_t;
            }
        }
        if (closest_t < 1000.0f) ++naive_hits;
        do_not_optimize(closest_t);
    }
    const auto t_naive_end = Clock::now();
    const double naive_ms = std::chrono::duration<double, std::milli>(t_naive_end - t_naive_start).count();

    // 2. LBVH O(log N) Raycasting
    const auto t_bvh_start = Clock::now();
    std::size_t bvh_hits = 0;
    for (const auto& ray : rays) {
        const auto hit = bvh.raycast(ray, 1000.0f);
        if (hit.has_value()) ++bvh_hits;
        do_not_optimize(hit.has_value());
    }
    const auto t_bvh_end = Clock::now();
    const double bvh_ms = std::chrono::duration<double, std::milli>(t_bvh_end - t_bvh_start).count();

    const double speedup = naive_ms / std::max(bvh_ms, 1e-6);
    std::cout << "Naive O(N) Raycast: " << naive_ms << " ms (" << naive_hits << " hits, "
              << static_cast<std::size_t>(num_rays / (naive_ms / 1000.0)) << " rays/sec)\n";
    std::cout << "LBVH O(log N) Raycast: " << bvh_ms << " ms (" << bvh_hits << " hits, "
              << static_cast<std::size_t>(num_rays / (bvh_ms / 1000.0)) << " rays/sec)\n";
    std::cout << "Speedup: " << std::setprecision(2) << speedup << "x\n";
}

void benchmark_eight_way_navigation() {
    std::cout << "\n=======================================================\n";
    std::cout << "  SCANDIUM ADVANCEMENTS: 8-WAY DIAGONAL VS 4-WAY A*    \n";
    std::cout << "=======================================================\n";

    scandium::navigation::Grid grid(80, 80);
    const scandium::navigation::GridNode start{5, 5};
    const scandium::navigation::GridNode goal{75, 75};

    constexpr int iters = 200;

    // 4-Way Search (Manhattan)
    const auto t4_start = Clock::now();
    scandium::navigation::SearchMetrics m4;
    for (int i = 0; i < iters; ++i) {
        const auto res = scandium::navigation::AStar::find_path_with_metrics(
            grid, start, goal, scandium::navigation::GridConnectivity::FourWay
        );
        m4 = res.metrics;
        do_not_optimize(res.path.size());
    }
    const auto t4_end = Clock::now();
    const double ms_4way = std::chrono::duration<double, std::milli>(t4_end - t4_start).count() / iters;

    // 8-Way Search (Octile)
    const auto t8_start = Clock::now();
    scandium::navigation::SearchMetrics m8;
    for (int i = 0; i < iters; ++i) {
        const auto res = scandium::navigation::AStar::find_path_with_metrics(
            grid, start, goal, scandium::navigation::GridConnectivity::EightWay
        );
        m8 = res.metrics;
        do_not_optimize(res.path.size());
    }
    const auto t8_end = Clock::now();
    const double ms_8way = std::chrono::duration<double, std::milli>(t8_end - t8_start).count() / iters;

    const float cost_reduction = (1.0f - m8.path_cost / m4.path_cost) * 100.0f;
    std::cout << "4-Way Cardinal Path: cost = " << std::fixed << std::setprecision(2) << m4.path_cost
              << ", nodes = " << m4.nodes_explored << ", time = " << ms_4way << " ms\n";
    std::cout << "8-Way Diagonal Path: cost = " << std::fixed << std::setprecision(2) << m8.path_cost
              << ", nodes = " << m8.nodes_explored << ", time = " << ms_8way << " ms\n";
    std::cout << "Path Cost Reduction: " << std::setprecision(1) << cost_reduction << "%\n";
}

void benchmark_simd_steering_separation() {
    std::cout << "\n=======================================================\n";
    std::cout << "  SCANDIUM ADVANCEMENTS: SIMD SoA VS SCALAR AoS STEERING\n";
    std::cout << "=======================================================\n";

    constexpr std::size_t num_neighbors = 1000;
    constexpr int repetitions = 50000;

    scandium::ai::AgentSoA soa;
    std::vector<scandium::math::Vec3f> aos;
    soa.reserve(num_neighbors);
    aos.reserve(num_neighbors);

    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> offset_dist(-3.0f, 3.0f);

    for (std::size_t i = 0; i < num_neighbors; ++i) {
        const float x = offset_dist(rng);
        const float y = 0.0f;
        const float z = offset_dist(rng);
        soa.push_back(x, y, z);
        aos.push_back({x, y, z});
    }

    const scandium::math::Vec3f agent_pos{0.0f, 0.0f, 0.0f};
    const float radius = 2.5f;
    const float max_speed = 5.0f;

    // 1. Scalar AoS
    const auto t_aos_start = Clock::now();
    scandium::math::Vec3f force_aos{};
    for (int r = 0; r < repetitions; ++r) {
        force_aos = scandium::ai::compute_simd_separation_aos(
            agent_pos, aos.data(), aos.size(), radius, max_speed
        );
        do_not_optimize(force_aos.x);
    }
    const auto t_aos_end = Clock::now();
    const double ms_aos = std::chrono::duration<double, std::milli>(t_aos_end - t_aos_start).count();

    // 2. SIMD SSE SoA
    const auto t_soa_start = Clock::now();
    scandium::math::Vec3f force_soa{};
    for (int r = 0; r < repetitions; ++r) {
        force_soa = scandium::ai::compute_simd_separation_soa(
            agent_pos.x, agent_pos.y, agent_pos.z,
            soa.x.data(), soa.y.data(), soa.z.data(),
            soa.size(), radius, max_speed
        );
        do_not_optimize(force_soa.x);
    }
    const auto t_soa_end = Clock::now();
    const double ms_soa = std::chrono::duration<double, std::milli>(t_soa_end - t_soa_start).count();

    const double speedup = ms_aos / std::max(ms_soa, 1e-6);
    std::cout << "Scalar AoS Separation (" << repetitions << " reps): " << std::fixed << std::setprecision(2) << ms_aos << " ms\n";
    std::cout << "SIMD SoA Separation   (" << repetitions << " reps): " << std::fixed << std::setprecision(2) << ms_soa << " ms\n";
    std::cout << "Speedup: " << std::setprecision(2) << speedup << "x\n";
}

void benchmark_batch_compute_pipeline() {
    std::cout << "\n=======================================================\n";
    std::cout << "  SCANDIUM ADVANCEMENTS: BATCH COMPUTE PIPELINE         \n";
    std::cout << "=======================================================\n";

    constexpr std::size_t agent_count = 10000;
    constexpr int steps = 50;

    scandium::simulation::BatchSimulationPipeline pipeline;
    std::vector<scandium::simulation::Agent> agents;
    agents.reserve(agent_count);

    std::mt19937 rng(999);
    std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);

    for (std::size_t i = 0; i < agent_count; ++i) {
        agents.push_back({
            static_cast<std::uint32_t>(i),
            {pos_dist(rng), 0.0f, pos_dist(rng)},
            {},
            {},
            scandium::simulation::AgentState::Idle,
            5.0f,
            0.5f,
            {0.0f, 0.0f, 0.0f}
        });
    }

    pipeline.upload(agents);

    scandium::simulation::GPUSimulationParams params;
    params.dt = 1.0f / 60.0f;
    params.agent_count = static_cast<std::uint32_t>(agent_count);
    params.enable_separation = 1;

    scandium::core::JobSystem jobs(8);

    const auto t_start = Clock::now();
    for (int s = 0; s < steps; ++s) {
        pipeline.step(params, &jobs);
    }
    const auto t_end = Clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    const double ms_per_step = total_ms / steps;
    const double throughput = (agent_count * steps) / (total_ms / 1000.0);

    std::cout << "Stepped " << agent_count << " agents x " << steps << " frames across 8 workers\n";
    std::cout << "Time per step: " << std::fixed << std::setprecision(2) << ms_per_step << " ms\n";
    std::cout << "Throughput:    " << std::setprecision(1) << (throughput / 1'000'000.0) << " Million agent-steps / sec\n";
    std::cout << "=======================================================\n";
}


#include "scandium/navigation/flow_field.hpp"

void benchmark_flow_field_crowd_navigation() {
    std::cout << "\n=======================================================\n";
    std::cout << "  SCANDIUM ADVANCEMENTS: FLOW FIELD CROWD NAVIGATION   \n";
    std::cout << "=======================================================\n";

    constexpr int width = 100;
    constexpr int height = 100;
    scandium::navigation::FlowField flow(width, height, 1.0f);

    // Add maze walls
    for (int y = 20; y < 80; y += 15) {
        for (int x = 10; x < 90; ++x) {
            if (x % 12 != 0) {
                flow.set_blocked(x, y, true);
            }
        }
    }

    // 1. Measure Flow Field generation time for 10,000 cells
    const auto t_gen_start = Clock::now();
    constexpr int gen_reps = 20;
    for (int r = 0; r < gen_reps; ++r) {
        flow.compute({{50, 50}});
    }
    const auto t_gen_end = Clock::now();
    const double gen_ms = std::chrono::duration<double, std::milli>(t_gen_end - t_gen_start).count() / gen_reps;

    std::cout << "Flow Field Generation (10,000 cells): " << std::fixed << std::setprecision(2) << gen_ms << " ms\n";

    // 2. Measure continuous bilinear sampling throughput for 100,000 agents
    constexpr std::size_t num_agents = 100000;
    std::vector<scandium::math::Vec3f> agent_positions;
    agent_positions.reserve(num_agents);

    std::mt19937 rng(777);
    std::uniform_real_distribution<float> pos_dist(0.0f, 99.0f);
    for (std::size_t i = 0; i < num_agents; ++i) {
        agent_positions.push_back({pos_dist(rng), 0.0f, pos_dist(rng)});
    }

    const auto t_sample_start = Clock::now();
    std::size_t valid_vectors = 0;
    for (const auto& pos : agent_positions) {
        const auto vec = flow.sample(pos);
        if (vec.length_squared() > 1e-4f) ++valid_vectors;
        do_not_optimize(vec.x);
    }
    const auto t_sample_end = Clock::now();
    const double sample_ms = std::chrono::duration<double, std::milli>(t_sample_end - t_sample_start).count();
    const double samples_per_sec = (num_agents / (sample_ms / 1000.0));

    std::cout << "Sampled " << num_agents << " agent flow vectors in " << sample_ms << " ms\n";
    std::cout << "Sampling Throughput: " << std::setprecision(2) << (samples_per_sec / 1'000'000.0) << " Million agents/sec (O(1) per agent)\n";
    std::cout << "=======================================================\n";
}

int main() {
    benchmark_lbvh_vs_naive_raycast();
    benchmark_eight_way_navigation();
    benchmark_simd_steering_separation();
    benchmark_batch_compute_pipeline();
    benchmark_flow_field_crowd_navigation();
    return 0;
}
