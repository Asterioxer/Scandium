#include <cassert>
#include <cmath>
#include <iostream>

#include "scandium/core/cpu_features.hpp"
#include "scandium/ai/simd_steering.hpp"
#include "scandium/simulation/agent_world.hpp"

using namespace scandium;
using namespace scandium::simulation;

void test_cpu_features_and_dynamic_simd_dispatch() {
    std::cout << "Starting test_cpu_features_and_dynamic_simd_dispatch..." << std::endl;
    const auto& info = core::CPUFeatures::get();
    assert(!info.vendor.empty());
    std::cout << "Detected CPU Vendor: " << info.vendor << "\n";
    if (!info.brand.empty()) {
        std::cout << "Detected CPU Brand:  " << info.brand << "\n";
    }
    std::cout << "CPU Features: SSE=" << info.sse
              << ", SSE2=" << info.sse2
              << ", SSE4.1=" << info.sse4_1
              << ", AVX=" << info.avx
              << ", AVX2=" << info.avx2
              << ", AVX512F=" << info.avx512f
              << ", NEON=" << info.neon << "\n";

    // Differential validation across all SIMD instruction kernels
    ai::AgentSoA soa;
    constexpr std::size_t count = 127; // Non-multiple of 8 and 4 to stress vector + scalar tails
    for (std::size_t i = 0; i < count; ++i) {
        const float a = static_cast<float>(i) * 0.15f;
        const float r = 0.5f + static_cast<float>(i % 7) * 0.25f;
        soa.push_back(std::cos(a) * r, 0.0f, std::sin(a) * r);
    }

    const float px = 0.2f, py = 0.0f, pz = 0.2f;
    const float radius = 3.0f;
    const float max_speed = 5.0f;

    const math::Vec3f f_scalar = ai::compute_simd_separation_soa(
        px, py, pz, soa.x.data(), soa.y.data(), soa.z.data(), count, radius, max_speed, ai::SIMDInstructionSet::Scalar);
    const math::Vec3f f_sse = ai::compute_simd_separation_soa(
        px, py, pz, soa.x.data(), soa.y.data(), soa.z.data(), count, radius, max_speed, ai::SIMDInstructionSet::SSE);
    const math::Vec3f f_avx2 = ai::compute_simd_separation_soa(
        px, py, pz, soa.x.data(), soa.y.data(), soa.z.data(), count, radius, max_speed, ai::SIMDInstructionSet::AVX2);
    const math::Vec3f f_auto = ai::compute_simd_separation_soa(
        px, py, pz, soa.x.data(), soa.y.data(), soa.z.data(), count, radius, max_speed, ai::SIMDInstructionSet::Auto);

    assert(f_scalar.length() > 0.0f);
    assert(std::fabs((f_scalar - f_sse).length()) < 1e-3f);
    assert(std::fabs((f_scalar - f_avx2).length()) < 1e-3f);
    assert(std::fabs((f_scalar - f_auto).length()) < 1e-3f);

    std::cout << "Passed test_cpu_features_and_dynamic_simd_dispatch." << std::endl;
}

void test_agent_collision_avoidance() {
    AgentWorld world{4.0f};
    world.config().enable_collision_avoidance = true;
    world.config().separation_radius = 2.0f;
    world.config().separation_weight = 2.0f;

    // Two agents placed very close together
    world.add_agent({0, {0.0f, 0.0f, 0.0f}, {}, {}, AgentState::Idle, 4.0f, 0.5f});
    world.add_agent({1, {0.2f, 0.0f, 0.0f}, {}, {}, AgentState::Idle, 4.0f, 0.5f});

    // Update towards a target point ahead
    const math::Vec3f target{0.0f, 0.0f, 20.0f};
    for (int i = 0; i < 30; ++i) {
        world.update(1.0f / 60.0f, target);
    }

    const auto& a0 = world.agents()[0];
    const auto& a1 = world.agents()[1];

    const float separation = (a0.position - a1.position).length();
    // After 30 ticks of avoidance, separation should have expanded beyond the initial 0.2f
    assert(separation > 0.25f);
}

void test_grid_navigation_and_waypoints() {
    AgentWorld world{2.0f};

    navigation::Grid grid{20, 20};
    // Wall from y = 3 to 17 at x = 10, leaving a door at (10, 10)
    for (int y = 3; y <= 17; ++y) {
        if (y != 10) {
            grid.set_blocked({10, y}, true);
        }
    }

    world.set_navigation_grid(grid);

    // Agent starts at (2, 0, 10) and wants to reach (18, 0, 10)
    world.add_agent({0, {2.0f, 0.0f, 10.0f}, {}, {}, AgentState::Idle, 6.0f, 0.5f});

    const bool path_found = world.navigate_agent_to(0, {18.0f, 0.0f, 10.0f}, false);
    assert(path_found);

    const auto& agent_initial = world.agents()[0];
    assert(agent_initial.has_active_path);
    assert(agent_initial.state == AgentState::Navigating);
    assert(!agent_initial.waypoints.empty());

    // Advance simulation
    for (int frame = 0; frame < 300; ++frame) {
        world.update(1.0f / 60.0f, {18.0f, 0.0f, 10.0f});
        if (world.agents()[0].state == AgentState::Arrived) {
            break;
        }
    }

    const auto& agent_final = world.agents()[0];
    // Agent should have reached destination across the door
    assert(agent_final.state == AgentState::Arrived);
    assert(agent_final.position.x > 15.0f);
}

void test_dynamic_obstacle_avoidance() {
    AgentWorld world{4.0f};
    world.config().enable_obstacle_avoidance = true;

    // Obstacle at (5, 0, 0) with radius 2.0
    world.add_obstacle({{5.0f, 0.0f, 0.0f}, 2.0f});

    // Agent starting at (0, 0, 0) targeting (10, 0, 0)
    world.add_agent({0, {0.0f, 0.0f, 0.0f}, {}, {}, AgentState::Idle, 4.0f, 0.5f});

    for (int i = 0; i < 120; ++i) {
        world.update(1.0f / 60.0f, {10.0f, 0.0f, 0.0f});
    }

    const auto& agent = world.agents()[0];
    // Agent should not collide with obstacle center
    const float dist_to_obstacle = (agent.position - math::Vec3f{5.0f, 0.0f, 0.0f}).length();
    assert(dist_to_obstacle > 0.8f);
}

void test_parallel_vs_sequential_simulation_equivalence() {
    std::cout << "Starting test_parallel_vs_sequential_simulation_equivalence..." << std::endl;
    constexpr std::size_t count = 300;
    constexpr int frames = 30;

    auto make_world = [](bool avoid) {
        AgentWorld w{4.0f};
        w.config().enable_collision_avoidance = avoid;
        w.config().parallel_threshold = 100;
        for (std::size_t i = 0; i < count; ++i) {
            const float angle = static_cast<float>(i) * 0.1f;
            const float r = 10.0f + static_cast<float>(i % 10) * 2.0f;
            w.add_agent({
                static_cast<std::uint32_t>(i),
                {std::cos(angle) * r, 0.0f, std::sin(angle) * r},
                {},
                {},
                AgentState::Idle,
                5.0f,
                0.5f
            });
        }
        return w;
    };

    AgentWorld world_seq = make_world(true);
    AgentWorld world_par = make_world(true);
    core::JobSystem jobs{4};

    const math::Vec3f target{0.0f, 0.0f, 0.0f};

    for (int f = 0; f < frames; ++f) {
        world_seq.update(1.0f / 60.0f, target);
        world_par.update_parallel(1.0f / 60.0f, target, jobs);

        for (std::size_t i = 0; i < count; ++i) {
            const auto& a_seq = world_seq.agents()[i];
            const auto& a_par = world_par.agents()[i];
            const float pos_diff = (a_seq.position - a_par.position).length();
            assert(pos_diff < 1e-4f);
        }
    }
    std::cout << "Passed test_parallel_vs_sequential_simulation_equivalence." << std::endl;
}

void test_simd_steering_soa() {
    std::cout << "Starting test_simd_steering_soa..." << std::endl;
    ai::AgentSoA soa;
    constexpr std::size_t count = 35; // non-power of 2, non-multiple of 4 to test SIMD + scalar tail
    soa.reserve(count);

    // Place neighbors in a small cluster around (0, 0, 0)
    for (std::size_t i = 0; i < count; ++i) {
        const float angle = static_cast<float>(i) * 0.2f;
        const float dist = 0.5f + static_cast<float>(i % 5) * 0.3f;
        soa.push_back(std::cos(angle) * dist, 0.0f, std::sin(angle) * dist);
    }
    assert(soa.size() == count);

    // Reference agent at (0.1, 0, 0.1)
    const float px = 0.1f;
    const float py = 0.0f;
    const float pz = 0.1f;
    const float radius = 2.5f;
    const float max_speed = 6.0f;

    // Test SIMD separation force computation
    const math::Vec3f force = ai::compute_simd_separation_soa(
        px, py, pz,
        soa.x.data(), soa.y.data(), soa.z.data(),
        soa.size(),
        radius, max_speed
    );

    // Force should be directed away from cluster centroid and have length approximately equal to max_speed
    assert(force.length() > 0.0f);
    assert(std::abs(force.length() - max_speed) < 1e-4f);

    // Test distance calculation batch
    std::vector<float> distances(count, 0.0f);
    ai::simd_compute_distances_soa(
        px, py, pz,
        soa.x.data(), soa.y.data(), soa.z.data(),
        distances.data(),
        count
    );

    for (std::size_t i = 0; i < count; ++i) {
        const float dx = px - soa.x[i];
        const float dy = py - soa.y[i];
        const float dz = pz - soa.z[i];
        const float expected_dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        assert(std::abs(distances[i] - expected_dist) < 1e-4f);
    }

    // Test AoS fallback equivalence
    std::vector<math::Vec3f> aos_neighbors;
    aos_neighbors.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        aos_neighbors.push_back(soa.get_position(i));
    }

    const math::Vec3f aos_force = ai::compute_simd_separation_aos(
        {px, py, pz}, aos_neighbors.data(), aos_neighbors.size(), radius, max_speed
    );
    assert(std::abs(aos_force.x - force.x) < 1e-3f);
    assert(std::abs(aos_force.y - force.y) < 1e-3f);
    assert(std::abs(aos_force.z - force.z) < 1e-3f);

    std::cout << "Passed test_simd_steering_soa." << std::endl;
}

int main() {
    test_cpu_features_and_dynamic_simd_dispatch();
    test_simd_steering_soa();
    test_agent_collision_avoidance();
    test_grid_navigation_and_waypoints();
    test_dynamic_obstacle_avoidance();
    test_parallel_vs_sequential_simulation_equivalence();
    std::cout << "All multi-agent simulation tests passed successfully.\n";
    return 0;
}
