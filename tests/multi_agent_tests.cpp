#include <cassert>
#include <cmath>
#include <iostream>

#include "scandium/simulation/agent_world.hpp"

using namespace scandium;
using namespace scandium::simulation;

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
        w.config().min_agents_per_chunk = 64;
        for (std::size_t i = 0; i < count; ++i) {
            const float x = static_cast<float>(i % 20) * 2.0f - 19.0f;
            const float z = static_cast<float>(i / 20) * 2.0f - 14.0f;
            w.add_agent({
                static_cast<std::uint32_t>(i),
                {x, 0.0f, z}, {}, {},
                AgentState::Idle, 4.0f, 0.5f
            });
        }
        return w;
    };

    // Sequential reference run
    auto seq_world = make_world(true);
    for (int f = 0; f < frames; ++f) {
        seq_world.update(1.0f / 60.0f, {0.0f, 0.0f, 10.0f});
    }

    // Parallel runs across 1, 2, 4, 8, 16 workers
    for (const std::size_t workers : {1ULL, 2ULL, 4ULL, 8ULL, 16ULL}) {
        core::JobSystem jobs(workers);
        auto par_world = make_world(true);
        for (int f = 0; f < frames; ++f) {
            par_world.update_parallel(1.0f / 60.0f, {0.0f, 0.0f, 10.0f}, jobs);
        }

        assert(par_world.agents().size() == seq_world.agents().size());
        for (std::size_t i = 0; i < count; ++i) {
            const auto& a_seq = seq_world.agents()[i];
            const auto& a_par = par_world.agents()[i];
            assert(a_seq.position.x == a_par.position.x);
            assert(a_seq.position.y == a_par.position.y);
            assert(a_seq.position.z == a_par.position.z);
            assert(a_seq.velocity.x == a_par.velocity.x);
            assert(a_seq.velocity.y == a_par.velocity.y);
            assert(a_seq.velocity.z == a_par.velocity.z);
            assert(a_seq.state == a_par.state);
        }
    }
    std::cout << "Passed test_parallel_vs_sequential_simulation_equivalence." << std::endl;
}


#include "scandium/ai/simd_steering.hpp"

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
    test_simd_steering_soa();
    test_agent_collision_avoidance();
    test_grid_navigation_and_waypoints();
    test_dynamic_obstacle_avoidance();
    test_parallel_vs_sequential_simulation_equivalence();
    std::cout << "All multi-agent simulation tests passed successfully.\n";
    return 0;
}
