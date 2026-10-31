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

int main() {
    test_agent_collision_avoidance();
    test_grid_navigation_and_waypoints();
    test_dynamic_obstacle_avoidance();
    std::cout << "All multi-agent simulation tests passed successfully.\n";
    return 0;
}
