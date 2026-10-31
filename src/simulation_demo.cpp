#include <cmath>
#include <iomanip>
#include <iostream>

#include "scandium/simulation/agent_world.hpp"
#include "scandium/simulation/simulation_clock.hpp"

namespace {
constexpr int width = 44;
constexpr int height = 22;

void render(const scandium::simulation::AgentWorld& world, float alpha) {
    char grid[height][width]{};

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            grid[y][x] = '.';
        }
    }

    // Render static and dynamic obstacles
    for (const auto& obs : world.obstacles()) {
        const int ox = static_cast<int>(std::lround(obs.position.x)) + width / 2;
        const int oy = static_cast<int>(std::lround(obs.position.z)) + height / 2;
        const int rad = static_cast<int>(std::lround(obs.radius));
        for (int dy = -rad; dy <= rad; ++dy) {
            for (int dx = -rad; dx <= rad; ++dx) {
                const int px = ox + dx;
                const int py = oy + dy;
                if (px >= 0 && px < width && py >= 0 && py < height && (dx * dx + dy * dy <= rad * rad)) {
                    grid[height - 1 - py][px] = '#';
                }
            }
        }
    }

    // Render agents
    int collisions = 0;
    for (const auto& agent : world.agents()) {
        const auto position = world.interpolated_position(agent, alpha);
        const int x = static_cast<int>(std::lround(position.x)) + width / 2;
        const int y = static_cast<int>(std::lround(position.z)) + height / 2;
        if (x >= 0 && x < width && y >= 0 && y < height) {
            const int gy = height - 1 - y;
            if (grid[gy][x] == 'A') {
                grid[gy][x] = '!'; // Proximity/overlap indicator
                ++collisions;
            } else if (grid[gy][x] != '#') {
                grid[gy][x] = agent.state == scandium::simulation::AgentState::Arrived ? '*' : 'A';
            }
        }
    }

    // Render target
    grid[height / 2][width / 2] = 'X';

    std::cout << "============================================\n";
    std::cout << " SCANDIUM MULTI-AGENT ASCII SIMULATION\n";
    std::cout << " X = Target | A = Agent | # = Obstacle | ! = Near\n";
    std::cout << "============================================\n";
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::cout << grid[y][x];
        }
        std::cout << '\n';
    }
}
} // namespace

int main() {
    using namespace scandium::simulation;

    AgentWorld world{3.0f};
    world.config().enable_collision_avoidance = true;
    world.config().enable_obstacle_avoidance = true;
    world.config().separation_radius = 2.0f;
    world.config().separation_weight = 1.8f;

    // Add obstacles in the arena
    world.add_obstacle({{-6.0f, 0.0f, 0.0f}, 1.5f});
    world.add_obstacle({{6.0f, 0.0f, 0.0f}, 1.5f});
    world.add_obstacle({{0.0f, 0.0f, 6.0f}, 1.2f});
    world.add_obstacle({{0.0f, 0.0f, -6.0f}, 1.2f});

    for (std::uint32_t id = 0; id < 32; ++id) {
        const float x = static_cast<float>(id % 8) * 4.0f - 14.0f;
        const float z = static_cast<float>(id / 8) * 4.0f - 6.0f;
        world.add_agent({id, {x, 0.0f, z}, {}, {}, AgentState::Idle, 4.0f, 0.6f});
    }

    SimulationClock clock{{1.0 / 30.0, 8}};
    const scandium::math::Vec3f target{0.0f, 0.0f, 0.0f};

    for (int frame = 0; frame < 30; ++frame) {
        clock.advance(1.0 / 30.0, [&](double dt) {
            world.update(static_cast<float>(dt), target);
        });

        if (frame % 5 == 0 || frame == 29) {
            render(world, static_cast<float>(clock.interpolation_alpha()));
            const auto stats = world.stats();
            std::cout << "Frame " << std::setw(2) << frame
                      << " | Tick: " << std::setw(3) << clock.total_steps()
                      << " | Agents: " << stats.agent_count
                      << " (Seek: " << stats.seek_count
                      << ", Idle: " << stats.idle_count << ")\n\n";
        }
    }

    return 0;
}
