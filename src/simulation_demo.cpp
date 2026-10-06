#include <cmath>
#include <iomanip>
#include <iostream>

#include "scandium/simulation/agent_world.hpp"
#include "scandium/simulation/simulation_clock.hpp"

namespace {
constexpr int width = 40;
constexpr int height = 20;

void render(const scandium::simulation::AgentWorld& world, float alpha) {
    char grid[height][width]{};

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            grid[y][x] = '.';
        }
    }

    for (const auto& agent : world.agents()) {
        const auto position = world.interpolated_position(agent, alpha);
        const int x = static_cast<int>(std::lround(position.x)) + width / 2;
        const int y = static_cast<int>(std::lround(position.z)) + height / 2;
        if (x >= 0 && x < width && y >= 0 && y < height) {
            grid[height - 1 - y][x] = 'A';
        }
    }

    grid[height / 2][width / 2] = 'X';

    std::cout << "SCANDIUM ASCII SIMULATION\n";
    std::cout << "X = target, A = agent\n\n";
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::cout << grid[y][x];
        }
        std::cout << '\n';
    }
}
}

int main() {
    using namespace scandium::simulation;

    AgentWorld world{3.0f};
    for (std::uint32_t id = 0; id < 32; ++id) {
        const float x = static_cast<float>(id % 8) * 4.0f - 14.0f;
        const float z = static_cast<float>(id / 8) * 4.0f - 6.0f;
        world.add_agent({id, {x, 0.0f, z}});
    }

    SimulationClock clock{{1.0 / 30.0, 8}};
    const scandium::math::Vec3f target{0.0f, 0.0f, 0.0f};

    for (int frame = 0; frame < 30; ++frame) {
        clock.advance(1.0 / 30.0, [&](double dt) {
            world.update(static_cast<float>(dt), target);
        });

        if (frame % 5 == 0 || frame == 29) {
            render(world, static_cast<float>(clock.interpolation_alpha()));
            std::cout << "Frame " << std::setw(2) << frame
                      << " | steps " << clock.total_steps()
                      << " | agents " << world.stats().agent_count << "\n\n";
        }
    }

    return 0;
}
