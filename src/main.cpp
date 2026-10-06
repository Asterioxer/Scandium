#include <iomanip>
#include <iostream>

#include "scandium/math/vec3.hpp"
#include "scandium/simulation/agent_world.hpp"
#include "scandium/simulation/simulation_clock.hpp"

int main() {
    using scandium::math::Vec3f;
    using scandium::simulation::Agent;
    using scandium::simulation::AgentState;
    using scandium::simulation::AgentWorld;
    using scandium::simulation::SimulationClock;

    AgentWorld world{5.0f};

    constexpr std::size_t agent_count = 128;
    for (std::size_t i = 0; i < agent_count; ++i) {
        const float x = static_cast<float>(i % 16) * 3.0f - 22.5f;
        const float z = static_cast<float>(i / 16) * 3.0f - 10.5f;
        world.add_agent(Agent{
            static_cast<std::uint32_t>(i),
            {x, 0.0f, z},
            {},
            AgentState::Idle,
            4.0f
        });
    }

    SimulationClock clock{{1.0 / 60.0, 8}};
    const Vec3f target{0.0f, 0.0f, 0.0f};

    std::size_t simulated_steps = 0;
    for (int frame = 0; frame < 120; ++frame) {
        simulated_steps += clock.advance(1.0 / 60.0, [&](double dt) {
            world.update(static_cast<float>(dt), target);
        });
    }

    const auto stats = world.stats();
    const auto nearby = world.nearby(target);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "SCANDIUM SIMULATION RUNTIME\n";
    std::cout << "---------------------------\n";
    std::cout << "Agents:             " << world.agents().size() << '\n';
    std::cout << "Fixed timestep:     " << clock.fixed_delta_seconds() << " s\n";
    std::cout << "Render frames:      120\n";
    std::cout << "Simulation steps:   " << simulated_steps << '\n';
    std::cout << "Seek agents:        " << seeking << '\n';
    std::cout << "Idle agents:        " << idle << '\n';
    std::cout << "Nearby target cell: " << nearby.size() << " agents\n";
    std::cout << "Interpolation alpha: " << clock.interpolation_alpha() << '\n';
    std::cout << '\n';

    const Vec3f forward{0.0f, 0.0f, 1.0f};
    const Vec3f right{1.0f, 0.0f, 0.0f};
    const auto normal = forward.cross(right);

    std::cout << "MATH SMOKE TEST\n";
    std::cout << "dot(forward, right) = " << forward.dot(right) << '\n';
    std::cout << "cross(forward, right) = ("
              << normal.x << ", "
              << normal.y << ", "
              << normal.z << ")\n";
    std::cout << "normalized(3,4,0) length = "
              << Vec3f{3.0f, 4.0f, 0.0f}.normalized().length() << '\n';

    return 0;
}
