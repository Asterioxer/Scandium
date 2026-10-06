#include <iomanip>
#include <iostream>

#include "scandium/math/vec3.hpp"
#include "scandium/profiling/trace.hpp"
#include "scandium/runtime/engine.hpp"
#include "scandium/simulation/determinism.hpp"

int main() {
    using namespace scandium;
    using math::Vec3f;

    runtime::Engine engine{{1.0 / 60.0, 8}};
    constexpr std::size_t agent_count = 512;

    for (std::size_t i = 0; i < agent_count; ++i) {
        const float x = static_cast<float>(i % 32) * 2.0f - 31.0f;
        const float z = static_cast<float>(i / 32) * 2.0f - 15.0f;
        engine.world().add_agent({
            static_cast<std::uint32_t>(i),
            {x, 0.0f, z},
            {},
            {},
            simulation::AgentState::Idle,
            4.0f
        });
    }

    profiling::TraceCollector trace;
    std::uint64_t last_hash = 0;
    engine.events().subscribe<runtime::SimulationTickEvent>(
        [&](const runtime::SimulationTickEvent& event) {
            last_hash = event.state_hash;
        });

    engine.set_target({0.0f, 0.0f, 0.0f});
    engine.record_input();

    for (int frame = 0; frame < 120; ++frame) {
        profiling::TraceScope scope(trace, "simulation_frame");
        engine.advance(1.0 / 60.0);
    }

    const auto stats = engine.world().stats();
    const auto nearby = engine.world().nearby({0.0f, 0.0f, 0.0f});
    const auto snapshot = engine.snapshot();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "SCANDIUM FLAGSHIP RUNTIME 0.2\n";
    std::cout << "============================\n";
    std::cout << "Agents:              " << engine.world().agents().size() << '\n';
    std::cout << "Workers:             " << engine.worker_count() << '\n';
    std::cout << "Fixed timestep:      " << engine.clock().fixed_delta_seconds() << " s\n";
    std::cout << "Render frames:       120\n";
    std::cout << "Simulation ticks:    " << engine.clock().total_steps() << '\n';
    std::cout << "Seek agents:         " << stats.seek_count << '\n';
    std::cout << "Idle agents:         " << stats.idle_count << '\n';
    std::cout << "Nearby target cell:  " << nearby.size() << " agents\n";
    std::cout << "Interpolation alpha: " << snapshot.interpolation_alpha << '\n';
    std::cout << "State hash:          " << last_hash << '\n';
    std::cout << "Trace events:        " << trace.events().size() << '\n';
    std::cout << "Replay commands:     " << engine.replay().size() << '\n';
    std::cout << '\n';

    const Vec3f forward{0.0f, 0.0f, 1.0f};
    const Vec3f right{1.0f, 0.0f, 0.0f};
    const auto normal = forward.cross(right);

    std::cout << "MATH SMOKE TEST\n";
    std::cout << "dot(forward, right) = " << forward.dot(right) << '\n';
    std::cout << "cross(forward, right) = ("
              << normal.x << ", " << normal.y << ", " << normal.z << ")\n";
    std::cout << "normalized(3,4,0) length = "
              << Vec3f{3.0f, 4.0f, 0.0f}.normalized().length() << '\n';

    return 0;
}
