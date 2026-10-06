#include <cassert>
#include <cmath>

#include "scandium/profiling/frame_stats.hpp"
#include "scandium/simulation/agent_world.hpp"
#include "scandium/simulation/simulation_clock.hpp"

int main() {
    using namespace scandium::simulation;

    AgentWorld world{5.0f};
    world.add_agent({1, {0.0f, 0.0f, 0.0f}});
    world.add_agent({2, {20.0f, 0.0f, 0.0f}});

    world.update(0.1f, {10.0f, 0.0f, 0.0f});

    assert(world.agents().size() == 2);
    assert(world.agents()[0].position.x > 0.0f);
    assert(world.agents()[0].state == AgentState::Seek);

    const auto simulation_stats = world.stats();
    assert(simulation_stats.agent_count == 2);
    assert(simulation_stats.seek_count == 2);
    assert(simulation_stats.idle_count == 0);

    SimulationClock clock{{1.0 / 60.0, 8}};
    std::size_t steps = 0;
    const auto first_frame_steps = clock.advance(1.0 / 30.0, [&](double dt) {
        assert(std::abs(dt - (1.0 / 60.0)) < 1e-12);
        ++steps;
    });
    assert(first_frame_steps == 2);
    assert(steps == 2);
    assert(clock.total_steps() == 2);
    assert(clock.interpolation_alpha() < 1e-9);

    const auto stalled_frame_steps = clock.advance(1.0, [&](double) {
        ++steps;
    });
    assert(stalled_frame_steps == 8);
    assert(clock.total_steps() == 10);
    assert(clock.interpolation_alpha() >= 0.0);
    assert(clock.interpolation_alpha() < 1.0);

    clock.reset();
    assert(clock.total_steps() == 0);
    assert(clock.accumulated_seconds() == 0.0);

    scandium::profiling::FrameStats stats;
    stats.record(16.0);
    stats.record(20.0);
    assert(stats.frame_count == 2);
    assert(stats.average_milliseconds() == 18.0);
    assert(stats.max_milliseconds == 20.0);

    return 0;
}
