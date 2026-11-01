#include <cassert>
#include <cmath>

#include "scandium/profiling/frame_stats.hpp"
#include "scandium/simulation/agent_world.hpp"
#include "scandium/simulation/simulation_clock.hpp"


#include "scandium/simulation/compute_pipeline.hpp"

void test_batch_simulation_pipeline() {
    using namespace scandium::simulation;

    BatchSimulationPipeline pipeline;
    std::vector<Agent> initial_agents;
    for (std::size_t i = 0; i < 128; ++i) {
        initial_agents.push_back({
            static_cast<std::uint32_t>(i),
            {static_cast<float>(i), 0.0f, 0.0f},
            {},
            {},
            AgentState::Idle,
            5.0f,
            0.5f,
            {50.0f, 0.0f, 0.0f}
        });
    }

    pipeline.upload(initial_agents);
    assert(pipeline.agent_count() == 128);

    GPUSimulationParams params;
    params.dt = 1.0f / 60.0f;
    params.agent_count = 128;
    params.enable_separation = 1;

    // Step pipeline
    scandium::core::JobSystem jobs(2);
    pipeline.step(params, &jobs);

    std::vector<Agent> stepped_agents;
    pipeline.download(stepped_agents);
    assert(stepped_agents.size() == 128);

    // Agent 0 started at 0.0, target is 50.0, max_speed 5.0 -> should have moved forward
    assert(stepped_agents[0].position.x > 0.0f);
    assert(stepped_agents[0].state == AgentState::Seek);

    // Verify HLSL compute shader export
    const auto hlsl = BatchSimulationPipeline::get_hlsl_compute_shader();
    assert(!hlsl.empty());
    assert(hlsl.find("CSMain") != std::string_view::npos);
    assert(hlsl.find("numthreads(64, 1, 1)") != std::string_view::npos);
}

int main() {
    test_batch_simulation_pipeline();
    using namespace scandium::simulation;

    AgentWorld world{5.0f};
    world.add_agent({1, {0.0f, 0.0f, 0.0f}});
    world.add_agent({2, {20.0f, 0.0f, 0.0f}});

    world.update(0.1f, {10.0f, 0.0f, 0.0f});

    assert(world.agents().size() == 2);
    assert(world.agents()[0].position.x > 0.0f);
    assert(world.agents()[0].state == AgentState::Seek);
    const auto interpolated = world.interpolated_position(world.agents()[0], 0.5f);
    assert(interpolated.x > 0.0f);
    assert(interpolated.x < world.agents()[0].position.x);

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

    SimulationClock invalid_clock{{0.0, 0}};
    assert(std::abs(invalid_clock.fixed_delta_seconds() - (1.0 / 60.0)) < 1e-12);
    assert(invalid_clock.advance(1.0 / 60.0, [&](double) {}) == 1);

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
