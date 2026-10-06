#pragma once

#include <algorithm>
#include <cstdint>

#include "scandium/core/event_bus.hpp"
#include "scandium/core/job_system.hpp"
#include "scandium/runtime/render_snapshot.hpp"
#include "scandium/simulation/agent_world.hpp"
#include "scandium/simulation/determinism.hpp"
#include "scandium/simulation/replay.hpp"
#include "scandium/simulation/simulation_clock.hpp"

namespace scandium::runtime {

struct SimulationTickEvent {
    std::uint64_t tick{};
    std::uint64_t state_hash{};
};

struct FrameAdvancedEvent {
    std::size_t simulation_steps{};
    float interpolation_alpha{};
};

class Engine {
public:
    explicit Engine(
        simulation::SimulationClock::Config clock_config = {},
        std::size_t worker_count = 0)
        : clock_(clock_config),
          jobs_(worker_count) {}

    simulation::AgentWorld& world() noexcept {
        return world_;
    }

    const simulation::AgentWorld& world() const noexcept {
        return world_;
    }

    core::EventBus& events() noexcept {
        return events_;
    }

    simulation::Replay& replay() noexcept {
        return replay_;
    }

    void set_target(math::Vec3f target) noexcept {
        target_ = target;
    }

    void record_input() {
        replay_.record({
            clock_.total_steps(),
            static_cast<std::int32_t>(target_.x),
            static_cast<std::int32_t>(target_.z)
        });
    }

    std::size_t advance(double frame_delta_seconds) {
        const auto steps = clock_.advance(frame_delta_seconds, [&](double dt) {
            world_.update_parallel(static_cast<float>(dt), target_, jobs_);
            events_.publish(SimulationTickEvent{
                clock_.total_steps() + 1,
                simulation::hash_agent_state(world_)
            });
        });

        events_.publish(FrameAdvancedEvent{
            steps,
            static_cast<float>(clock_.interpolation_alpha())
        });
        return steps;
    }

    [[nodiscard]] RenderSnapshot snapshot() const {
        RenderSnapshot result;
        result.simulation_tick = clock_.total_steps();
        result.interpolation_alpha =
            static_cast<float>(clock_.interpolation_alpha());
        result.agents.reserve(world_.agents().size());

        for (const auto& agent : world_.agents()) {
            result.agents.push_back({
                agent.id,
                world_.interpolated_position(agent, result.interpolation_alpha),
                agent.velocity
            });
        }
        return result;
    }

    [[nodiscard]] const simulation::SimulationClock& clock() const noexcept {
        return clock_;
    }

private:
    simulation::SimulationClock clock_;
    core::JobSystem jobs_;
    core::EventBus events_;
    simulation::Replay replay_;
    simulation::AgentWorld world_{4.0f};
    math::Vec3f target_{};
};

} // namespace scandium::runtime
