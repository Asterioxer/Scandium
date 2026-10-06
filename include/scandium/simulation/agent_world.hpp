#pragma once

#include <cstdint>
#include <vector>

#include "scandium/math/vec3.hpp"
#include "scandium/simulation/simulation_stats.hpp"
#include "scandium/spatial/spatial_hash.hpp"

namespace scandium::simulation {

enum class AgentState : std::uint8_t {
    Idle,
    Seek,
    Flee
};

struct Agent {
    std::uint32_t id{};
    math::Vec3f position{};
    math::Vec3f previous_position{};
    math::Vec3f velocity{};
    AgentState state{AgentState::Idle};
    float max_speed{5.0f};
};

class AgentWorld {
public:
    explicit AgentWorld(float cell_size) : spatial_hash_(cell_size) {}

    void add_agent(Agent agent) {
        agents_.push_back(agent);
    }

    void update(float dt, const math::Vec3f& target) {
        spatial_hash_.clear();

        for (const auto& agent : agents_) {
            spatial_hash_.insert(agent.id, agent.position);
        }

        for (auto& agent : agents_) {
            agent.previous_position = agent.position;
            const math::Vec3f delta = target - agent.position;
            const float distance_sq = delta.length_squared();

            agent.state = distance_sq < 4.0f ? AgentState::Idle : AgentState::Seek;

            if (agent.state == AgentState::Seek) {
                agent.velocity = delta.normalized() * agent.max_speed;
                agent.position += agent.velocity * dt;
            } else {
                agent.velocity = {};
            }
        }
    }

    [[nodiscard]] const std::vector<Agent>& agents() const noexcept {
        return agents_;
    }

    [[nodiscard]] math::Vec3f interpolated_position(
        const Agent& agent, float alpha) const noexcept {
        return agent.previous_position +
               (agent.position - agent.previous_position) * alpha;
    }

    [[nodiscard]] std::vector<std::uint32_t> nearby(
        const math::Vec3f& position) const {
        return spatial_hash_.query(position);
    }

    [[nodiscard]] SimulationStats stats() const noexcept {
        SimulationStats result;
        result.agent_count = agents_.size();

        for (const auto& agent : agents_) {
            if (agent.state == AgentState::Seek) {
                ++result.seek_count;
            } else if (agent.state == AgentState::Idle) {
                ++result.idle_count;
            }
        }

        return result;
    }

private:
    std::vector<Agent> agents_;
    spatial::SpatialHash spatial_hash_;
};

} // namespace scandium::simulation
