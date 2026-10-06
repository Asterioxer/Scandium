#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "scandium/simulation/agent_world.hpp"

namespace scandium::simulation {

inline std::uint64_t hash_agent_state(const AgentWorld& world) noexcept {
    std::uint64_t hash = 1469598103934665603ULL;

    const auto mix = [&hash](std::uint32_t value) {
        hash ^= value;
        hash *= 1099511628211ULL;
    };

    const auto quantize = [](float value) -> std::uint32_t {
        const auto scaled = static_cast<std::int64_t>(value * 10000.0f);
        return static_cast<std::uint32_t>(scaled);
    };

    for (const auto& agent : world.agents()) {
        mix(agent.id);
        mix(quantize(agent.position.x));
        mix(quantize(agent.position.y));
        mix(quantize(agent.position.z));
        mix(quantize(agent.velocity.x));
        mix(quantize(agent.velocity.y));
        mix(quantize(agent.velocity.z));
        mix(static_cast<std::uint32_t>(agent.state));
    }

    return hash;
}

} // namespace scandium::simulation
