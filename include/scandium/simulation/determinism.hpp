#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

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

struct StateMismatch {
    std::uint32_t agent_id{};
    std::string field_name;
    double expected_value{};
    double actual_value{};
    std::string details;
};

struct DiagnosticsReport {
    bool matches{true};
    std::size_t total_agents_compared{};
    std::vector<StateMismatch> mismatches;

    [[nodiscard]] std::string to_string() const {
        if (matches) {
            return "Simulation states match exactly (" +
                   std::to_string(total_agents_compared) + " agents compared).";
        }

        std::ostringstream ss;
        ss << "Simulation state mismatch detected: " << mismatches.size()
           << " difference(s) across " << total_agents_compared << " agents:\n";
        for (const auto& m : mismatches) {
            ss << "  [Agent " << m.agent_id << "] " << m.field_name
               << ": expected=" << m.expected_value
               << ", actual=" << m.actual_value;
            if (!m.details.empty()) {
                ss << " (" << m.details << ")";
            }
            ss << "\n";
        }
        return ss.str();
    }
};

inline DiagnosticsReport diagnose_state_mismatch(
    const AgentWorld& expected,
    const AgentWorld& actual,
    float tolerance = 1e-4f) {

    DiagnosticsReport report;
    const auto& exp_agents = expected.agents();
    const auto& act_agents = actual.agents();

    if (exp_agents.size() != act_agents.size()) {
        report.matches = false;
        report.mismatches.push_back({
            0,
            "agent_count",
            static_cast<double>(exp_agents.size()),
            static_cast<double>(act_agents.size()),
            "Agent counts do not match"
        });
        return report;
    }

    report.total_agents_compared = exp_agents.size();

    const auto check_field = [&](std::uint32_t id, const char* name,
                                 float exp_v, float act_v) {
        if (std::fabs(exp_v - act_v) > tolerance) {
            report.matches = false;
            report.mismatches.push_back({
                id,
                name,
                static_cast<double>(exp_v),
                static_cast<double>(act_v),
                "Delta: " + std::to_string(std::fabs(exp_v - act_v))
            });
        }
    };

    for (std::size_t i = 0; i < exp_agents.size(); ++i) {
        const auto& e = exp_agents[i];
        const auto& a = act_agents[i];

        if (e.id != a.id) {
            report.matches = false;
            report.mismatches.push_back({
                e.id, "id", static_cast<double>(e.id), static_cast<double>(a.id),
                "Agent ID mismatch at index " + std::to_string(i)
            });
        }

        if (e.state != a.state) {
            report.matches = false;
            report.mismatches.push_back({
                e.id, "state",
                static_cast<double>(static_cast<uint8_t>(e.state)),
                static_cast<double>(static_cast<uint8_t>(a.state)),
                "State enum mismatch"
            });
        }

        check_field(e.id, "position.x", e.position.x, a.position.x);
        check_field(e.id, "position.y", e.position.y, a.position.y);
        check_field(e.id, "position.z", e.position.z, a.position.z);

        check_field(e.id, "velocity.x", e.velocity.x, a.velocity.x);
        check_field(e.id, "velocity.y", e.velocity.y, a.velocity.y);
        check_field(e.id, "velocity.z", e.velocity.z, a.velocity.z);
    }

    return report;
}

} // namespace scandium::simulation
