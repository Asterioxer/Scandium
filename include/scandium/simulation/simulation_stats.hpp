#pragma once

#include <cstddef>

namespace scandium::simulation {

struct SimulationStats {
    std::size_t agent_count{};
    std::size_t seek_count{};
    std::size_t idle_count{};
    std::size_t simulation_steps{};
    double fixed_delta_seconds{};
    double interpolation_alpha{};
};

} // namespace scandium::simulation
