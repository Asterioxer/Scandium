#pragma once

#include <cstdint>
#include <vector>

#include "scandium/math/vec3.hpp"

namespace scandium::runtime {

struct RenderAgent {
    std::uint32_t id{};
    math::Vec3f position{};
    math::Vec3f velocity{};
};

struct RenderSnapshot {
    std::uint64_t simulation_tick{};
    float interpolation_alpha{};
    std::vector<RenderAgent> agents;
};

} // namespace scandium::runtime
