#pragma once

#include <cstdint>

namespace scandium::ecs {

using EntityId = std::uint32_t;

struct Entity {
    EntityId id{};
    bool active{true};
};

} // namespace scandium::ecs
