#pragma once

#include <cstdint>

namespace scandium::ecs {

using EntityId = std::uint32_t;

struct Entity {
    EntityId id{};
    std::uint32_t generation{1};
    bool active{true};

    friend bool operator==(const Entity&, const Entity&) = default;
};

} // namespace scandium::ecs
