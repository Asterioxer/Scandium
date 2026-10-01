#pragma once

#include <algorithm>

#include "scandium/math/vec3.hpp"

namespace scandium::physics {

struct Aabb {
    math::Vec3f min{};
    math::Vec3f max{};

    [[nodiscard]] constexpr math::Vec3f center() const noexcept {
        return (min + max) * 0.5f;
    }

    [[nodiscard]] constexpr math::Vec3f extent() const noexcept {
        return (max - min) * 0.5f;
    }

    [[nodiscard]] constexpr bool contains(const math::Vec3f& point) const noexcept {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    [[nodiscard]] constexpr bool intersects(const Aabb& other) const noexcept {
        return min.x <= other.max.x && max.x >= other.min.x &&
               min.y <= other.max.y && max.y >= other.min.y &&
               min.z <= other.max.z && max.z >= other.min.z;
    }
};

[[nodiscard]] constexpr Aabb merge(const Aabb& a, const Aabb& b) noexcept {
    return {
        {
            std::min(a.min.x, b.min.x),
            std::min(a.min.y, b.min.y),
            std::min(a.min.z, b.min.z)
        },
        {
            std::max(a.max.x, b.max.x),
            std::max(a.max.y, b.max.y),
            std::max(a.max.z, b.max.z)
        }
    };
}

} // namespace scandium::physics
