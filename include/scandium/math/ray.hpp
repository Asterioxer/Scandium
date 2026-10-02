#pragma once

#include "scandium/math/vec3.hpp"

namespace scandium::math {

struct Ray {
    Vec3f origin{};
    Vec3f direction{0.0f, 0.0f, 1.0f};

    [[nodiscard]] constexpr Vec3f at(float distance) const noexcept {
        return origin + direction * distance;
    }
};

} // namespace scandium::math
