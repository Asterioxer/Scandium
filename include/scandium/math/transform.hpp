#pragma once

#include "scandium/math/mat4.hpp"

namespace scandium::math {

struct Transform {
    Vec3f position{};
    Vec3f scale{1.0f, 1.0f, 1.0f};

    [[nodiscard]] constexpr Mat4 matrix() const noexcept {
        return Mat4::translation(position) * Mat4::scale(scale);
    }

    [[nodiscard]] constexpr Vec3f transform_point(const Vec3f& point) const noexcept {
        return matrix().transform_point(point);
    }
};

} // namespace scandium::math
