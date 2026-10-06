#pragma once

#include "scandium/math/mat4.hpp"
#include "scandium/math/quaternion.hpp"

namespace scandium::math {

struct Transform {
    Vec3f position{};
    Quaternion rotation{Quaternion::identity()};
    Vec3f scale{1.0f, 1.0f, 1.0f};

    [[nodiscard]] Mat4 matrix() const noexcept {
        return Mat4::translation(position) *
               Mat4::rotation(rotation.normalized()) *
               Mat4::scale(scale);
    }

    [[nodiscard]] Vec3f transform_point(const Vec3f& point) const noexcept {
        return matrix().transform_point(point);
    }
};

} // namespace scandium::math
