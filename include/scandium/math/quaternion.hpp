#pragma once

#include <cmath>

#include "scandium/math/vec3.hpp"

namespace scandium::math {

struct Quaternion {
    float w{1.0f};
    float x{};
    float y{};
    float z{};

    static constexpr Quaternion identity() noexcept { return {}; }

    static Quaternion from_axis_angle(Vec3f axis, float radians) {
        const float half = radians * 0.5f;
        const float s = std::sin(half);
        axis = axis.normalized();
        return {std::cos(half), axis.x * s, axis.y * s, axis.z * s};
    }

    constexpr Quaternion conjugate() const noexcept {
        return {w, -x, -y, -z};
    }

    constexpr Quaternion operator*(const Quaternion& rhs) const noexcept {
        return {
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
        };
    }

    Vec3f rotate(Vec3f point) const noexcept {
        const Quaternion p{0.0f, point.x, point.y, point.z};
        const Quaternion rotated = (*this) * p * conjugate();
        return {rotated.x, rotated.y, rotated.z};
    }
};

} // namespace scandium::math
