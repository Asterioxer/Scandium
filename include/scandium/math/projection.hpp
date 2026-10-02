#pragma once

#include <cmath>
#include "scandium/math/mat4.hpp"

namespace scandium::math {

[[nodiscard]] inline Mat4 perspective(
    float vertical_fov_radians,
    float aspect_ratio,
    float near_plane,
    float far_plane) noexcept {

    const float f = 1.0f / std::tan(vertical_fov_radians * 0.5f);
    Mat4 result{};
    result(0, 0) = f / aspect_ratio;
    result(1, 1) = f;
    result(2, 2) = (far_plane + near_plane) / (near_plane - far_plane);
    result(2, 3) = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
    result(3, 2) = -1.0f;
    return result;
}

[[nodiscard]] inline Mat4 orthographic(
    float left, float right,
    float bottom, float top,
    float near_plane, float far_plane) noexcept {

    Mat4 result = Mat4::identity();
    result(0, 0) = 2.0f / (right - left);
    result(1, 1) = 2.0f / (top - bottom);
    result(2, 2) = -2.0f / (far_plane - near_plane);
    result(0, 3) = -(right + left) / (right - left);
    result(1, 3) = -(top + bottom) / (top - bottom);
    result(2, 3) = -(far_plane + near_plane) / (far_plane - near_plane);
    return result;
}

} // namespace scandium::math
