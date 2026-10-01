#pragma once

#include <array>
#include <cstddef>
#include <cmath>

#include "scandium/math/vec3.hpp"

namespace scandium::math {

struct Mat4 {
    std::array<float, 16> m{};

    static constexpr Mat4 identity() noexcept {
        Mat4 result{};
        result(0, 0) = 1.0f;
        result(1, 1) = 1.0f;
        result(2, 2) = 1.0f;
        result(3, 3) = 1.0f;
        return result;
    }

    constexpr float& operator()(std::size_t row, std::size_t column) noexcept {
        return m[column * 4 + row];
    }

    constexpr float operator()(std::size_t row, std::size_t column) const noexcept {
        return m[column * 4 + row];
    }

    static constexpr Mat4 translation(const Vec3f& position) noexcept {
        Mat4 result = identity();
        result(0, 3) = position.x;
        result(1, 3) = position.y;
        result(2, 3) = position.z;
        return result;
    }

    static constexpr Mat4 scale(const Vec3f& factors) noexcept {
        Mat4 result = identity();
        result(0, 0) = factors.x;
        result(1, 1) = factors.y;
        result(2, 2) = factors.z;
        return result;
    }

    constexpr Mat4 operator*(const Mat4& rhs) const noexcept {
        Mat4 result{};
        for (std::size_t row = 0; row < 4; ++row) {
            for (std::size_t column = 0; column < 4; ++column) {
                float value = 0.0f;
                for (std::size_t k = 0; k < 4; ++k) {
                    value += (*this)(row, k) * rhs(k, column);
                }
                result(row, column) = value;
            }
        }
        return result;
    }

    constexpr Vec3f transform_point(const Vec3f& point) const noexcept {
        const float x = (*this)(0, 0) * point.x + (*this)(0, 1) * point.y +
                        (*this)(0, 2) * point.z + (*this)(0, 3);
        const float y = (*this)(1, 0) * point.x + (*this)(1, 1) * point.y +
                        (*this)(1, 2) * point.z + (*this)(1, 3);
        const float z = (*this)(2, 0) * point.x + (*this)(2, 1) * point.y +
                        (*this)(2, 2) * point.z + (*this)(2, 3);
        const float w = (*this)(3, 0) * point.x + (*this)(3, 1) * point.y +
                        (*this)(3, 2) * point.z + (*this)(3, 3);

        if (std::fabs(w) > 1e-6f && std::fabs(w - 1.0f) > 1e-6f) {
            return {x / w, y / w, z / w};
        }
        return {x, y, z};
    }
};

} // namespace scandium::math
