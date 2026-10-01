#pragma once

#include <cmath>
#include <concepts>

namespace scandium::math {

template <std::floating_point T>
struct Vec3 {
    T x{};
    T y{};
    T z{};

    constexpr Vec3() = default;
    constexpr Vec3(T x_value, T y_value, T z_value)
        : x(x_value), y(y_value), z(z_value) {}

    constexpr Vec3 operator+(const Vec3& rhs) const noexcept {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    constexpr Vec3 operator-(const Vec3& rhs) const noexcept {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    constexpr Vec3 operator*(T scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }

    constexpr Vec3 operator/(T scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    constexpr Vec3& operator+=(const Vec3& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    constexpr T dot(const Vec3& rhs) const noexcept {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    constexpr Vec3 cross(const Vec3& rhs) const noexcept {
        return {
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        };
    }

    constexpr T length_squared() const noexcept {
        return dot(*this);
    }

    T length() const noexcept {
        return std::sqrt(length_squared());
    }

    Vec3 normalized() const {
        const T magnitude = length();
        if (magnitude == T{}) {
            return {};
        }
        return *this / magnitude;
    }
};

using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;

} // namespace scandium::math
