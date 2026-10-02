#pragma once

#include "scandium/math/vec3.hpp"

namespace scandium::ai {

[[nodiscard]] inline math::Vec3f seek(
    const math::Vec3f& position,
    const math::Vec3f& target,
    float max_speed) noexcept {
    return (target - position).normalized() * max_speed;
}

[[nodiscard]] inline math::Vec3f flee(
    const math::Vec3f& position,
    const math::Vec3f& threat,
    float max_speed) noexcept {
    return (position - threat).normalized() * max_speed;
}

[[nodiscard]] inline math::Vec3f arrive(
    const math::Vec3f& position,
    const math::Vec3f& target,
    float max_speed,
    float slowing_radius) noexcept {

    const math::Vec3f offset = target - position;
    const float distance = offset.length();
    if (distance <= 1e-5f) {
        return {};
    }

    const float speed = distance < slowing_radius
        ? max_speed * (distance / slowing_radius)
        : max_speed;

    return offset.normalized() * speed;
}

} // namespace scandium::ai
