#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "scandium/math/vec3.hpp"

namespace scandium::ai {

[[nodiscard]] inline math::Vec3f seek(
    const math::Vec3f& position,
    const math::Vec3f& target,
    float max_speed) noexcept {
    const math::Vec3f offset = target - position;
    const float dist_sq = offset.length_squared();
    if (dist_sq <= 1e-6f) {
        return {};
    }
    return offset.normalized() * max_speed;
}

[[nodiscard]] inline math::Vec3f flee(
    const math::Vec3f& position,
    const math::Vec3f& threat,
    float max_speed) noexcept {
    const math::Vec3f offset = position - threat;
    const float dist_sq = offset.length_squared();
    if (dist_sq <= 1e-6f) {
        return {};
    }
    return offset.normalized() * max_speed;
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

[[nodiscard]] inline math::Vec3f separation(
    const math::Vec3f& position,
    const std::vector<math::Vec3f>& neighbor_positions,
    float separation_radius,
    float max_speed) noexcept {

    math::Vec3f steer{};
    int count = 0;
    const float radius_sq = separation_radius * separation_radius;

    for (const auto& neighbor : neighbor_positions) {
        const math::Vec3f diff = position - neighbor;
        const float dist_sq = diff.length_squared();
        if (dist_sq > 1e-6f && dist_sq < radius_sq) {
            const float dist = std::sqrt(dist_sq);
            steer += (diff / dist) / dist; // inversely proportional to distance
            ++count;
        }
    }

    if (count > 0) {
        steer = (steer / static_cast<float>(count)).normalized() * max_speed;
    }

    return steer;
}

[[nodiscard]] inline math::Vec3f alignment(
    const math::Vec3f& current_velocity,
    const std::vector<math::Vec3f>& neighbor_velocities,
    float max_speed) noexcept {

    (void)current_velocity;
    if (neighbor_velocities.empty()) {
        return {};
    }

    math::Vec3f avg_vel{};
    for (const auto& v : neighbor_velocities) {
        avg_vel += v;
    }
    avg_vel = avg_vel / static_cast<float>(neighbor_velocities.size());

    if (avg_vel.length_squared() <= 1e-6f) {
        return {};
    }

    return avg_vel.normalized() * max_speed;
}

[[nodiscard]] inline math::Vec3f cohesion(
    const math::Vec3f& position,
    const std::vector<math::Vec3f>& neighbor_positions,
    float max_speed) noexcept {

    if (neighbor_positions.empty()) {
        return {};
    }

    math::Vec3f center{};
    for (const auto& p : neighbor_positions) {
        center += p;
    }
    center = center / static_cast<float>(neighbor_positions.size());

    return seek(position, center, max_speed);
}

struct SteeringWeights {
    float seek{1.0f};
    float arrive{1.0f};
    float separation{1.5f};
    float alignment{0.4f};
    float cohesion{0.3f};
    float obstacle_avoidance{2.0f};
};

} // namespace scandium::ai
