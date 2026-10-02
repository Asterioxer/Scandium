#pragma once

#include <cmath>
#include <limits>
#include <optional>

#include "scandium/math/ray.hpp"
#include "scandium/physics/aabb.hpp"

namespace scandium::physics {

[[nodiscard]] inline std::optional<float> intersect_ray_aabb(
    const math::Ray& ray,
    const Aabb& box,
    float max_distance = std::numeric_limits<float>::infinity()) noexcept {

    float t_min = 0.0f;
    float t_max = max_distance;

    const float origins[3] = {ray.origin.x, ray.origin.y, ray.origin.z};
    const float directions[3] = {ray.direction.x, ray.direction.y, ray.direction.z};
    const float mins[3] = {box.min.x, box.min.y, box.min.z};
    const float maxs[3] = {box.max.x, box.max.y, box.max.z};

    for (int axis = 0; axis < 3; ++axis) {
        if (std::fabs(directions[axis]) < 1e-7f) {
            if (origins[axis] < mins[axis] || origins[axis] > maxs[axis]) {
                return std::nullopt;
            }
            continue;
        }

        const float inv = 1.0f / directions[axis];
        float near_t = (mins[axis] - origins[axis]) * inv;
        float far_t = (maxs[axis] - origins[axis]) * inv;

        if (near_t > far_t) {
            std::swap(near_t, far_t);
        }

        t_min = std::max(t_min, near_t);
        t_max = std::min(t_max, far_t);

        if (t_min > t_max) {
            return std::nullopt;
        }
    }

    return t_min;
}

} // namespace scandium::physics
