#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "scandium/math/ray.hpp"
#include "scandium/math/vec3.hpp"
#include "scandium/physics/aabb.hpp"

namespace scandium::physics {

struct Contact {
    bool has_contact{false};
    float toi{1.0f};           // Time of impact in [0, 1] interval along trajectory
    math::Vec3f point{};       // World-space contact position
    math::Vec3f normal{};      // Outward contact normal (pointing towards primary moving body)
    float penetration{0.0f};   // Initial penetration depth if already overlapping at t = 0
};

/**
 * Continuous collision test between two moving spheres.
 * Solves exact quadratic equation for earliest contact time t in [0, 1].
 */
[[nodiscard]] inline Contact swept_sphere_vs_sphere(
    const math::Vec3f& pos_a, const math::Vec3f& vel_a, float radius_a,
    const math::Vec3f& pos_b, const math::Vec3f& vel_b, float radius_b,
    float dt) noexcept {

    Contact result;
    const math::Vec3f delta_a = vel_a * dt;
    const math::Vec3f delta_b = vel_b * dt;
    const math::Vec3f s = delta_a - delta_b; // Relative displacement of A with respect to B
    const math::Vec3f d = pos_a - pos_b;     // Relative initial position
    const float r = radius_a + radius_b;
    const float r_sq = r * r;

    const float d_sq = d.length_squared();

    // Already overlapping at t = 0
    if (d_sq < r_sq) {
        result.has_contact = true;
        result.toi = 0.0f;
        const float dist = std::sqrt(std::max(d_sq, 1e-8f));
        result.normal = (dist > 1e-4f) ? (d / dist) : math::Vec3f{0.0f, 1.0f, 0.0f};
        result.penetration = r - dist;
        result.point = pos_b + result.normal * radius_b;
        return result;
    }

    const float a = s.length_squared();
    if (a < 1e-8f) {
        // No relative movement
        return result;
    }

    const float b = 2.0f * d.dot(s);
    // If moving apart, no collision can happen
    if (b >= 0.0f) {
        return result;
    }

    const float c = d_sq - r_sq;
    const float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f) {
        // Trajectories do not intersect
        return result;
    }

    const float sqrt_d = std::sqrt(discriminant);
    const float t = (-b - sqrt_d) / (2.0f * a);

    if (t >= 0.0f && t <= 1.0f) {
        result.has_contact = true;
        result.toi = t;
        const math::Vec3f pos_a_at_t = pos_a + delta_a * t;
        const math::Vec3f pos_b_at_t = pos_b + delta_b * t;
        const math::Vec3f diff = pos_a_at_t - pos_b_at_t;
        const float dist = diff.length();
        result.normal = (dist > 1e-4f) ? (diff / dist) : math::Vec3f{0.0f, 1.0f, 0.0f};
        result.point = pos_b_at_t + result.normal * radius_b;
        result.penetration = 0.0f;
    }

    return result;
}

/**
 * Continuous collision test between a moving sphere and a stationary AABB.
 * Employs Minkowski sum expansion and ray marching to find exact time of impact in [0, 1].
 */
[[nodiscard]] inline Contact swept_sphere_vs_aabb(
    const math::Vec3f& sphere_pos,
    const math::Vec3f& sphere_vel,
    float sphere_radius,
    const Aabb& box,
    float dt) noexcept {

    Contact result;
    const math::Vec3f disp = sphere_vel * dt;

    // Check if initially overlapping at t = 0
    const math::Vec3f clamped_init{
        std::clamp(sphere_pos.x, box.min.x, box.max.x),
        std::clamp(sphere_pos.y, box.min.y, box.max.y),
        std::clamp(sphere_pos.z, box.min.z, box.max.z)
    };
    const math::Vec3f init_diff = sphere_pos - clamped_init;
    const float init_dist_sq = init_diff.length_squared();

    if (init_dist_sq < sphere_radius * sphere_radius) {
        result.has_contact = true;
        result.toi = 0.0f;
        const float d = std::sqrt(std::max(init_dist_sq, 1e-8f));
        result.normal = (d > 1e-4f) ? (init_diff / d) : math::Vec3f{0.0f, 1.0f, 0.0f};
        result.penetration = sphere_radius - d;
        result.point = clamped_init;
        return result;
    }

    if (disp.length_squared() < 1e-8f) {
        return result;
    }

    // Minkowski expanded AABB
    const Aabb expanded{
        {box.min.x - sphere_radius, box.min.y - sphere_radius, box.min.z - sphere_radius},
        {box.max.x + sphere_radius, box.max.y + sphere_radius, box.max.z + sphere_radius}
    };

    // Conservative ray test against expanded box
    float t_min = 0.0f;
    float t_max = 1.0f;

    const float origins[3] = {sphere_pos.x, sphere_pos.y, sphere_pos.z};
    const float directions[3] = {disp.x, disp.y, disp.z};
    const float mins[3] = {expanded.min.x, expanded.min.y, expanded.min.z};
    const float maxs[3] = {expanded.max.x, expanded.max.y, expanded.max.z};

    int hit_axis = -1;
    float hit_dir = 0.0f;

    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(directions[axis]) < 1e-7f) {
            if (origins[axis] < mins[axis] || origins[axis] > maxs[axis]) {
                return result;
            }
            continue;
        }

        const float inv_d = 1.0f / directions[axis];
        float t1 = (mins[axis] - origins[axis]) * inv_d;
        float t2 = (maxs[axis] - origins[axis]) * inv_d;
        float face_dir = -1.0f;

        if (t1 > t2) {
            std::swap(t1, t2);
            face_dir = 1.0f;
        }

        if (t1 > t_min) {
            t_min = t1;
            hit_axis = axis;
            hit_dir = face_dir;
        }

        t_max = std::min(t_max, t2);
        if (t_min > t_max) {
            return result;
        }
    }

    if (t_min >= 0.0f && t_min <= 1.0f && hit_axis >= 0) {
        // Refine contact on rounded corner/face
        const math::Vec3f contact_pos = sphere_pos + disp * t_min;
        const math::Vec3f closest_box_pt{
            std::clamp(contact_pos.x, box.min.x, box.max.x),
            std::clamp(contact_pos.y, box.min.y, box.max.y),
            std::clamp(contact_pos.z, box.min.z, box.max.z)
        };

        const math::Vec3f diff = contact_pos - closest_box_pt;
        const float dist = diff.length();

        result.has_contact = true;
        result.toi = t_min;
        result.point = closest_box_pt;

        if (dist > 1e-4f) {
            result.normal = diff / dist;
        } else {
            math::Vec3f n{};
            if (hit_axis == 0) n.x = hit_dir;
            else if (hit_axis == 1) n.y = hit_dir;
            else if (hit_axis == 2) n.z = hit_dir;
            result.normal = n;
        }
    }

    return result;
}

/**
 * Resolves continuous collision velocity response with restitution (bounciness) and friction.
 */
inline void resolve_collision_response(
    math::Vec3f& vel,
    const math::Vec3f& normal,
    float restitution = 0.2f,
    float friction = 0.3f) noexcept {

    const float normal_vel = vel.dot(normal);
    if (normal_vel < 0.0f) {
        const math::Vec3f normal_component = normal * normal_vel;
        const math::Vec3f tangent_component = vel - normal_component;

        // Invert and dampen normal velocity based on restitution
        const math::Vec3f impulse_normal = normal_component * -(1.0f + restitution);
        // Tangential friction damping
        const math::Vec3f friction_damp = tangent_component * (1.0f - std::clamp(friction, 0.0f, 1.0f));

        vel = friction_damp - normal_component * restitution;
    }
}

} // namespace scandium::physics
