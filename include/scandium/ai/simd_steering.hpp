#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#if defined(_M_AMD64) || defined(_M_X64) || defined(__x86_64__) || defined(__i386__) || defined(_M_IX86)
    #include <immintrin.h>
    #define SCANDIUM_HAS_X86_SIMD 1
#else
    #define SCANDIUM_HAS_X86_SIMD 0
#endif

#include "scandium/math/vec3.hpp"

namespace scandium::ai {

/// Structure-of-Arrays (SoA) layout for high-throughput vectorized agent simulation
struct AgentSoA {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    std::vector<float> vx;
    std::vector<float> vy;
    std::vector<float> vz;
    std::vector<float> radius;
    std::vector<float> max_speed;

    void reserve(std::size_t capacity) {
        x.reserve(capacity);
        y.reserve(capacity);
        z.reserve(capacity);
        vx.reserve(capacity);
        vy.reserve(capacity);
        vz.reserve(capacity);
        radius.reserve(capacity);
        max_speed.reserve(capacity);
    }

    void resize(std::size_t size) {
        x.resize(size, 0.0f);
        y.resize(size, 0.0f);
        z.resize(size, 0.0f);
        vx.resize(size, 0.0f);
        vy.resize(size, 0.0f);
        vz.resize(size, 0.0f);
        radius.resize(size, 0.5f);
        max_speed.resize(size, 5.0f);
    }

    void clear() noexcept {
        x.clear();
        y.clear();
        z.clear();
        vx.clear();
        vy.clear();
        vz.clear();
        radius.clear();
        max_speed.clear();
    }

    [[nodiscard]] std::size_t size() const noexcept {
        return x.size();
    }

    [[nodiscard]] bool empty() const noexcept {
        return x.empty();
    }

    void push_back(float px, float py, float pz,
                   float vel_x = 0.0f, float vel_y = 0.0f, float vel_z = 0.0f,
                   float r = 0.5f, float spd = 5.0f) {
        x.push_back(px);
        y.push_back(py);
        z.push_back(pz);
        vx.push_back(vel_x);
        vy.push_back(vel_y);
        vz.push_back(vel_z);
        radius.push_back(r);
        max_speed.push_back(spd);
    }

    [[nodiscard]] math::Vec3f get_position(std::size_t index) const noexcept {
        return {x[index], y[index], z[index]};
    }

    [[nodiscard]] math::Vec3f get_velocity(std::size_t index) const noexcept {
        return {vx[index], vy[index], vz[index]};
    }

    void set_position(std::size_t index, const math::Vec3f& pos) noexcept {
        x[index] = pos.x;
        y[index] = pos.y;
        z[index] = pos.z;
    }

    void set_velocity(std::size_t index, const math::Vec3f& vel) noexcept {
        vx[index] = vel.x;
        vy[index] = vel.y;
        vz[index] = vel.z;
    }
};

namespace detail {

#if SCANDIUM_HAS_X86_SIMD
inline float hsum_ps_sse(__m128 v) noexcept {
    // [v0+v1, v2+v3, v0+v1, v2+v3]
    __m128 shuf = _mm_movehdup_ps(v);
    __m128 sums = _mm_add_ps(v, shuf);
    shuf = _mm_movehl_ps(shuf, sums);
    sums = _mm_add_ss(sums, shuf);
    return _mm_cvtss_f32(sums);
}
#endif

} // namespace detail

/**
 * Computes Reynolds separation steering force for an agent against a contiguous SoA neighbor set.
 * Uses 4-wide SSE hardware acceleration on x86/x64 with automatic scalar remainder handling.
 */
[[nodiscard]] inline math::Vec3f compute_simd_separation_soa(
    float px, float py, float pz,
    const float* __restrict xs,
    const float* __restrict ys,
    const float* __restrict zs,
    std::size_t count,
    float separation_radius,
    float max_speed) noexcept {

    if (count == 0 || separation_radius <= 0.0f) {
        return {};
    }

    float total_force_x = 0.0f;
    float total_force_y = 0.0f;
    float total_force_z = 0.0f;
    float valid_neighbors = 0.0f;

    const float radius_sq = separation_radius * separation_radius;
    constexpr float eps_sq = 1e-5f;

    std::size_t i = 0;

#if SCANDIUM_HAS_X86_SIMD
    const __m128 v_px = _mm_set1_ps(px);
    const __m128 v_py = _mm_set1_ps(py);
    const __m128 v_pz = _mm_set1_ps(pz);
    const __m128 v_rad = _mm_set1_ps(separation_radius);
    const __m128 v_rad_sq = _mm_set1_ps(radius_sq);
    const __m128 v_eps_sq = _mm_set1_ps(eps_sq);
    const __m128 v_ones = _mm_set1_ps(1.0f);

    __m128 acc_fx = _mm_setzero_ps();
    __m128 acc_fy = _mm_setzero_ps();
    __m128 acc_fz = _mm_setzero_ps();
    __m128 acc_cnt = _mm_setzero_ps();

    const std::size_t simd_end = count & ~std::size_t{3};
    for (; i < simd_end; i += 4) {
        const __m128 nx = _mm_loadu_ps(xs + i);
        const __m128 ny = _mm_loadu_ps(ys + i);
        const __m128 nz = _mm_loadu_ps(zs + i);

        const __m128 dx = _mm_sub_ps(v_px, nx);
        const __m128 dy = _mm_sub_ps(v_py, ny);
        const __m128 dz = _mm_sub_ps(v_pz, nz);

        const __m128 d_sq = _mm_add_ps(
            _mm_mul_ps(dx, dx),
            _mm_add_ps(_mm_mul_ps(dy, dy), _mm_mul_ps(dz, dz))
        );

        // Mask: eps_sq < d_sq < radius_sq
        const __m128 mask_gt = _mm_cmpgt_ps(d_sq, v_eps_sq);
        const __m128 mask_lt = _mm_cmplt_ps(d_sq, v_rad_sq);
        const __m128 mask = _mm_and_ps(mask_gt, mask_lt);

        // If any lanes matched
        if (_mm_movemask_ps(mask) != 0) {
            const __m128 dist = _mm_sqrt_ps(d_sq);
            // force factor: (radius - dist) / dist
            const __m128 factor = _mm_div_ps(_mm_sub_ps(v_rad, dist), dist);
            const __m128 masked_factor = _mm_and_ps(factor, mask);

            acc_fx = _mm_add_ps(acc_fx, _mm_mul_ps(dx, masked_factor));
            acc_fy = _mm_add_ps(acc_fy, _mm_mul_ps(dy, masked_factor));
            acc_fz = _mm_add_ps(acc_fz, _mm_mul_ps(dz, masked_factor));
            acc_cnt = _mm_add_ps(acc_cnt, _mm_and_ps(v_ones, mask));
        }
    }

    total_force_x = detail::hsum_ps_sse(acc_fx);
    total_force_y = detail::hsum_ps_sse(acc_fy);
    total_force_z = detail::hsum_ps_sse(acc_fz);
    valid_neighbors = detail::hsum_ps_sse(acc_cnt);
#endif

    // Remainder scalar loop
    for (; i < count; ++i) {
        const float dx = px - xs[i];
        const float dy = py - ys[i];
        const float dz = pz - zs[i];
        const float d_sq = dx * dx + dy * dy + dz * dz;

        if (d_sq > eps_sq && d_sq < radius_sq) {
            const float dist = std::sqrt(d_sq);
            const float factor = (separation_radius - dist) / dist;
            total_force_x += dx * factor;
            total_force_y += dy * factor;
            total_force_z += dz * factor;
            valid_neighbors += 1.0f;
        }
    }

    if (valid_neighbors > 0.0f) {
        math::Vec3f steer{total_force_x, total_force_y, total_force_z};
        const float len_sq = steer.length_squared();
        if (len_sq > 1e-6f) {
            return steer.normalized() * max_speed;
        }
    }

    return {};
}

/**
 * Computes distance from a reference origin to an array of points in parallel.
 * Stores result in out_distances.
 */
inline void simd_compute_distances_soa(
    float px, float py, float pz,
    const float* __restrict xs,
    const float* __restrict ys,
    const float* __restrict zs,
    float* __restrict out_distances,
    std::size_t count) noexcept {

    if (count == 0) return;

    std::size_t i = 0;

#if SCANDIUM_HAS_X86_SIMD
    const __m128 v_px = _mm_set1_ps(px);
    const __m128 v_py = _mm_set1_ps(py);
    const __m128 v_pz = _mm_set1_ps(pz);

    const std::size_t simd_end = count & ~std::size_t{3};
    for (; i < simd_end; i += 4) {
        const __m128 dx = _mm_sub_ps(v_px, _mm_loadu_ps(xs + i));
        const __m128 dy = _mm_sub_ps(v_py, _mm_loadu_ps(ys + i));
        const __m128 dz = _mm_sub_ps(v_pz, _mm_loadu_ps(zs + i));

        const __m128 d_sq = _mm_add_ps(
            _mm_mul_ps(dx, dx),
            _mm_add_ps(_mm_mul_ps(dy, dy), _mm_mul_ps(dz, dz))
        );
        const __m128 dist = _mm_sqrt_ps(d_sq);
        _mm_storeu_ps(out_distances + i, dist);
    }
#endif

    for (; i < count; ++i) {
        const float dx = px - xs[i];
        const float dy = py - ys[i];
        const float dz = pz - zs[i];
        out_distances[i] = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
}

/**
 * Computes Reynolds separation for an Array-of-Structures (AoS) vector of Vec3f neighbors.
 */
[[nodiscard]] inline math::Vec3f compute_simd_separation_aos(
    const math::Vec3f& position,
    const math::Vec3f* neighbors,
    std::size_t count,
    float separation_radius,
    float max_speed) noexcept {

    if (count == 0 || separation_radius <= 0.0f) {
        return {};
    }

    math::Vec3f total_force{};
    int valid_count = 0;
    const float radius_sq = separation_radius * separation_radius;
    constexpr float eps_sq = 1e-5f;

    for (std::size_t i = 0; i < count; ++i) {
        const math::Vec3f diff = position - neighbors[i];
        const float d_sq = diff.length_squared();
        if (d_sq > eps_sq && d_sq < radius_sq) {
            const float dist = std::sqrt(d_sq);
            total_force += (diff / dist) * (separation_radius - dist);
            ++valid_count;
        }
    }

    if (valid_count > 0 && total_force.length_squared() > 1e-6f) {
        return total_force.normalized() * max_speed;
    }

    return {};
}

} // namespace scandium::ai
