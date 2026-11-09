#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#if defined(_M_AMD64) || defined(_M_X64) || defined(__x86_64__) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
#define SCANDIUM_HAS_X86_SIMD 1
#else
#define SCANDIUM_HAS_X86_SIMD 0
#endif

#if defined(__GNUC__) || defined(__clang__)
    #define SCANDIUM_TARGET_AVX2 __attribute__((target("avx2,fma")))
    #define SCANDIUM_TARGET_SSE  __attribute__((target("sse,sse2,sse3,ssse3,sse4.1")))
#else
    #define SCANDIUM_TARGET_AVX2
    #define SCANDIUM_TARGET_SSE
#endif

#include "scandium/core/cpu_features.hpp"
#include "scandium/math/ray.hpp"
#include "scandium/physics/aabb.hpp"

namespace scandium::physics {

/**
 * @brief Structure of Arrays (SoA) layout for 8 AABBs optimized for 256-bit AVX2 processing.
 */
struct alignas(32) AABB8 {
    float min_x[8];
    float min_y[8];
    float min_z[8];
    float max_x[8];
    float max_y[8];
    float max_z[8];

    static AABB8 from_aabbs(const Aabb* boxes, std::size_t count) noexcept {
        AABB8 soa{};
        const std::size_t valid = std::min<std::size_t>(count, 8);
        for (std::size_t i = 0; i < valid; ++i) {
            soa.min_x[i] = boxes[i].min.x;
            soa.min_y[i] = boxes[i].min.y;
            soa.min_z[i] = boxes[i].min.z;
            soa.max_x[i] = boxes[i].max.x;
            soa.max_y[i] = boxes[i].max.y;
            soa.max_z[i] = boxes[i].max.z;
        }
        // Fill remaining lanes with inverted bounds to guarantee miss
        for (std::size_t i = valid; i < 8; ++i) {
            soa.min_x[i] = std::numeric_limits<float>::infinity();
            soa.min_y[i] = std::numeric_limits<float>::infinity();
            soa.min_z[i] = std::numeric_limits<float>::infinity();
            soa.max_x[i] = -std::numeric_limits<float>::infinity();
            soa.max_y[i] = -std::numeric_limits<float>::infinity();
            soa.max_z[i] = -std::numeric_limits<float>::infinity();
        }
        return soa;
    }
};

/**
 * @brief Structure of Arrays (SoA) layout for 4 AABBs optimized for 128-bit SSE processing.
 */
struct alignas(16) AABB4 {
    float min_x[4];
    float min_y[4];
    float min_z[4];
    float max_x[4];
    float max_y[4];
    float max_z[4];

    static AABB4 from_aabbs(const Aabb* boxes, std::size_t count) noexcept {
        AABB4 soa{};
        const std::size_t valid = std::min<std::size_t>(count, 4);
        for (std::size_t i = 0; i < valid; ++i) {
            soa.min_x[i] = boxes[i].min.x;
            soa.min_y[i] = boxes[i].min.y;
            soa.min_z[i] = boxes[i].min.z;
            soa.max_x[i] = boxes[i].max.x;
            soa.max_y[i] = boxes[i].max.y;
            soa.max_z[i] = boxes[i].max.z;
        }
        for (std::size_t i = valid; i < 4; ++i) {
            soa.min_x[i] = std::numeric_limits<float>::infinity();
            soa.min_y[i] = std::numeric_limits<float>::infinity();
            soa.min_z[i] = std::numeric_limits<float>::infinity();
            soa.max_x[i] = -std::numeric_limits<float>::infinity();
            soa.max_y[i] = -std::numeric_limits<float>::infinity();
            soa.max_z[i] = -std::numeric_limits<float>::infinity();
        }
        return soa;
    }
};

struct Ray8Result {
    std::uint32_t hit_mask{0}; // Bit i is set if box i was intersected
    float hit_distances[8]{};   // Entry distance for each box
};

struct Ray4Result {
    std::uint32_t hit_mask{0};
    float hit_distances[4]{};
};

/**
 * @brief 8-wide AVX2 vectorized ray vs 8 AABBs intersection test.
 */
#if SCANDIUM_HAS_X86_SIMD
SCANDIUM_TARGET_AVX2 inline Ray8Result intersect_ray_8aabb_avx2(const math::Ray& ray, const AABB8& boxes, float max_distance = 1e30f) noexcept {
    Ray8Result result{};

    const float inv_x = std::fabs(ray.direction.x) > 1e-7f ? 1.0f / ray.direction.x : (ray.direction.x >= 0.0f ? 1e30f : -1e30f);
    const float inv_y = std::fabs(ray.direction.y) > 1e-7f ? 1.0f / ray.direction.y : (ray.direction.y >= 0.0f ? 1e30f : -1e30f);
    const float inv_z = std::fabs(ray.direction.z) > 1e-7f ? 1.0f / ray.direction.z : (ray.direction.z >= 0.0f ? 1e30f : -1e30f);

    const __m256 ro_x = _mm256_set1_ps(ray.origin.x);
    const __m256 ro_y = _mm256_set1_ps(ray.origin.y);
    const __m256 ro_z = _mm256_set1_ps(ray.origin.z);

    const __m256 rinv_x = _mm256_set1_ps(inv_x);
    const __m256 rinv_y = _mm256_set1_ps(inv_y);
    const __m256 rinv_z = _mm256_set1_ps(inv_z);

    const __m256 bmin_x = _mm256_loadu_ps(boxes.min_x);
    const __m256 bmin_y = _mm256_loadu_ps(boxes.min_y);
    const __m256 bmin_z = _mm256_loadu_ps(boxes.min_z);

    const __m256 bmax_x = _mm256_loadu_ps(boxes.max_x);
    const __m256 bmax_y = _mm256_loadu_ps(boxes.max_y);
    const __m256 bmax_z = _mm256_loadu_ps(boxes.max_z);

    // X slabs
    const __m256 t1_x = _mm256_mul_ps(_mm256_sub_ps(bmin_x, ro_x), rinv_x);
    const __m256 t2_x = _mm256_mul_ps(_mm256_sub_ps(bmax_x, ro_x), rinv_x);
    __m256 near_x = _mm256_min_ps(t1_x, t2_x);
    __m256 far_x = _mm256_max_ps(t1_x, t2_x);

    // Y slabs
    const __m256 t1_y = _mm256_mul_ps(_mm256_sub_ps(bmin_y, ro_y), rinv_y);
    const __m256 t2_y = _mm256_mul_ps(_mm256_sub_ps(bmax_y, ro_y), rinv_y);
    __m256 near_y = _mm256_min_ps(t1_y, t2_y);
    __m256 far_y = _mm256_max_ps(t1_y, t2_y);

    // Z slabs
    const __m256 t1_z = _mm256_mul_ps(_mm256_sub_ps(bmin_z, ro_z), rinv_z);
    const __m256 t2_z = _mm256_mul_ps(_mm256_sub_ps(bmax_z, ro_z), rinv_z);
    __m256 near_z = _mm256_min_ps(t1_z, t2_z);
    __m256 far_z = _mm256_max_ps(t1_z, t2_z);

    __m256 t_min = _mm256_max_ps(_mm256_max_ps(near_x, near_y), near_z);
    __m256 t_max = _mm256_min_ps(_mm256_min_ps(far_x, far_y), far_z);

    t_min = _mm256_max_ps(t_min, _mm256_setzero_ps());
    t_max = _mm256_min_ps(t_max, _mm256_set1_ps(max_distance));

    const __m256 hit_mask = _mm256_cmp_ps(t_min, t_max, _CMP_LE_OQ);
    result.hit_mask = static_cast<std::uint32_t>(_mm256_movemask_ps(hit_mask));

    _mm256_storeu_ps(result.hit_distances, t_min);
    return result;
}
#endif

/**
 * @brief 4-wide SSE vectorized ray vs 4 AABBs intersection test.
 */
#if SCANDIUM_HAS_X86_SIMD
SCANDIUM_TARGET_SSE inline Ray4Result intersect_ray_4aabb_sse(const math::Ray& ray, const AABB4& boxes, float max_distance = 1e30f) noexcept {
    Ray4Result result{};

    const float inv_x = std::fabs(ray.direction.x) > 1e-7f ? 1.0f / ray.direction.x : (ray.direction.x >= 0.0f ? 1e30f : -1e30f);
    const float inv_y = std::fabs(ray.direction.y) > 1e-7f ? 1.0f / ray.direction.y : (ray.direction.y >= 0.0f ? 1e30f : -1e30f);
    const float inv_z = std::fabs(ray.direction.z) > 1e-7f ? 1.0f / ray.direction.z : (ray.direction.z >= 0.0f ? 1e30f : -1e30f);

    const __m128 ro_x = _mm_set1_ps(ray.origin.x);
    const __m128 ro_y = _mm_set1_ps(ray.origin.y);
    const __m128 ro_z = _mm_set1_ps(ray.origin.z);

    const __m128 rinv_x = _mm_set1_ps(inv_x);
    const __m128 rinv_y = _mm_set1_ps(inv_y);
    const __m128 rinv_z = _mm_set1_ps(inv_z);

    const __m128 bmin_x = _mm_loadu_ps(boxes.min_x);
    const __m128 bmin_y = _mm_loadu_ps(boxes.min_y);
    const __m128 bmin_z = _mm_loadu_ps(boxes.min_z);

    const __m128 bmax_x = _mm_loadu_ps(boxes.max_x);
    const __m128 bmax_y = _mm_loadu_ps(boxes.max_y);
    const __m128 bmax_z = _mm_loadu_ps(boxes.max_z);

    const __m128 t1_x = _mm_mul_ps(_mm_sub_ps(bmin_x, ro_x), rinv_x);
    const __m128 t2_x = _mm_mul_ps(_mm_sub_ps(bmax_x, ro_x), rinv_x);
    __m128 near_x = _mm_min_ps(t1_x, t2_x);
    __m128 far_x = _mm_max_ps(t1_x, t2_x);

    const __m128 t1_y = _mm_mul_ps(_mm_sub_ps(bmin_y, ro_y), rinv_y);
    const __m128 t2_y = _mm_mul_ps(_mm_sub_ps(bmax_y, ro_y), rinv_y);
    __m128 near_y = _mm_min_ps(t1_y, t2_y);
    __m128 far_y = _mm_max_ps(t1_y, t2_y);

    const __m128 t1_z = _mm_mul_ps(_mm_sub_ps(bmin_z, ro_z), rinv_z);
    const __m128 t2_z = _mm_mul_ps(_mm_sub_ps(bmax_z, ro_z), rinv_z);
    __m128 near_z = _mm_min_ps(t1_z, t2_z);
    __m128 far_z = _mm_max_ps(t1_z, t2_z);

    __m128 t_min = _mm_max_ps(_mm_max_ps(near_x, near_y), near_z);
    __m128 t_max = _mm_min_ps(_mm_min_ps(far_x, far_y), far_z);

    t_min = _mm_max_ps(t_min, _mm_setzero_ps());
    t_max = _mm_min_ps(t_max, _mm_set1_ps(max_distance));

    const __m128 hit_mask = _mm_cmple_ps(t_min, t_max);
    result.hit_mask = static_cast<std::uint32_t>(_mm_movemask_ps(hit_mask));

    _mm_storeu_ps(result.hit_distances, t_min);
    return result;
}
#endif

/**
 * @brief Portable scalar fallback for 8 AABBs intersection.
 */
inline Ray8Result intersect_ray_8aabb_scalar(const math::Ray& ray, const AABB8& boxes, float max_distance = 1e30f) noexcept {
    Ray8Result result{};

    const float inv_x = std::fabs(ray.direction.x) > 1e-7f ? 1.0f / ray.direction.x : (ray.direction.x >= 0.0f ? 1e30f : -1e30f);
    const float inv_y = std::fabs(ray.direction.y) > 1e-7f ? 1.0f / ray.direction.y : (ray.direction.y >= 0.0f ? 1e30f : -1e30f);
    const float inv_z = std::fabs(ray.direction.z) > 1e-7f ? 1.0f / ray.direction.z : (ray.direction.z >= 0.0f ? 1e30f : -1e30f);

    for (int i = 0; i < 8; ++i) {
        float near_x = (boxes.min_x[i] - ray.origin.x) * inv_x;
        float far_x = (boxes.max_x[i] - ray.origin.x) * inv_x;
        if (near_x > far_x) std::swap(near_x, far_x);

        float near_y = (boxes.min_y[i] - ray.origin.y) * inv_y;
        float far_y = (boxes.max_y[i] - ray.origin.y) * inv_y;
        if (near_y > far_y) std::swap(near_y, far_y);

        float near_z = (boxes.min_z[i] - ray.origin.z) * inv_z;
        float far_z = (boxes.max_z[i] - ray.origin.z) * inv_z;
        if (near_z > far_z) std::swap(near_z, far_z);

        const float t_min = std::max({0.0f, near_x, near_y, near_z});
        const float t_max = std::min({max_distance, far_x, far_y, far_z});

        if (t_min <= t_max) {
            result.hit_mask |= (1u << i);
            result.hit_distances[i] = t_min;
        }
    }

    return result;
}

/**
 * @brief Dynamic hardware-dispatched 8-box intersection using CPUID.
 */
inline Ray8Result intersect_ray_8aabb(const math::Ray& ray, const AABB8& boxes, float max_distance = 1e30f) noexcept {
#if SCANDIUM_HAS_X86_SIMD
    static const bool has_avx2 = core::CPUFeatures::has_avx2();
    if (has_avx2) {
        return intersect_ray_8aabb_avx2(ray, boxes, max_distance);
    }
#endif
    return intersect_ray_8aabb_scalar(ray, boxes, max_distance);
}

} // namespace scandium::physics
