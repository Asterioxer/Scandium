#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "scandium/math/vec3.hpp"
#include "scandium/physics/aabb.hpp"

namespace scandium::spatial {

enum class BroadphaseStrategy {
    Auto,        // Chooses Naive if N < crossover_threshold, else SpatialHash
    Naive,       // Always uses naive contiguous all-pairs check
    SpatialHash  // Always uses spatial hash grid
};

class SpatialHash {
public:
    explicit SpatialHash(float cell_size, std::size_t initial_bucket_count = 8192)
        : cell_size_(cell_size > 1e-4f ? cell_size : 1.0f),
          bucket_mask_(next_pow2(initial_bucket_count) - 1),
          bucket_heads_(next_pow2(initial_bucket_count), -1) {}

    void clear() noexcept {
        if (entries_.size() < bucket_heads_.size() / 16) {
            for (const auto& entry : entries_) {
                const auto bucket = hash_cell(entry.x, entry.y, entry.z);
                bucket_heads_[bucket] = -1;
            }
        } else {
            std::fill(bucket_heads_.begin(), bucket_heads_.end(), -1);
        }
        entries_.clear();
    }

    void insert(std::uint32_t id, const math::Vec3f& position) {
        const auto c = cell(position);
        const auto bucket = hash_cell(c.x, c.y, c.z);
        const auto entry_idx = static_cast<std::int32_t>(entries_.size());
        entries_.push_back(Entry{
            id,
            bucket_heads_[bucket],
            c.x, c.y, c.z
        });
        bucket_heads_[bucket] = entry_idx;
    }

    template <typename Callback>
    void query(const math::Vec3f& position, Callback&& cb) const {
        const auto [cx, cy, cz] = cell(position);

        for (int x = cx - 1; x <= cx + 1; ++x) {
            for (int y = cy - 1; y <= cy + 1; ++y) {
                for (int z = cz - 1; z <= cz + 1; ++z) {
                    const auto bucket = hash_cell(x, y, z);
                    for (auto idx = bucket_heads_[bucket]; idx != -1; idx = entries_[idx].next) {
                        const auto& entry = entries_[idx];
                        if (entry.x == x && entry.y == y && entry.z == z) {
                            cb(entry.id);
                        }
                    }
                }
            }
        }
    }

    void query(const math::Vec3f& position, std::vector<std::uint32_t>& out_results) const {
        out_results.clear();
        query(position, [&out_results](std::uint32_t id) {
            out_results.push_back(id);
        });
    }

    [[nodiscard]] std::vector<std::uint32_t> query(const math::Vec3f& position) const {
        std::vector<std::uint32_t> result;
        result.reserve(16);
        query(position, [&result](std::uint32_t id) {
            result.push_back(id);
        });
        return result;
    }

    template <typename Callback>
    void query_radius(const math::Vec3f& center, float radius,
                      const std::vector<math::Vec3f>& positions,
                      Callback&& cb) const {
        const float radius_sq = radius * radius;
        const int min_x = static_cast<int>(std::floor((center.x - radius) / cell_size_));
        const int max_x = static_cast<int>(std::floor((center.x + radius) / cell_size_));
        const int min_y = static_cast<int>(std::floor((center.y - radius) / cell_size_));
        const int max_y = static_cast<int>(std::floor((center.y + radius) / cell_size_));
        const int min_z = static_cast<int>(std::floor((center.z - radius) / cell_size_));
        const int max_z = static_cast<int>(std::floor((center.z + radius) / cell_size_));

        for (int x = min_x; x <= max_x; ++x) {
            for (int y = min_y; y <= max_y; ++y) {
                for (int z = min_z; z <= max_z; ++z) {
                    const auto bucket = hash_cell(x, y, z);
                    for (auto idx = bucket_heads_[bucket]; idx != -1; idx = entries_[idx].next) {
                        const auto& entry = entries_[idx];
                        if (entry.x == x && entry.y == y && entry.z == z) {
                            if (entry.id < positions.size()) {
                                const float dist_sq = (positions[entry.id] - center).length_squared();
                                if (dist_sq <= radius_sq) {
                                    cb(entry.id, dist_sq);
                                }
                            } else {
                                cb(entry.id, 0.0f);
                            }
                        }
                    }
                }
            }
        }
    }

    [[nodiscard]] std::size_t size() const noexcept {
        return entries_.size();
    }

    [[nodiscard]] float cell_size() const noexcept {
        return cell_size_;
    }

    void reserve(std::size_t capacity) {
        entries_.reserve(capacity);
    }

private:
    struct Cell {
        std::int32_t x{};
        std::int32_t y{};
        std::int32_t z{};
    };

    struct Entry {
        std::uint32_t id{};
        std::int32_t next{-1};
        std::int32_t x{};
        std::int32_t y{};
        std::int32_t z{};
    };

    static std::size_t next_pow2(std::size_t n) noexcept {
        if (n <= 16) return 16;
        --n;
        n |= n >> 1;
        n |= n >> 2;
        n |= n >> 4;
        n |= n >> 8;
        n |= n >> 16;
        n |= n >> 32;
        return n + 1;
    }

    [[nodiscard]] Cell cell(const math::Vec3f& p) const noexcept {
        return {
            static_cast<std::int32_t>(std::floor(p.x / cell_size_)),
            static_cast<std::int32_t>(std::floor(p.y / cell_size_)),
            static_cast<std::int32_t>(std::floor(p.z / cell_size_))
        };
    }

    [[nodiscard]] std::size_t hash_cell(std::int32_t x, std::int32_t y, std::int32_t z) const noexcept {
        // High-dispersion 3D spatial prime multiplier hash
        const std::uint64_t h = static_cast<std::uint64_t>(x) * 73856093ULL ^
                                static_cast<std::uint64_t>(y) * 19349663ULL ^
                                static_cast<std::uint64_t>(z) * 83492791ULL;
        return static_cast<std::size_t>(h & bucket_mask_);
    }

    float cell_size_;
    std::size_t bucket_mask_;
    std::vector<std::int32_t> bucket_heads_;
    std::vector<Entry> entries_;
};

class AdaptiveBroadphase {
public:
    explicit AdaptiveBroadphase(float cell_size, std::size_t crossover_threshold = 150)
        : spatial_hash_(cell_size), crossover_threshold_(crossover_threshold) {}

    void set_strategy(BroadphaseStrategy strategy) noexcept { strategy_ = strategy; }
    [[nodiscard]] BroadphaseStrategy strategy() const noexcept { return strategy_; }

    void set_crossover_threshold(std::size_t threshold) noexcept { crossover_threshold_ = threshold; }
    [[nodiscard]] std::size_t crossover_threshold() const noexcept { return crossover_threshold_; }

    [[nodiscard]] bool should_use_spatial_hash(std::size_t entity_count) const noexcept {
        if (strategy_ == BroadphaseStrategy::SpatialHash) return true;
        if (strategy_ == BroadphaseStrategy::Naive) return false;
        return entity_count >= crossover_threshold_;
    }

    SpatialHash& spatial_hash() noexcept { return spatial_hash_; }
    [[nodiscard]] const SpatialHash& spatial_hash() const noexcept { return spatial_hash_; }

private:
    SpatialHash spatial_hash_;
    std::size_t crossover_threshold_{150};
    BroadphaseStrategy strategy_{BroadphaseStrategy::Auto};
};

} // namespace scandium::spatial
