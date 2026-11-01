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

struct SpatialHashDiagnostics {
    std::size_t total_entries{0};
    std::size_t total_buckets{0};
    std::size_t occupied_buckets{0};
    double empty_bucket_pct{0.0};
    double avg_bucket_occupancy{0.0};
    std::size_t max_bucket_occupancy{0};
    std::size_t hash_collisions{0}; // non-empty buckets with >1 distinct cell
    std::size_t memory_bytes{0};
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
                const auto bucket = hash_cell_key(entry.cell_key);
                bucket_heads_[bucket] = -1;
            }
        } else {
            std::fill(bucket_heads_.begin(), bucket_heads_.end(), -1);
        }
        entries_.clear();
    }

    void insert(std::uint32_t id, const math::Vec3f& position) {
        const auto c = cell(position);
        const auto key = pack_cell(c.x, c.y, c.z);
        const auto bucket = hash_cell_key(key);
        const auto entry_idx = static_cast<std::int32_t>(entries_.size());
        entries_.push_back(Entry{
            id,
            bucket_heads_[bucket],
            key
        });
        bucket_heads_[bucket] = entry_idx;
    }

    template <typename Callback>
    void query(const math::Vec3f& position, Callback&& cb) const {
        const auto [cx, cy, cz] = cell(position);

        for (int x = cx - 1; x <= cx + 1; ++x) {
            for (int y = cy - 1; y <= cy + 1; ++y) {
                for (int z = cz - 1; z <= cz + 1; ++z) {
                    const auto key = pack_cell(x, y, z);
                    const auto bucket = hash_cell_key(key);
                    for (auto idx = bucket_heads_[bucket]; idx != -1; idx = entries_[idx].next) {
                        const auto& entry = entries_[idx];
                        if (entry.cell_key == key) {
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
                    const auto key = pack_cell(x, y, z);
                    const auto bucket = hash_cell_key(key);
                    for (auto idx = bucket_heads_[bucket]; idx != -1; idx = entries_[idx].next) {
                        const auto& entry = entries_[idx];
                        if (entry.cell_key == key) {
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

    [[nodiscard]] SpatialHashDiagnostics diagnostics() const {
        SpatialHashDiagnostics diag{};
        diag.total_entries = entries_.size();
        diag.total_buckets = bucket_heads_.size();
        diag.memory_bytes = sizeof(*this) +
                            bucket_heads_.capacity() * sizeof(std::int32_t) +
                            entries_.capacity() * sizeof(Entry);

        std::size_t occupied = 0;
        std::size_t max_chain = 0;
        std::size_t total_chain_len = 0;
        std::size_t hash_collisions = 0;

        for (std::size_t b = 0; b < bucket_heads_.size(); ++b) {
            auto idx = bucket_heads_[b];
            if (idx == -1) continue;
            ++occupied;
            std::size_t chain_len = 0;
            std::uint64_t first_key = entries_[idx].cell_key;
            bool multiple_cells = false;

            while (idx != -1) {
                ++chain_len;
                const auto& e = entries_[idx];
                if (e.cell_key != first_key) {
                    multiple_cells = true;
                }
                idx = e.next;
            }
            if (multiple_cells) {
                ++hash_collisions;
            }
            total_chain_len += chain_len;
            if (chain_len > max_chain) {
                max_chain = chain_len;
            }
        }

        diag.occupied_buckets = occupied;
        diag.empty_bucket_pct = diag.total_buckets > 0
            ? 100.0 * static_cast<double>(diag.total_buckets - occupied) / static_cast<double>(diag.total_buckets)
            : 0.0;
        diag.avg_bucket_occupancy = occupied > 0
            ? static_cast<double>(total_chain_len) / static_cast<double>(occupied)
            : 0.0;
        diag.max_bucket_occupancy = max_chain;
        diag.hash_collisions = hash_collisions;

        return diag;
    }

private:
    struct Cell {
        std::int32_t x{};
        std::int32_t y{};
        std::int32_t z{};
    };

    // Compact 16-byte Entry aligned to 16 bytes: exactly 4 entries per 64-byte cache line
    struct alignas(16) Entry {
        std::uint32_t id{};
        std::int32_t next{-1};
        std::uint64_t cell_key{};
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

    static constexpr std::uint64_t pack_cell(std::int32_t x, std::int32_t y, std::int32_t z) noexcept {
        const std::uint64_t ux = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) & 0x1FFFFFULL;
        const std::uint64_t uy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(y)) & 0x1FFFFFULL;
        const std::uint64_t uz = static_cast<std::uint64_t>(static_cast<std::uint32_t>(z)) & 0x3FFFFFULL;
        return ux | (uy << 21) | (uz << 42);
    }

    [[nodiscard]] std::size_t hash_cell_key(std::uint64_t key) const noexcept {
        // High-dispersion SplitMix64 finalizer guarantees uniform bit mixing into low bits
        key ^= key >> 30;
        key *= 0xbf58476d1ce4e5b9ULL;
        key ^= key >> 27;
        key *= 0x94d049bb133111ebULL;
        key ^= key >> 31;
        return static_cast<std::size_t>(key & bucket_mask_);
    }

    [[nodiscard]] std::size_t hash_cell(std::int32_t x, std::int32_t y, std::int32_t z) const noexcept {
        return hash_cell_key(pack_cell(x, y, z));
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
