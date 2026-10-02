#pragma once

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "scandium/math/vec3.hpp"

namespace scandium::spatial {

class SpatialHash {
public:
    explicit SpatialHash(float cell_size) : cell_size_(cell_size) {}

    void clear() { cells_.clear(); }

    void insert(std::uint32_t id, const math::Vec3f& position) {
        cells_[key(position)].push_back(id);
    }

    [[nodiscard]] std::vector<std::uint32_t> query(const math::Vec3f& position) const {
        const auto [cx, cy, cz] = cell(position);
        std::vector<std::uint32_t> result;

        for (int x = cx - 1; x <= cx + 1; ++x) {
            for (int y = cy - 1; y <= cy + 1; ++y) {
                for (int z = cz - 1; z <= cz + 1; ++z) {
                    const auto it = cells_.find(pack(x, y, z));
                    if (it != cells_.end()) {
                        result.insert(result.end(), it->second.begin(), it->second.end());
                    }
                }
            }
        }
        return result;
    }

private:
    struct Cell {
        std::int32_t x{};
        std::int32_t y{};
        std::int32_t z{};
    };

    struct CellHash {
        std::size_t operator()(const std::uint64_t value) const noexcept {
            return std::hash<std::uint64_t>{}(value);
        }
    };

    float cell_size_;
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>, CellHash> cells_;

    [[nodiscard]] Cell cell(const math::Vec3f& p) const noexcept {
        return {
            static_cast<std::int32_t>(std::floor(p.x / cell_size_)),
            static_cast<std::int32_t>(std::floor(p.y / cell_size_)),
            static_cast<std::int32_t>(std::floor(p.z / cell_size_))
        };
    }

    [[nodiscard]] std::uint64_t key(const math::Vec3f& p) const noexcept {
        const auto c = cell(p);
        return pack(c.x, c.y, c.z);
    }

    [[nodiscard]] static std::uint64_t pack(
        std::int32_t x, std::int32_t y, std::int32_t z) noexcept {
        const std::uint64_t ux = static_cast<std::uint32_t>(x);
        const std::uint64_t uy = static_cast<std::uint32_t>(y);
        const std::uint64_t uz = static_cast<std::uint32_t>(z);
        return (ux << 32) ^ (uy * 0x9E3779B1ULL) ^ uz;
    }
};

} // namespace scandium::spatial
