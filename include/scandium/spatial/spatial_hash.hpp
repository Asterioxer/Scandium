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
                    const auto it = cells_.find(Cell{
                        static_cast<std::int32_t>(x),
                        static_cast<std::int32_t>(y),
                        static_cast<std::int32_t>(z)
                    });
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
        std::size_t operator()(const Cell& c) const noexcept {
            const auto h1 = std::hash<std::int32_t>{}(c.x);
            const auto h2 = std::hash<std::int32_t>{}(c.y);
            const auto h3 = std::hash<std::int32_t>{}(c.z);
            return h1 ^ (h2 + 0x9e3779b9U + (h1 << 6) + (h1 >> 2)) ^
                   (h3 + 0x9e3779b9U + (h2 << 6) + (h2 >> 2));
        }
    };

    struct CellEqual {
        bool operator()(const Cell& a, const Cell& b) const noexcept {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }
    };

    float cell_size_;
    std::unordered_map<Cell, std::vector<std::uint32_t>, CellHash, CellEqual> cells_;

    [[nodiscard]] Cell cell(const math::Vec3f& p) const noexcept {
        return {
            static_cast<std::int32_t>(std::floor(p.x / cell_size_)),
            static_cast<std::int32_t>(std::floor(p.y / cell_size_)),
            static_cast<std::int32_t>(std::floor(p.z / cell_size_))
        };
    }

    [[nodiscard]] Cell key(const math::Vec3f& p) const noexcept {
        return cell(p);
    }
};

} // namespace scandium::spatial
