#include <chrono>
#include <cstddef>
#include <iostream>
#include <random>
#include <vector>

#include "scandium/physics/aabb.hpp"
#include "scandium/spatial/spatial_hash.hpp"

using Clock = std::chrono::steady_clock;
using scandium::math::Vec3f;
using scandium::physics::Aabb;
using scandium::spatial::SpatialHash;

std::vector<Aabb> make_boxes(std::size_t count) {
    std::mt19937 rng{42};
    std::uniform_real_distribution<float> d{-100.0f, 100.0f};
    std::vector<Aabb> boxes;
    boxes.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const Vec3f p{d(rng), d(rng), d(rng)};
        boxes.push_back({p, p + Vec3f{1.0f, 1.0f, 1.0f}});
    }
    return boxes;
}

std::size_t naive(const std::vector<Aabb>& boxes) {
    std::size_t hits = 0;
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        for (std::size_t j = i + 1; j < boxes.size(); ++j) {
            hits += boxes[i].intersects(boxes[j]) ? 1U : 0U;
        }
    }
    return hits;
}

std::size_t hashed(const std::vector<Aabb>& boxes) {
    SpatialHash hash{5.0f};
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        hash.insert(static_cast<std::uint32_t>(i), boxes[i].center());
    }

    std::size_t candidates = 0;
    for (const auto& box : boxes) {
        candidates += hash.query(box.center()).size();
    }
    return candidates;
}

template <typename Fn>
double time_ms(Fn&& fn) {
    const auto start = Clock::now();
    volatile auto result = fn();
    (void)result;
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

int main() {
    std::cout << "entities,naive_ms,spatial_hash_ms\n";
    for (const std::size_t count : {100U, 250U, 500U, 1000U, 2000U}) {
        const auto boxes = make_boxes(count);
        const auto naive_ms = time_ms([&] { return naive(boxes); });
        const auto hash_ms = time_ms([&] { return hashed(boxes); });
        std::cout << count << ',' << naive_ms << ',' << hash_ms << '\n';
    }
}
