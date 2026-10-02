#include <chrono>
#include <cstddef>
#include <iostream>
#include <random>
#include <vector>

#include "scandium/physics/aabb.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using scandium::physics::Aabb;

std::size_t naive_pairs(const std::vector<Aabb>& boxes) {
    std::size_t intersections = 0;
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        for (std::size_t j = i + 1; j < boxes.size(); ++j) {
            intersections += boxes[i].intersects(boxes[j]) ? 1U : 0U;
        }
    }
    return intersections;
}

} // namespace

int main() {
    std::mt19937 rng{42};
    std::uniform_real_distribution<float> distribution{-100.0f, 100.0f};

    std::cout << "Scandium collision baseline benchmark\n";
    std::cout << "deterministic seed: 42\n";
    std::cout << "entities,elapsed_ms,intersections\n";

    for (const std::size_t count : {100U, 250U, 500U, 1000U, 2000U}) {
        std::vector<Aabb> boxes;
        boxes.reserve(count);

        for (std::size_t i = 0; i < count; ++i) {
            const float x = distribution(rng);
            const float y = distribution(rng);
            const float z = distribution(rng);
            boxes.push_back({{x, y, z}, {x + 1.0f, y + 1.0f, z + 1.0f}});
        }

        const auto start = Clock::now();
        const std::size_t intersections = naive_pairs(boxes);
        const double elapsed = std::chrono::duration<double, std::milli>(
            Clock::now() - start).count();

        std::cout << count << ',' << elapsed << ',' << intersections << '\n';
    }
}
