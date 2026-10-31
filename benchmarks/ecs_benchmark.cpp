#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <vector>

#include "scandium/ecs/registry.hpp"

using Clock = std::chrono::steady_clock;
using namespace scandium::ecs;

struct VelocityComponent {
    float vx{1.0f};
    float vy{0.5f};
    float vz{-0.2f};
};

struct HealthComponent {
    float hp{100.0f};
    float max_hp{100.0f};
};

int main() {
    std::cout << "SCANDIUM ECS REPRODUCIBLE BENCHMARK\n";
    std::cout << "===================================\n";
    std::cout << "entities,dense_iter_ms,view_iter_ms,throughput_Mops\n";

    for (const std::size_t count : {1000ULL, 10000ULL, 50000ULL, 100000ULL}) {
        Registry registry;

        for (std::size_t i = 0; i < count; ++i) {
            const auto id = registry.create();
            registry.transform(id).position = {static_cast<float>(i), 0.0f, 0.0f};
            registry.emplace<VelocityComponent>(id, {1.0f, 0.0f, 1.0f});
            if (i % 2 == 0) {
                registry.emplace<HealthComponent>(id, {100.0f, 100.0f});
            }
        }

        constexpr int iterations = 50;

        // Benchmark 1: Dense component iteration
        const auto start_dense = Clock::now();
        volatile float sum_x = 0.0f;
        for (int it = 0; it < iterations; ++it) {
            registry.each<VelocityComponent>([&](EntityId, VelocityComponent& v) {
                sum_x += v.vx;
            });
        }
        const auto end_dense = Clock::now();
        const double dense_ms =
            std::chrono::duration<double, std::milli>(end_dense - start_dense).count() / iterations;

        // Benchmark 2: Multi-component view iteration
        const auto start_view = Clock::now();
        volatile float sum_hp = 0.0f;
        for (int it = 0; it < iterations; ++it) {
            registry.view<VelocityComponent, HealthComponent>(
                [&](EntityId, VelocityComponent& v, HealthComponent& h) {
                    sum_hp += (h.hp + v.vx);
                });
        }
        const auto end_view = Clock::now();
        const double view_ms =
            std::chrono::duration<double, std::milli>(end_view - start_view).count() / iterations;

        const double throughput_mops = (count / (dense_ms * 1e-3)) / 1e6;

        std::cout << count << ","
                  << std::fixed << std::setprecision(4)
                  << dense_ms << ","
                  << view_ms << ","
                  << std::setprecision(2) << throughput_mops << "\n";
    }

    return 0;
}
