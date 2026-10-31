#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include "scandium/runtime/engine.hpp"

namespace {
void populate(scandium::runtime::Engine& engine, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        const float x = static_cast<float>(i % 200) - 100.0f;
        const float z = static_cast<float>(i / 200) - 25.0f;
        engine.world().add_agent({
            static_cast<std::uint32_t>(i), {x, 0.0f, z}, {}, {},
            scandium::simulation::AgentState::Idle, 4.0f
        });
    }
}

double run_simulation(scandium::runtime::Engine& engine, int frames) {
    using clock = std::chrono::steady_clock;
    const auto begin = clock::now();
    for (int frame = 0; frame < frames; ++frame) {
        engine.set_target({0.0f, 0.0f, 0.0f});
        engine.advance(1.0 / 60.0);
    }
    const auto end = clock::now();
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

double median(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t mid = v.size() / 2;
    return (v.size() % 2 == 0) ? (v[mid - 1] + v[mid]) * 0.5 : v[mid];
}
} // namespace

int main() {
    using namespace scandium;

    std::cout << "SCANDIUM SIMULATION SCALE RIGOROUS BENCHMARK\n";
    std::cout << "============================================\n";
    std::cout << "agents,frames,workers,adaptive_threshold,median_ms,speedup_vs_seq,tick_us\n";

    constexpr int frames = 120;
    constexpr int trials = 3;

    for (const std::size_t agents : {500ULL, 1000ULL, 5000ULL, 10000ULL, 20000ULL}) {
        double seq_median_ms = 0.0;

        for (const std::size_t workers : {1ULL, 2ULL, 4ULL, 8ULL, 16ULL}) {
            std::vector<double> trial_times;
            trial_times.reserve(trials);

            for (int t = 0; t < trials; ++t) {
                runtime::Engine engine{{1.0 / 60.0, 8}, workers};
                engine.world().config().parallel_threshold = 2000;
                populate(engine, agents);

                const double elapsed_ms = run_simulation(engine, frames);
                trial_times.push_back(elapsed_ms);
            }

            const double med_ms = median(trial_times);
            if (workers == 1) {
                seq_median_ms = med_ms;
            }

            const double speedup = seq_median_ms > 0.0 ? seq_median_ms / med_ms : 1.0;
            const double tick_us = (med_ms / frames) * 1000.0;

            std::cout << agents << ","
                      << frames << ","
                      << workers << ","
                      << 2000 << ","
                      << std::fixed << std::setprecision(2)
                      << med_ms << ","
                      << std::setprecision(2) << speedup << "x,"
                      << std::setprecision(1) << tick_us << "us\n";
        }
    }

    return 0;
}
