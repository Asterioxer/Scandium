#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>

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

double run(scandium::runtime::Engine& engine) {
    using clock = std::chrono::steady_clock;
    const auto begin = clock::now();
    for (int frame = 0; frame < 120; ++frame) {
        engine.set_target({0.0f, 0.0f, 0.0f});
        engine.advance(1.0 / 60.0);
    }
    const auto end = clock::now();
    return std::chrono::duration<double, std::milli>(end - begin).count();
}
}

int main() {
    using namespace scandium;

    constexpr std::size_t agents = 10000;

    runtime::Engine single{{1.0 / 60.0, 8}, 1};
    runtime::Engine parallel{{1.0 / 60.0, 8}, 0};

    populate(single, agents);
    populate(parallel, agents);

    const double single_ms = run(single);
    const double parallel_ms = run(parallel);
    const double speedup = parallel_ms > 0.0 ? single_ms / parallel_ms : 0.0;

    std::cout << "agents,frames,single_ms,parallel_ms,speedup,parallel_workers\n";
    std::cout << agents << ",120," << single_ms << ',' << parallel_ms << ','
              << speedup << ',' << parallel.worker_count() << '\n';
    return 0;
}
