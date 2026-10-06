#include <chrono>
#include <cstddef>
#include <iostream>

#include "scandium/core/job_system.hpp"
#include "scandium/runtime/engine.hpp"

int main() {
    using clock = std::chrono::steady_clock;
    using namespace scandium;

    runtime::Engine engine{{1.0 / 60.0, 8}};
    constexpr std::size_t agents = 10000;

    for (std::size_t i = 0; i < agents; ++i) {
        const float x = static_cast<float>(i % 200) - 100.0f;
        const float z = static_cast<float>(i / 200) - 25.0f;
        engine.world().add_agent({
            static_cast<std::uint32_t>(i), {x, 0.0f, z}
        });
    }

    const auto begin = clock::now();
    for (int frame = 0; frame < 120; ++frame) {
        engine.set_target({0.0f, 0.0f, 0.0f});
        engine.advance(1.0 / 60.0);
    }
    const auto end = clock::now();

    const double elapsed =
        std::chrono::duration<double, std::milli>(end - begin).count();

    std::cout << "agents,frames,total_ms,avg_frame_ms,workers\n";
    std::cout << agents << ',' << 120 << ',' << elapsed << ','
              << elapsed / 120.0 << ',' << engine.worker_count() << '\n';
    return 0;
}
