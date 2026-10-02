#include <cassert>
#include <cmath>

#include "scandium/ai/steering.hpp"
#include "scandium/math/projection.hpp"
#include "scandium/navigation/jps.hpp"
#include "scandium/profiling/allocation_stats.hpp"

int main() {
    const auto velocity = scandium::ai::seek({0,0,0}, {10,0,0}, 5.0f);
    assert(std::fabs(velocity.x - 5.0f) < 1e-5f);

    const auto projection = scandium::math::perspective(1.0f, 16.0f/9.0f, 0.1f, 100.0f);
    assert(projection(0,0) > 0.0f);
    assert(projection(3,2) == -1.0f);

    scandium::navigation::Grid grid{4,4};
    const auto path = scandium::navigation::JumpPointSearch::find_path(grid, {0,0}, {3,3});
    assert(!path.empty());

    scandium::profiling::AllocationStats stats;
    stats.record_allocation(64);
    stats.record_deallocation(16);
    assert(stats.live_bytes() == 48);

    return 0;
}
