#include <cassert>

#include "scandium/profiling/frame_stats.hpp"
#include "scandium/simulation/agent_world.hpp"

int main() {
    using namespace scandium::simulation;

    AgentWorld world{5.0f};
    world.add_agent({1, {0.0f, 0.0f, 0.0f}});
    world.add_agent({2, {20.0f, 0.0f, 0.0f}});

    world.update(0.1f, {10.0f, 0.0f, 0.0f});

    assert(world.agents().size() == 2);
    assert(world.agents()[0].position.x > 0.0f);
    assert(world.agents()[0].state == AgentState::Seek);

    scandium::profiling::FrameStats stats;
    stats.record(16.0);
    stats.record(20.0);
    assert(stats.frame_count == 2);
    assert(stats.average_milliseconds() == 18.0);
    assert(stats.max_milliseconds == 20.0);

    return 0;
}
