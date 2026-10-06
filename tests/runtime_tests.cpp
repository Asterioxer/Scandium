#include <cassert>
#include <cmath>
#include <atomic>
#include <cstddef>
#include <string>

#include "scandium/core/event_bus.hpp"
#include "scandium/core/job_system.hpp"
#include "scandium/network/reliability.hpp"
#include "scandium/profiling/trace.hpp"
#include "scandium/runtime/engine.hpp"
#include "scandium/simulation/determinism.hpp"
#include "scandium/simulation/replay.hpp"

struct Ping {
    int value{};
};

int main() {
    using namespace scandium;

    core::EventBus events;
    int received = 0;
    const auto subscription = events.subscribe<Ping>(
        [&](const Ping& ping) { received += ping.value; });
    events.publish(Ping{7});
    assert(received == 7);
    events.unsubscribe<Ping>(subscription);
    events.publish(Ping{3});
    assert(received == 7);

    core::JobSystem jobs(2);
    std::atomic<std::size_t> sum{0};
    jobs.parallel_for(100, [&](std::size_t index) { sum.fetch_add(index + 1); });
    assert(sum.load() == 5050);
    assert(jobs.worker_count() == 2);

    network::AckWindow window;
    window.observe(10);
    window.observe(11);
    window.observe(9);
    assert(window.latest() == 11);
    assert(window.has_seen(10));
    assert(window.has_seen(9));
    assert(!window.has_seen(8));

    profiling::TraceCollector trace;
    {
        profiling::TraceScope scope(trace, "unit");
    }
    assert(trace.events().size() == 1);
    assert(trace.to_chrome_json().find(""ph":"X"") != std::string::npos);

    simulation::Replay replay;
    replay.record({1, 4, -2});
    replay.record({2, 8, 3});
    replay.rewind();
    assert(replay.next()->target_x == 4);
    assert(replay.next()->target_z == 3);
    assert(!replay.has_next());

    runtime::Engine engine{{1.0 / 60.0, 4}, 2};
    engine.world().add_agent({1, {-5.0f, 0.0f, 0.0f}});
    engine.set_target({0.0f, 0.0f, 0.0f});

    std::uint64_t last_hash = 0;
    engine.events().subscribe<runtime::SimulationTickEvent>(
        [&](const runtime::SimulationTickEvent& event) {
            last_hash = event.state_hash;
        });

    engine.advance(1.0 / 60.0);
    const auto snapshot = engine.snapshot();

    assert(engine.clock().total_steps() == 1);
    assert(snapshot.agents.size() == 1);
    assert(snapshot.agents.front().position.x > -5.0f);
    assert(last_hash != 0);
    assert(simulation::hash_agent_state(engine.world()) == last_hash);

    return 0;
}
