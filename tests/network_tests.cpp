#include <cassert>
#include <iostream>
#include <vector>

#include "scandium/network/lockstep.hpp"
#include "scandium/network/rollback.hpp"
#include "scandium/simulation/agent_world.hpp"

using namespace scandium;
using namespace scandium::network;

void test_lockstep_session() {
    LockstepSession session{{10, 20}};
    session.submit(10, {7, 1, 0});
    assert(!session.ready(7));

    session.submit(20, {7, 0, 1});
    assert(session.ready(7));

    const auto frames = session.consume(7);
    assert(frames.has_value());
    assert(frames->size() == 2);
    assert((*frames)[0].axis_x == 1);
    assert((*frames)[1].axis_y == 1);
    assert(!session.ready(7));

    session.submit(999, {8, 4, 4});
    assert(!session.ready(8));
    std::cout << "Passed test_lockstep_session." << std::endl;
}

void test_state_ring_buffer() {
    StateRingBuffer<int, 64> buffer;
    assert(buffer.empty());

    for (std::uint64_t t = 0; t < 200; ++t) {
        buffer.store(t, static_cast<int>(t * 10));
        assert(buffer.contains(t));
        assert(*buffer.get(t) == static_cast<int>(t * 10));
    }

    assert(buffer.newest_tick() == 199);
    // Capacity is 64, so tick 199 wraps; ticks older than 199 - 63 should be overwritten
    assert(!buffer.contains(100));
    assert(buffer.contains(199));
    assert(buffer.contains(199 - 63));

    buffer.clear();
    assert(buffer.empty());
    assert(!buffer.contains(199));
    std::cout << "Passed test_state_ring_buffer." << std::endl;
}

struct TestSimWorld {
    float x1{0.0f};
    float x2{0.0f};

    struct Snapshot {
        std::uint64_t tick{0};
        float x1{0.0f};
        float x2{0.0f};
    };

    [[nodiscard]] Snapshot capture_snapshot(std::uint64_t tick) const {
        return Snapshot{tick, x1, x2};
    }

    void restore_snapshot(const Snapshot& s) {
        x1 = s.x1;
        x2 = s.x2;
    }
};

void test_client_side_prediction_and_rollback() {
    TestSimWorld client_world;

    auto step_simulation = [](TestSimWorld& world, std::uint64_t, const std::unordered_map<std::uint32_t, InputFrame>& inputs) {
        const auto it1 = inputs.find(1);
        if (it1 != inputs.end()) {
            world.x1 += static_cast<float>(it1->second.axis_x);
        }
        const auto it2 = inputs.find(2);
        if (it2 != inputs.end()) {
            world.x2 += static_cast<float>(it2->second.axis_x);
        }
    };

    RollbackController<TestSimWorld, TestSimWorld::Snapshot, InputFrame, 64> controller(
        client_world, {1, 2}, 1, step_simulation
    );

    // Predict 10 ticks locally: player 1 moves +1 each tick, player 2 is speculated as 0
    for (std::uint64_t t = 0; t < 10; ++t) {
        controller.predict_local_tick(InputFrame{t, 1, 0});
    }

    assert(controller.current_tick() == 10);
    assert(client_world.x1 == 10.0f);
    assert(client_world.x2 == 0.0f); // remote speculated as 0
    assert(controller.total_predictions() == 10);
    assert(controller.total_rollbacks() == 0);

    // Authoritative frame arrives for tick 5: player 2 actually moved +5 at tick 5!
    const bool rolled_back = controller.receive_authoritative_input(2, 5, InputFrame{5, 5, 0});
    assert(rolled_back);
    assert(controller.total_rollbacks() == 1);
    assert(controller.max_rollback_frames() == 5);

    // After rollback and resimulation, player 1 is still at 10.0f, and player 2 is now at 5.0f
    assert(client_world.x1 == 10.0f);
    assert(client_world.x2 == 5.0f);

    // Another authoritative frame arrives for tick 5 with matching input -> no rollback
    const bool second_rollback = controller.receive_authoritative_input(2, 5, InputFrame{5, 5, 0});
    assert(!second_rollback);
    assert(controller.total_rollbacks() == 1);

    std::cout << "Passed test_client_side_prediction_and_rollback." << std::endl;
}

int main() {
    test_lockstep_session();
    test_state_ring_buffer();
    test_client_side_prediction_and_rollback();
    std::cout << "All network and rollback tests passed successfully." << std::endl;
    return 0;
}
