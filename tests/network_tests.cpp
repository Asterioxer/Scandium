#include <cassert>
#include <deque>
#include <iostream>
#include <random>
#include <vector>

#include "scandium/network/lockstep.hpp"
#include "scandium/network/reliability.hpp"
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

void test_ack_window_adversity() {
    std::cout << "Starting test_ack_window_adversity..." << std::endl;
    AckWindow window;

    // Out of order: 1, 2, 5, 3, 4
    window.observe(1);
    assert(window.latest() == 1);
    assert(window.has_seen(1));
    assert(!window.has_seen(2));

    window.observe(2);
    assert(window.latest() == 2);
    assert(window.has_seen(1));
    assert(window.has_seen(2));

    // Jump forward to 5 (missing 3 and 4)
    window.observe(5);
    assert(window.latest() == 5);
    assert(window.has_seen(5));
    assert(window.has_seen(2));
    assert(window.has_seen(1));
    assert(!window.has_seen(3));
    assert(!window.has_seen(4));

    // Late packets arrive: 3 and 4
    window.observe(3);
    assert(window.has_seen(3));
    assert(!window.has_seen(4));

    window.observe(4);
    assert(window.has_seen(4));
    assert(window.has_seen(5));

    // Duplicate observe of 3
    window.observe(3);
    assert(window.has_seen(3));

    // Large jump forward (> 32)
    window.observe(100);
    assert(window.latest() == 100);
    assert(window.has_seen(100));
    assert(!window.has_seen(5)); // Displaced from 32-bit history

    std::cout << "Passed test_ack_window_adversity." << std::endl;
}

void test_simulated_network_adversity_rollback() {
    std::cout << "Starting test_simulated_network_adversity_rollback..." << std::endl;

    // Authoritative Server World
    TestSimWorld server_world;

    // Speculative Client World
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

    RollbackController<TestSimWorld, TestSimWorld::Snapshot, InputFrame, 128> controller(
        client_world, {1, 2}, 1, step_simulation
    );

    constexpr std::uint64_t total_ticks = 60;
    std::vector<InputFrame> p1_inputs;
    std::vector<InputFrame> p2_inputs;
    p1_inputs.reserve(total_ticks);
    p2_inputs.reserve(total_ticks);

    for (std::uint64_t t = 0; t < total_ticks; ++t) {
        p1_inputs.push_back(InputFrame{t, static_cast<std::int16_t>(1 + (t % 3)), 0});
        // Player 2 changes direction every 5 ticks
        p2_inputs.push_back(InputFrame{t, static_cast<std::int16_t>((t / 5) % 2 == 0 ? 2 : -1), 0});
    }

    // 1. Run authoritative server simulation
    for (std::uint64_t t = 0; t < total_ticks; ++t) {
        std::unordered_map<std::uint32_t, InputFrame> server_inputs;
        server_inputs[1] = p1_inputs[t];
        server_inputs[2] = p2_inputs[t];
        step_simulation(server_world, t, server_inputs);
    }

    // 2. Run client simulation with simulated network adversity:
    // Packet delay buffer: (arrival_tick, player_id, input_frame)
    struct DelayedPacket {
        std::uint64_t arrival_tick;
        std::uint32_t player_id;
        InputFrame input;
    };
    std::vector<DelayedPacket> in_flight_packets;

    std::mt19937 rng(1337);
    std::uniform_int_distribution<int> drop_dist(0, 9); // 0..9
    std::uniform_int_distribution<int> jitter_dist(1, 4); // 1..4 ticks delay

    for (std::uint64_t t = 0; t < total_ticks; ++t) {
        // Client predicts local input immediately
        controller.predict_local_tick(p1_inputs[t]);

        // Server transmits player 2 input across adverse network
        // 20% packet drop rate (if drop_dist < 2, packet lost!)
        if (drop_dist(rng) >= 2) {
            const std::uint64_t delay = static_cast<std::uint64_t>(jitter_dist(rng));
            in_flight_packets.push_back({t + delay, 2, p2_inputs[t]});
        }

        // Deliver any packets scheduled for arrival on or before tick t
        for (auto it = in_flight_packets.begin(); it != in_flight_packets.end();) {
            if (it->arrival_tick <= t) {
                controller.receive_authoritative_input(it->player_id, it->input.tick, it->input);
                it = in_flight_packets.erase(it);
            } else {
                ++it;
            }
        }
    }

    // Deliver all remaining in-flight packets (e.g. at end of round)
    // and deliver any dropped packets via reliable resend
    for (const auto& pkt : in_flight_packets) {
        controller.receive_authoritative_input(pkt.player_id, pkt.input.tick, pkt.input);
    }
    in_flight_packets.clear();

    // Resend any ticks player 2 missed due to drops
    for (std::uint64_t t = 0; t < total_ticks; ++t) {
        controller.receive_authoritative_input(2, t, p2_inputs[t]);
    }

    // Assert that client-side prediction and rollback converged to bit-exact match with server
    assert(client_world.x1 == server_world.x1);
    assert(client_world.x2 == server_world.x2);
    assert(controller.total_rollbacks() > 0); // Rollbacks occurred and corrected state!

    std::cout << "Passed test_simulated_network_adversity_rollback (total rollbacks: "
              << controller.total_rollbacks()
              << ", max frames rewound: " << controller.max_rollback_frames() << ")." << std::endl;
}

int main() {
    test_lockstep_session();
    test_state_ring_buffer();
    test_client_side_prediction_and_rollback();
    test_ack_window_adversity();
    test_simulated_network_adversity_rollback();
    std::cout << "All network and rollback tests passed successfully." << std::endl;
    return 0;
}
