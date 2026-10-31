#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>

#include "scandium/runtime/engine.hpp"
#include "scandium/simulation/agent_world.hpp"
#include "scandium/simulation/determinism.hpp"
#include "scandium/simulation/replay.hpp"

using namespace scandium;

void test_replay_serialization() {
    simulation::Replay original;
    original.record({0, 10, -5});
    original.record({1, 20, 15});
    original.record({2, -30, 42});

    const std::string serialized = original.to_string();
    assert(!serialized.empty());

    simulation::Replay loaded;
    const bool success = loaded.from_string(serialized);
    assert(success);
    assert(loaded.size() == 3);

    for (std::size_t i = 0; i < 3; ++i) {
        assert(loaded.commands()[i] == original.commands()[i]);
    }

    const auto* cmd = loaded.find_command_for_tick(1);
    assert(cmd != nullptr);
    assert(cmd->target_x == 20);
    assert(cmd->target_z == 15);

    assert(loaded.find_command_for_tick(999) == nullptr);
}

void test_mismatch_diagnostics() {
    simulation::AgentWorld world1{4.0f};
    simulation::AgentWorld world2{4.0f};

    world1.add_agent({1, {1.0f, 0.0f, 1.0f}, {}, {2.0f, 0.0f, 0.0f}, simulation::AgentState::Seek, 5.0f});
    world2.add_agent({1, {1.0f, 0.0f, 1.0f}, {}, {2.0f, 0.0f, 0.0f}, simulation::AgentState::Seek, 5.0f});

    auto report = simulation::diagnose_state_mismatch(world1, world2);
    assert(report.matches);
    assert(report.mismatches.empty());

    // Introduce difference
    world2.agents_mutable()[0].position.x += 0.05f;
    report = simulation::diagnose_state_mismatch(world1, world2);
    assert(!report.matches);
    assert(report.mismatches.size() == 1);
    assert(report.mismatches[0].agent_id == 1);
    assert(report.mismatches[0].field_name == "position.x");
    assert(!report.to_string().empty());
}

void test_serialization_edge_cases() {
    // 1. Empty replay
    {
        simulation::Replay empty;
        const std::string serialized = empty.to_string();
        simulation::Replay loaded;
        assert(loaded.from_string(serialized));
        assert(loaded.size() == 0);
        assert(!loaded.has_next());
    }

    // 2. Large replay (50,000 commands)
    {
        simulation::Replay large;
        constexpr std::size_t large_count = 50000;
        for (std::size_t i = 0; i < large_count; ++i) {
            large.record({i, static_cast<std::int32_t>(i % 100), static_cast<std::int32_t>(i % 200)});
        }
        const std::string serialized = large.to_string();
        simulation::Replay loaded;
        assert(loaded.from_string(serialized));
        assert(loaded.size() == large_count);
        assert(loaded.commands()[0] == large.commands()[0]);
        assert(loaded.commands()[large_count - 1] == large.commands()[large_count - 1]);
    }

    // 3. Truncated files
    {
        simulation::Replay r;
        assert(!r.from_string("")); // completely empty
        assert(!r.from_string("SCANDIUM_REPLAY_V1")); // missing count
        assert(!r.from_string("SCANDIUM_REPLAY_V1\n2\n0 10")); // missing z of command 0 and command 1
        assert(!r.from_string("SCANDIUM_REPLAY_V1\n1\n0 10")); // missing z
    }

    // 4. Corrupted files / malformed tokens
    {
        simulation::Replay r;
        assert(!r.from_string("SCANDIUM_REPLAY_V1\n1\nnot_a_number 10 20"));
        assert(!r.from_string("SCANDIUM_REPLAY_V1\n1\n0 abc 20"));
        assert(!r.from_string("SCANDIUM_REPLAY_V1\ngarbage_count\n0 10 20"));
        assert(!r.from_string("RANDOM_CORRUPT_BYTES_XYZ_123456789"));
    }

    // 5. Unsupported version
    {
        simulation::Replay r;
        assert(!r.from_string("SCANDIUM_REPLAY_V2\n1\n0 10 20"));
        assert(!r.from_string("SCANDIUM_REPLAY_V0\n0\n"));
        assert(!r.from_string("OTHER_HEADER_FORMAT\n1\n0 10 20"));
    }

    // 6. Invalid counts
    {
        simulation::Replay r;
        // Count larger than reasonable limit (10,000,000)
        assert(!r.from_string("SCANDIUM_REPLAY_V1\n99999999999999999\n0 10 20"));
        // Negative count interpreted as massive uint
        assert(!r.from_string("SCANDIUM_REPLAY_V1\n-5\n0 10 20"));
        // Count claims 5 but only 1 entry present
        assert(!r.from_string("SCANDIUM_REPLAY_V1\n5\n0 10 20\n"));
    }

    // 7. File I/O failures
    {
        simulation::Replay r;
        // Nonexistent file load
        assert(!r.load_from_file("nonexistent_directory_xyz/impossible_replay_file.bin"));
        // Save to invalid path
        assert(!r.save_to_file(""));
#if defined(_WIN32)
        assert(!r.save_to_file("Z:/impossible_volume_path/cannot_write.bin"));
#endif
    }
}

void test_repeatable_simulation_determinism() {
    constexpr std::size_t agent_count = 128;
    constexpr int frames = 60;

    auto run_sim = [](std::size_t workers) -> std::vector<std::uint64_t> {
        runtime::Engine engine{{1.0 / 60.0, 8}, workers};
        for (std::size_t i = 0; i < agent_count; ++i) {
            const float x = static_cast<float>(i % 16) * 2.0f - 15.0f;
            const float z = static_cast<float>(i / 16) * 2.0f - 7.0f;
            engine.world().add_agent({
                static_cast<std::uint32_t>(i),
                {x, 0.0f, z},
                {},
                {},
                simulation::AgentState::Idle,
                4.0f
            });
        }

        std::vector<std::uint64_t> hashes;
        engine.events().subscribe<runtime::SimulationTickEvent>(
            [&](const runtime::SimulationTickEvent& event) {
                hashes.push_back(event.state_hash);
            });

        for (int frame = 0; frame < frames; ++frame) {
            // Apply deterministic inputs at specific ticks
            if (frame == 10) {
                engine.set_target({10.0f, 0.0f, 5.0f});
            } else if (frame == 30) {
                engine.set_target({-15.0f, 0.0f, -10.0f});
            }
            engine.advance(1.0 / 60.0);
        }

        return hashes;
    };

    const auto h1 = run_sim(1);
    const auto h2 = run_sim(2);
    const auto h4 = run_sim(4);
    const auto h8 = run_sim(8);
    const auto h16 = run_sim(16);

    assert(h1.size() == frames);
    assert(h2.size() == frames);
    assert(h4.size() == frames);
    assert(h8.size() == frames);
    assert(h16.size() == frames);

    for (std::size_t i = 0; i < frames; ++i) {
        if (h1[i] != h2[i] || h1[i] != h4[i] || h1[i] != h8[i] || h1[i] != h16[i]) {
            std::cerr << "Determinism mismatch at tick " << i << ": "
                      << "1 worker=" << h1[i] << ", "
                      << "2 workers=" << h2[i] << ", "
                      << "4 workers=" << h4[i] << ", "
                      << "8 workers=" << h8[i] << ", "
                      << "16 workers=" << h16[i] << "\n";
            assert(false && "State hash mismatch across worker thread counts!");
        }
    }
}

int main() {
    test_replay_serialization();
    test_serialization_edge_cases();
    test_mismatch_diagnostics();
    test_repeatable_simulation_determinism();
    std::cout << "All determinism and serialization tests passed successfully across 1, 2, 4, 8, 16 workers.\n";
    return 0;
}
