#include <cassert>
#include <cmath>

#include "scandium/ai/steering.hpp"
#include "scandium/math/projection.hpp"
#include "scandium/navigation/jps.hpp"
#include "scandium/profiling/allocation_stats.hpp"


#include "scandium/ai/behavior_tree.hpp"
#include "scandium/ai/perception.hpp"

void test_behavior_tree() {
    using namespace scandium::ai;

    BehaviorTree tree;
    auto seq = std::make_unique<SequenceNode>();

    // 1. Condition: check blackboard "has_target" == true
    seq->add_child(std::make_unique<ConditionNode>([](const Blackboard& bb) {
        return bb.get_or<bool>("has_target", false);
    }));

    // 2. Action: perform attack
    int attack_count = 0;
    seq->add_child(std::make_unique<ActionNode>([&](Blackboard&, float) {
        ++attack_count;
        return NodeStatus::Success;
    }));

    tree.set_root(std::move(seq));

    // Initially has_target is false -> should fail
    assert(tree.tick(0.1f) == NodeStatus::Failure);
    assert(attack_count == 0);

    // Set has_target to true -> should succeed and execute action
    tree.blackboard().set("has_target", true);
    assert(tree.tick(0.1f) == NodeStatus::Success);
    assert(attack_count == 1);

    // Selector test: first child fails, second child succeeds -> overall Success
    auto sel = std::make_unique<SelectorNode>();
    sel->add_child(std::make_unique<ActionNode>([](Blackboard&, float) {
        return NodeStatus::Failure;
    }));
    sel->add_child(std::make_unique<ActionNode>([](Blackboard&, float) {
        return NodeStatus::Success;
    }));
    tree.set_root(std::move(sel));
    assert(tree.tick(0.1f) == NodeStatus::Success);

    // Inverter test: invert Success -> Failure
    auto inv = std::make_unique<InverterNode>(std::make_unique<ActionNode>([](Blackboard&, float) {
        return NodeStatus::Success;
    }));
    tree.set_root(std::move(inv));
    assert(tree.tick(0.1f) == NodeStatus::Failure);

    // Cooldown test
    auto cd = std::make_unique<CooldownNode>(
        std::make_unique<ActionNode>([](Blackboard&, float) { return NodeStatus::Success; }),
        1.0f
    );
    tree.set_root(std::move(cd));
    // First tick succeeds
    assert(tree.tick(0.1f) == NodeStatus::Success);
    // Immediate second tick fails due to cooldown
    assert(tree.tick(0.1f) == NodeStatus::Failure);
    // After advancing 1.0s, succeeds again
    assert(tree.tick(1.0f) == NodeStatus::Success);
}

void test_perception_system() {
    using namespace scandium::ai;
    using scandium::math::Vec3f;

    VisionConfig vision_cfg;
    vision_cfg.fov_degrees = 90.0f;
    vision_cfg.view_distance = 20.0f;
    vision_cfg.peripheral_radius = 2.0f;

    const Vec3f observer_pos{0.0f, 0.0f, 0.0f};
    const Vec3f observer_forward{0.0f, 0.0f, 1.0f}; // Looking along +Z

    // Target directly in front (dist 10m) -> Visible
    assert(PerceptionSystem::can_see(observer_pos, observer_forward, {0.0f, 0.0f, 10.0f}, vision_cfg));

    // Target behind observer (dist 10m, -Z) -> Not visible
    assert(!PerceptionSystem::can_see(observer_pos, observer_forward, {0.0f, 0.0f, -10.0f}, vision_cfg));

    // Target behind observer but within peripheral radius (dist 1m, -Z) -> Visible
    assert(PerceptionSystem::can_see(observer_pos, observer_forward, {0.0f, 0.0f, -1.0f}, vision_cfg));

    // Target beyond view distance (dist 25m) -> Not visible
    assert(!PerceptionSystem::can_see(observer_pos, observer_forward, {0.0f, 0.0f, 25.0f}, vision_cfg));

    // Occlusion raycast test
    auto occlusion_block = [](const scandium::math::Ray&, float max_d) {
        return max_d > 5.0f; // Wall at 5m
    };
    // Target at 10m is blocked by wall
    assert(!PerceptionSystem::can_see(observer_pos, observer_forward, {0.0f, 0.0f, 10.0f}, vision_cfg, occlusion_block));
    // Target at 3m is clear
    assert(PerceptionSystem::can_see(observer_pos, observer_forward, {0.0f, 0.0f, 3.0f}, vision_cfg, occlusion_block));

    // Hearing acoustic attenuation test
    HearingConfig hearing_cfg;
    hearing_cfg.hearing_threshold = 0.2f;
    hearing_cfg.attenuation_rate = 0.05f;

    Stimulus sound;
    sound.type = StimulusType::Acoustic;
    sound.intensity = 1.0f;
    sound.position = {0.0f, 0.0f, 2.0f}; // d^2 = 4 -> I = 1.0 / (1 + 0.2) = 0.833 >= 0.2
    assert(PerceptionSystem::can_hear(observer_pos, sound, hearing_cfg));

    sound.position = {0.0f, 0.0f, 30.0f}; // d^2 = 900 -> I = 1.0 / (1 + 45) = 0.021 < 0.2
    assert(!PerceptionSystem::can_hear(observer_pos, sound, hearing_cfg));

    // Perception memory test
    PerceptionMemory memory;
    memory.register_sensed(42, {10.0f, 0.0f, 5.0f}, {1.0f, 0.0f, 0.0f}, StimulusType::Visual);
    assert(memory.known_entities().size() == 1);
    const auto* entry = memory.find(42);
    assert(entry != nullptr);
    assert(entry->confidence == 1.0f);

    // Update memory by 2.5s (decay half life = 5s)
    memory.update(2.5f, 5.0f);
    assert(std::fabs(memory.find(42)->confidence - 0.5f) < 1e-4f);

    // Update memory by another 3.0s -> total 5.5s >= 5s -> forgotten
    memory.update(3.0f, 5.0f);
    assert(memory.known_entities().empty());
}

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

    test_behavior_tree();
    test_perception_system();
    return 0;
}
