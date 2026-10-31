#include <cassert>
#include <string>

#include "scandium/core/object_pool.hpp"
#include "scandium/ecs/registry.hpp"

struct Particle {
    int value{};
};

struct AgentComponent {
    std::string tag;
    float speed{5.0f};
};

struct ColliderComponent {
    float radius{1.0f};
};

int main() {
    // Test 1: Object Pool
    scandium::core::ObjectPool<Particle> pool{2};
    auto* first = pool.acquire();
    auto* second = pool.acquire();
    assert(first != nullptr);
    assert(second != nullptr);
    assert(pool.owns(first));
    Particle external;
    assert(!pool.owns(&external));
    assert(pool.acquire() == nullptr);
    pool.release(first);
    assert(pool.available() == 1);
    assert(pool.acquire() == first);

    // Test 2: ECS Registry Legacy API
    scandium::ecs::Registry registry;
    const auto a = registry.create();
    const auto b = registry.create();
    registry.transform(a).position = {1.0f, 2.0f, 3.0f};
    assert(registry.alive(a));
    assert(registry.alive(b));
    registry.destroy(b);
    assert(!registry.alive(b));
    assert(registry.active_count() == 1);
    const auto recycled = registry.create();
    assert(recycled == b);
    assert(registry.alive(recycled));
    assert(registry.active_count() == 2);
    assert(registry.transform(recycled).position.x == 0.0f);
    assert(registry.transform(a).position.x == 1.0f);

    // Test 3: Generational Entity Lifecycle
    const auto e1 = registry.create_entity();
    assert(registry.alive(e1));
    const auto e1_old_handle = e1;
    registry.destroy(e1);
    assert(!registry.alive(e1_old_handle));
    const auto e2 = registry.create_entity();
    // Index recycled, but generation incremented
    assert(e2.id == e1.id);
    assert(e2.generation != e1_old_handle.generation);
    assert(registry.alive(e2));
    assert(!registry.alive(e1_old_handle)); // Stale handle protection

    // Test 4: Generic Sparse-Set Component Storage
    registry.emplace<AgentComponent>(a, {"Scout", 7.5f});
    registry.emplace<ColliderComponent>(a, {2.0f});

    assert(registry.has<AgentComponent>(a));
    assert(registry.has<ColliderComponent>(a));
    assert(!registry.has<AgentComponent>(b));

    auto* agent_comp = registry.get<AgentComponent>(a);
    assert(agent_comp != nullptr);
    assert(agent_comp->tag == "Scout");
    assert(agent_comp->speed == 7.5f);

    // Test 5: Multi-Component View
    std::size_t matched_count = 0;
    registry.view<AgentComponent, ColliderComponent>(
        [&](scandium::ecs::EntityId id, AgentComponent& agent, ColliderComponent& collider) {
            assert(id == a);
            assert(agent.tag == "Scout");
            assert(collider.radius == 2.0f);
            ++matched_count;
        });
    assert(matched_count == 1);

    // Test 6: Component removal
    registry.remove<AgentComponent>(a);
    assert(!registry.has<AgentComponent>(a));
    assert(registry.get<AgentComponent>(a) == nullptr);

    return 0;
}
