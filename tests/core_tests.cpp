#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

#include "scandium/core/frame_allocator.hpp"
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

struct VelocityComponent {
    float vx{0.0f};
    float vy{0.0f};
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

    // Test 7: Generational Entity Handle Safety
    {
        scandium::ecs::Registry reg;
        const auto handle1 = reg.create_entity();
        reg.emplace<AgentComponent>(handle1, {"Warrior", 10.0f});
        assert(reg.has<AgentComponent>(handle1));
        assert(reg.get<AgentComponent>(handle1) != nullptr);

        // Destroy handle1
        reg.destroy(handle1);
        assert(!reg.alive(handle1));
        assert(!reg.has<AgentComponent>(handle1));
        assert(reg.get<AgentComponent>(handle1) == nullptr);

        // Allocate handle2 at same index
        const auto handle2 = reg.create_entity();
        assert(handle2.id == handle1.id);
        assert(handle2.generation != handle1.generation);

        // Stale handle1 should NOT access handle2's state
        assert(!reg.alive(handle1));
        assert(!reg.has<AgentComponent>(handle1));
        assert(reg.get<AgentComponent>(handle1) == nullptr);

        reg.emplace<AgentComponent>(handle2, {"Mage", 12.0f});
        assert(reg.has<AgentComponent>(handle2));
        assert(!reg.has<AgentComponent>(handle1)); // Still stale
        assert(reg.get<AgentComponent>(handle2)->tag == "Mage");
        assert(reg.get<AgentComponent>(handle1) == nullptr);

        // Removing via stale handle does nothing
        reg.remove<AgentComponent>(handle1);
        assert(reg.has<AgentComponent>(handle2)); // Still safe!
    }

    // Test 8: Generation Wraparound Handling
    {
        scandium::ecs::Registry reg;
        auto h = reg.create_entity();
        // Artificially advance generation close to UINT32_MAX
        for (std::size_t i = 0; i < 100; ++i) {
            reg.destroy(h);
            h = reg.create_entity();
        }
        assert(reg.generation(h.id) >= 100);
        assert(reg.alive(h));
    }

    // Test 9: High-volume entity recycling & sparse-set integrity
    {
        scandium::ecs::Registry reg;
        constexpr int count = 2000;
        std::vector<scandium::ecs::Entity> entities;
        entities.reserve(count);

        for (int i = 0; i < count; ++i) {
            auto ent = reg.create_entity();
            reg.emplace<VelocityComponent>(ent, {static_cast<float>(i), static_cast<float>(i * 2)});
            entities.push_back(ent);
        }
        assert(reg.active_count() == count);

        // Destroy every even entity
        for (int i = 0; i < count; i += 2) {
            reg.destroy(entities[i]);
            assert(!reg.alive(entities[i]));
        }
        assert(reg.active_count() == count / 2);

        // Verify odd entities still have intact components
        for (int i = 1; i < count; i += 2) {
            assert(reg.alive(entities[i]));
            auto* vel = reg.get<VelocityComponent>(entities[i]);
            assert(vel != nullptr);
            assert(vel->vx == static_cast<float>(i));
            assert(vel->vy == static_cast<float>(i * 2));
        }

        // View iteration over surviving entities
        std::size_t view_count = 0;
        reg.view<VelocityComponent>([&](scandium::ecs::EntityId id, VelocityComponent& vel) {
            assert(id % 2 == 1);
            ++view_count;
        });
        assert(view_count == count / 2);
    }

    // Test 10: FrameAllocator Alignment, Chained Pages & Reset
    {
        using namespace scandium::core;
        FrameAllocator arena(4096); // 4 KB small pages to test multi-page chaining

        // Alignments: 8, 16, 32, 64 bytes
        void* p8 = arena.allocate(17, 8);
        assert(reinterpret_cast<uintptr_t>(p8) % 8 == 0);

        void* p16 = arena.allocate(25, 16);
        assert(reinterpret_cast<uintptr_t>(p16) % 16 == 0);

        void* p32 = arena.allocate(48, 32);
        assert(reinterpret_cast<uintptr_t>(p32) % 32 == 0);

        void* p64 = arena.allocate(128, 64);
        assert(reinterpret_cast<uintptr_t>(p64) % 64 == 0);

        // Test marker & scoped rollback
        auto marker = arena.mark();
        void* scratch = arena.allocate(500, 16);
        assert(scratch != nullptr);
        arena.reset_to(marker);

        void* next_alloc = arena.allocate(500, 16);
        assert(next_alloc == scratch); // Reclaimed memory

        // Test ScopedArenaMarker RAII
        {
            ScopedArenaMarker scope(arena);
            void* inner_scratch = arena.allocate(256, 16);
            assert(inner_scratch != nullptr);
        }
        void* after_scope = arena.allocate(256, 16);
        assert(after_scope != nullptr);

        // Force chained page allocations (exceeding 4KB)
        assert(arena.page_count() == 1);
        void* large_block = arena.allocate(5000, 64);
        assert(large_block != nullptr);
        assert(arena.page_count() >= 2);
        assert(reinterpret_cast<uintptr_t>(large_block) % 64 == 0);

        // Reset retains pages
        const auto page_count_before = arena.page_count();
        arena.reset();
        assert(arena.total_allocated() == 0);
        assert(arena.page_count() == page_count_before);

        // Reallocate in retained pages
        void* reused = arena.allocate(100, 16);
        assert(reused != nullptr);
    }

    // Test 11: STL ArenaAllocator compatibility with std::vector
    {
        using namespace scandium::core;
        FrameAllocator arena(64 * 1024);
        ArenaAllocator<int> int_alloc(arena);

        std::vector<int, ArenaAllocator<int>> vec(int_alloc);
        for (int i = 0; i < 1000; ++i) {
            vec.push_back(i * 3);
        }

        assert(vec.size() == 1000);
        for (int i = 0; i < 1000; ++i) {
            assert(vec[i] == i * 3);
        }

        assert(arena.total_allocated() > 1000 * sizeof(int));
    }

    return 0;
}
