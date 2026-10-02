#include <cassert>

#include "scandium/core/object_pool.hpp"
#include "scandium/ecs/registry.hpp"

struct Particle {
    int value{};
};

int main() {
    scandium::core::ObjectPool<Particle> pool{2};
    auto* first = pool.acquire();
    auto* second = pool.acquire();
    assert(first != nullptr);
    assert(second != nullptr);
    assert(pool.acquire() == nullptr);
    pool.release(first);
    assert(pool.available() == 1);
    assert(pool.acquire() == first);

    scandium::ecs::Registry registry;
    const auto a = registry.create();
    const auto b = registry.create();
    registry.transform(a).position = {1.0f, 2.0f, 3.0f};
    assert(registry.alive(a));
    assert(registry.alive(b));
    registry.destroy(b);
    assert(!registry.alive(b));
    assert(registry.transform(a).position.x == 1.0f);

    return 0;
}
