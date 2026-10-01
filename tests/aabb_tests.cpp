#include <cassert>

#include "scandium/physics/aabb.hpp"

int main() {
    using scandium::math::Vec3f;
    using scandium::physics::Aabb;
    using scandium::physics::merge;

    const Aabb player{{-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}};
    const Aabb obstacle{{0.5f, 0.5f, 0.5f}, {2.0f, 2.0f, 2.0f}};
    const Aabb distant{{5.0f, 5.0f, 5.0f}, {6.0f, 6.0f, 6.0f}};

    assert(player.intersects(obstacle));
    assert(!player.intersects(distant));

    assert(player.contains(Vec3f{0.0f, 0.0f, 0.0f}));
    assert(!player.contains(Vec3f{1.1f, 0.0f, 0.0f}));

    const auto combined = merge(player, distant);
    assert(combined.min.x == -1.0f);
    assert(combined.min.y == -1.0f);
    assert(combined.max.z == 6.0f);

    return 0;
}
