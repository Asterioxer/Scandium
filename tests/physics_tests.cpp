#include <cassert>
#include <cmath>

#include "scandium/math/quaternion.hpp"
#include "scandium/math/ray.hpp"
#include "scandium/physics/ray_intersection.hpp"
#include "scandium/spatial/spatial_hash.hpp"

int main() {
    using scandium::math::Quaternion;
    using scandium::math::Ray;
    using scandium::math::Vec3f;
    using scandium::physics::Aabb;
    using scandium::physics::intersect_ray_aabb;
    using scandium::spatial::SpatialHash;

    const float half_pi = 1.57079632679f;
    const auto rotation = Quaternion::from_axis_angle({0.0f, 1.0f, 0.0f}, half_pi);
    const auto rotated = rotation.rotate({1.0f, 0.0f, 0.0f});
    assert(std::fabs(rotated.x) < 1e-4f);
    assert(std::fabs(rotated.z + 1.0f) < 1e-4f);

    const Aabb target{{2.0f, -1.0f, -1.0f}, {4.0f, 1.0f, 1.0f}};
    const Ray ray{{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
    const auto hit = intersect_ray_aabb(ray, target);
    assert(hit.has_value());
    assert(std::fabs(*hit - 2.0f) < 1e-5f);

    SpatialHash hash{10.0f};
    hash.insert(7, {1.0f, 1.0f, 1.0f});
    hash.insert(9, {8.0f, 1.0f, 1.0f});
    hash.insert(42, {100.0f, 100.0f, 100.0f});

    const auto nearby = hash.query({2.0f, 2.0f, 2.0f});
    bool found7 = false;
    bool found9 = false;
    for (const auto id : nearby) {
        found7 |= id == 7;
        found9 |= id == 9;
    }
    assert(found7 && found9);

    (void)Vec3f{};
    return 0;
}
