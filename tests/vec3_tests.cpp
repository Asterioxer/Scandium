#include <cassert>
#include <cmath>

#include "scandium/math/vec3.hpp"

namespace {
bool near(float a, float b, float epsilon = 1e-5f) {
    return std::fabs(a - b) <= epsilon;
}
}

int main() {
    using scandium::math::Vec3f;

    const Vec3f a{1.0f, 2.0f, 3.0f};
    const Vec3f b{4.0f, -2.0f, 1.0f};

    const auto sum = a + b;
    assert(sum.x == 5.0f);
    assert(sum.y == 0.0f);
    assert(sum.z == 4.0f);

    assert(near(a.dot(b), 3.0f));

    const auto cross = a.cross(b);
    assert(near(cross.x, 8.0f));
    assert(near(cross.y, 11.0f));
    assert(near(cross.z, -10.0f));

    const auto normalized = Vec3f{3.0f, 4.0f, 0.0f}.normalized();
    assert(near(normalized.x, 0.6f));
    assert(near(normalized.y, 0.8f));
    assert(near(normalized.z, 0.0f));
    assert(near(normalized.length(), 1.0f));

    const auto zero = Vec3f{}.normalized();
    assert(zero.x == 0.0f && zero.y == 0.0f && zero.z == 0.0f);

    return 0;
}
