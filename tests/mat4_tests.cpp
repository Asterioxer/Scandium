#include <cassert>
#include <cmath>

#include "scandium/math/mat4.hpp"
#include "scandium/math/quaternion.hpp"
#include "scandium/math/transform.hpp"

namespace {
bool near(float a, float b, float epsilon = 1e-5f) {
    return std::fabs(a - b) <= epsilon;
}
}

int main() {
    using scandium::math::Mat4;
    using scandium::math::Quaternion;
    using scandium::math::Transform;
    using scandium::math::Vec3f;

    const auto translated = Mat4::translation({10.0f, -2.0f, 4.0f});
    const auto point = translated.transform_point({1.0f, 2.0f, 3.0f});

    assert(near(point.x, 11.0f));
    assert(near(point.y, 0.0f));
    assert(near(point.z, 7.0f));

    const auto scaled = Mat4::scale({2.0f, 3.0f, 4.0f});
    const auto scaled_point = scaled.transform_point({1.0f, 2.0f, 3.0f});

    assert(near(scaled_point.x, 2.0f));
    assert(near(scaled_point.y, 6.0f));
    assert(near(scaled_point.z, 12.0f));

    const Transform transform{{5.0f, 6.0f, 7.0f}, Quaternion::identity(), {2.0f, 2.0f, 2.0f}};
    const Vec3f result = transform.transform_point({1.0f, 1.0f, 1.0f});

    assert(near(result.x, 7.0f));
    assert(near(result.y, 8.0f));
    assert(near(result.z, 9.0f));

    const auto quarter_turn = Quaternion::from_axis_angle(
        {0.0f, 0.0f, 1.0f}, 3.1415926535f * 0.5f);
    const auto rotated = quarter_turn.rotate({1.0f, 0.0f, 0.0f});
    assert(near(rotated.x, 0.0f));
    assert(near(rotated.y, 1.0f));

    const auto rotated_matrix = Mat4::rotation(quarter_turn).transform_point(
        {1.0f, 0.0f, 0.0f});
    assert(near(rotated_matrix.x, 0.0f));
    assert(near(rotated_matrix.y, 1.0f));

    return 0;
}
