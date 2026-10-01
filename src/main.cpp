#include <iomanip>
#include <iostream>

#include "scandium/math/vec3.hpp"

int main() {
    using scandium::math::Vec3f;

    const Vec3f forward{0.0f, 0.0f, 1.0f};
    const Vec3f right{1.0f, 0.0f, 0.0f};

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Scandium math smoke test\n";
    std::cout << "dot(forward, right) = " << forward.dot(right) << '\n';

    const auto normal = forward.cross(right);
    std::cout << "cross(forward, right) = ("
              << normal.x << ", "
              << normal.y << ", "
              << normal.z << ")\n";

    std::cout << "normalized(3,4,0) length = "
              << Vec3f{3.0f, 4.0f, 0.0f}.normalized().length() << '\n';

    return 0;
}
