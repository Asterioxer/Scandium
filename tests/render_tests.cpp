#include <cassert>

#include "scandium/render/software_renderer.hpp"

int main() {
    using namespace scandium::render;

    Image image(32, 16, {1, 2, 3});
    assert(image.width() == 32);
    assert(image.height() == 16);

    image.set_pixel(4, 5, {9, 8, 7});
    const auto pixel = image.pixel(4, 5);
    assert(pixel.r == 9);
    assert(pixel.g == 8);
    assert(pixel.b == 7);

    image.fill_circle(16, 8, 2, {20, 30, 40});
    assert(image.pixel(16, 8).r == 20);

    return 0;
}
