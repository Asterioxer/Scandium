#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include "scandium/math/vec3.hpp"

namespace scandium::render {

struct Color {
    std::uint8_t r{};
    std::uint8_t g{};
    std::uint8_t b{};
};

class Image {
public:
    Image(std::size_t width, std::size_t height, Color clear = {})
        : width_(width), height_(height), pixels_(width * height, clear) {}

    void clear(Color color) {
        std::fill(pixels_.begin(), pixels_.end(), color);
    }

    void set_pixel(int x, int y, Color color) noexcept {
        if (x < 0 || y < 0 ||
            x >= static_cast<int>(width_) ||
            y >= static_cast<int>(height_)) {
            return;
        }
        pixels_[static_cast<std::size_t>(y) * width_ +
                static_cast<std::size_t>(x)] = color;
    }

    void fill_rect(int x, int y, int width, int height, Color color) {
        const int x_end = x + width;
        const int y_end = y + height;
        for (int py = y; py < y_end; ++py) {
            for (int px = x; px < x_end; ++px) {
                set_pixel(px, py, color);
            }
        }
    }

    void fill_circle(int cx, int cy, int radius, Color color) {
        const int radius_sq = radius * radius;
        for (int y = -radius; y <= radius; ++y) {
            for (int x = -radius; x <= radius; ++x) {
                if (x * x + y * y <= radius_sq) {
                    set_pixel(cx + x, cy + y, color);
                }
            }
        }
    }

    void line(int x0, int y0, int x1, int y1, Color color) {
        const int dx = std::abs(x1 - x0);
        const int sx = x0 < x1 ? 1 : -1;
        const int dy = -std::abs(y1 - y0);
        const int sy = y0 < y1 ? 1 : -1;
        int error = dx + dy;

        while (true) {
            set_pixel(x0, y0, color);
            if (x0 == x1 && y0 == y1) {
                break;
            }
            const int twice_error = 2 * error;
            if (twice_error >= dy) {
                error += dy;
                x0 += sx;
            }
            if (twice_error <= dx) {
                error += dx;
                y0 += sy;
            }
        }
    }

    bool write_ppm(const std::string& path) const {
        std::ofstream output(path, std::ios::binary);
        if (!output) {
            return false;
        }

        output << "P6\n" << width_ << ' ' << height_ << "\n255\n";
        for (const auto& pixel : pixels_) {
            output.put(static_cast<char>(pixel.r));
            output.put(static_cast<char>(pixel.g));
            output.put(static_cast<char>(pixel.b));
        }
        return static_cast<bool>(output);
    }

    [[nodiscard]] std::size_t width() const noexcept { return width_; }
    [[nodiscard]] std::size_t height() const noexcept { return height_; }

private:
    std::size_t width_;
    std::size_t height_;
    std::vector<Color> pixels_;
};

} // namespace scandium::render
