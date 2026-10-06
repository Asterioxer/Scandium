#include <iostream>
#include <string>

#include "scandium/game/arena.hpp"
#include "scandium/render/software_renderer.hpp"

namespace {
int screen_x(float world_x, float radius, int width) {
    return static_cast<int>((world_x / (2.0f * radius) + 0.5f) * width);
}

int screen_y(float world_z, float radius, int height) {
    return static_cast<int>((world_z / (2.0f * radius) + 0.5f) * height);
}
}

int main() {
    using namespace scandium;

    game::Arena arena{{40.0f}};
    for (std::size_t i = 0; i < 8; ++i) {
        arena.spawn(game::Team::Alpha, {
            -18.0f + static_cast<float>(i % 4) * 2.0f,
            0.0f,
            -6.0f + static_cast<float>(i / 4) * 4.0f
        });
        arena.spawn(game::Team::Bravo, {
            18.0f - static_cast<float>(i % 4) * 2.0f,
            0.0f,
            -6.0f + static_cast<float>(i / 4) * 4.0f
        });
    }

    for (int tick = 0; tick < 900 &&
         arena.alive_count(game::Team::Alpha) > 0 &&
         arena.alive_count(game::Team::Bravo) > 0; ++tick) {
        arena.step(1.0f / 60.0f);
    }

    render::Image image(960, 540, {18, 20, 24});
    constexpr float radius = 40.0f;

    for (int x = 0; x < 960; x += 48) {
        image.line(x, 0, x, 539, {34, 37, 43});
    }
    for (int y = 0; y < 540; y += 48) {
        image.line(0, y, 959, y, {34, 37, 43});
    }

    image.fill_circle(480, 270, 8, {210, 210, 210});

    for (const auto& fighter : arena.combatants()) {
        const int x = screen_x(fighter.position.x, radius, 960);
        const int y = 539 - screen_y(fighter.position.z, radius, 540);

        const render::Color color =
            fighter.team == game::Team::Alpha
                ? render::Color{70, 150, 255}
                : render::Color{255, 85, 95};

        if (!fighter.alive) {
            image.fill_circle(x, y, 5, {70, 70, 75});
            continue;
        }

        image.fill_circle(x, y, 8, color);
        image.fill_rect(x - 10, y - 16, 20, 3, {50, 50, 55});
        image.fill_rect(
            x - 10, y - 16,
            static_cast<int>(20.0f * fighter.health / fighter.max_health),
            3, {90, 220, 120});
    }

    const std::string output = "scandium_arena.ppm";
    if (!image.write_ppm(output)) {
        std::cerr << "Failed to write " << output << '\n';
        return 1;
    }

    std::cout << "Rendered Scandium Arena to " << output << '\n';
    return 0;
}
