#include <cstddef>
#include <iostream>

#include "scandium/game/arena.hpp"

int main() {
    using namespace scandium::game;

    Arena arena;
    std::size_t eliminations = 0;

    arena.events().subscribe<EliminationEvent>(
        [&](const EliminationEvent&) { ++eliminations; });

    for (std::size_t i = 0; i < 8; ++i) {
        arena.spawn(Team::Alpha, {
            -18.0f + static_cast<float>(i % 4) * 2.0f,
            0.0f,
            -6.0f + static_cast<float>(i / 4) * 4.0f
        });
        arena.spawn(Team::Bravo, {
            18.0f - static_cast<float>(i % 4) * 2.0f,
            0.0f,
            -6.0f + static_cast<float>(i / 4) * 4.0f
        });
    }

    for (int tick = 0; tick < 1800 &&
         arena.alive_count(Team::Alpha) > 0 &&
         arena.alive_count(Team::Bravo) > 0; ++tick) {
        arena.step(1.0f / 60.0f);
    }

    std::cout << "SCANDIUM ARENA\n";
    std::cout << "==============\n";
    std::cout << "Alpha alive: " << arena.alive_count(Team::Alpha) << '\n';
    std::cout << "Bravo alive: " << arena.alive_count(Team::Bravo) << '\n';
    std::cout << "Eliminations: " << eliminations << '\n';

    return 0;
}
