#include <cassert>
#include <cstddef>

#include "scandium/game/arena.hpp"

int main() {
    using namespace scandium::game;

    Arena arena;
    std::size_t hits = 0;
    std::size_t eliminations = 0;

    arena.events().subscribe<DamageEvent>(
        [&](const DamageEvent&) { ++hits; });
    arena.events().subscribe<EliminationEvent>(
        [&](const EliminationEvent&) { ++eliminations; });

    arena.spawn(Team::Alpha, {-1.0f, 0.0f, 0.0f});
    arena.spawn(Team::Alpha, {-1.5f, 0.0f, 0.0f});
    arena.spawn(Team::Bravo, {1.0f, 0.0f, 0.0f});

    for (int tick = 0; tick < 60 * 10 &&
         arena.alive_count(Team::Alpha) > 0 &&
         arena.alive_count(Team::Bravo) > 0; ++tick) {
        arena.step(1.0f / 60.0f);
    }

    assert(hits > 0);
    assert(eliminations == 1);
    assert(arena.alive_count(Team::Alpha) == 2);
    assert(arena.alive_count(Team::Bravo) == 0);

    const auto& fighters = arena.combatants();
    assert(fighters.size() == 3);
    assert(fighters[0].score + fighters[1].score + fighters[2].score == 1);

    return 0;
}
