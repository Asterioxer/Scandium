#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "scandium/core/event_bus.hpp"
#include "scandium/math/vec3.hpp"

namespace scandium::game {

enum class Team : std::uint8_t {
    Alpha,
    Bravo
};

struct Combatant {
    std::uint32_t id{};
    Team team{Team::Alpha};
    math::Vec3f position{};
    math::Vec3f velocity{};
    float health{100.0f};
    float max_health{100.0f};
    float move_speed{3.0f};
    float attack_range{3.0f};
    float attack_damage{10.0f};
    float attack_cooldown{0.5f};
    float attack_timer{};
    std::uint32_t score{};
    bool alive{true};
};

struct DamageEvent {
    std::uint32_t attacker{};
    std::uint32_t victim{};
    float amount{};
};

struct EliminationEvent {
    std::uint32_t attacker{};
    std::uint32_t victim{};
};

struct ArenaConfig {
    float arena_radius{40.0f};
    float separation_epsilon{0.001f};
};

class Arena {
public:
    explicit Arena(ArenaConfig config = {}) : config_(config) {}

    std::uint32_t spawn(Team team, math::Vec3f position) {
        const std::uint32_t id = next_id_++;
        combatants_.push_back({
            id, team, position, {}, 100.0f, 100.0f,
            3.0f, 3.0f, 10.0f, 0.5f, 0.0f, 0, true
        });
        return id;
    }

    void step(float dt) {
        struct PendingDamage {
            std::uint32_t attacker;
            std::uint32_t victim;
            float amount;
        };

        std::vector<PendingDamage> damage;
        damage.reserve(combatants_.size());

        for (auto& attacker : combatants_) {
            if (!attacker.alive) {
                continue;
            }

            attacker.attack_timer = std::max(0.0f, attacker.attack_timer - dt);

            const Combatant* target = nearest_enemy(attacker);
            if (target == nullptr) {
                attacker.velocity = {};
                continue;
            }

            const math::Vec3f delta = target->position - attacker.position;
            const float distance_sq = delta.length_squared();

            if (distance_sq > attacker.attack_range * attacker.attack_range) {
                attacker.velocity = delta.normalized() * attacker.move_speed;
                attacker.position += attacker.velocity * dt;
                clamp_to_arena(attacker.position);
                continue;
            }

            attacker.velocity = {};
            if (attacker.attack_timer <= 0.0f) {
                damage.push_back({
                    attacker.id, target->id, attacker.attack_damage
                });
                attacker.attack_timer = attacker.attack_cooldown;
            }
        }

        for (const auto& hit : damage) {
            apply_damage(hit.attacker, hit.victim, hit.amount);
        }
    }

    [[nodiscard]] const std::vector<Combatant>& combatants() const noexcept {
        return combatants_;
    }

    [[nodiscard]] std::size_t alive_count(Team team) const noexcept {
        std::size_t count = 0;
        for (const auto& combatant : combatants_) {
            if (combatant.alive && combatant.team == team) {
                ++count;
            }
        }
        return count;
    }

    [[nodiscard]] core::EventBus& events() noexcept {
        return events_;
    }

private:
    [[nodiscard]] const Combatant* nearest_enemy(
        const Combatant& source) const noexcept {

        const Combatant* best = nullptr;
        float best_distance = std::numeric_limits<float>::max();

        for (const auto& candidate : combatants_) {
            if (!candidate.alive || candidate.team == source.team) {
                continue;
            }

            const float distance = (candidate.position - source.position).length_squared();
            if (distance < best_distance ||
                (distance == best_distance && best != nullptr &&
                 candidate.id < best->id)) {
                best_distance = distance;
                best = &candidate;
            }
        }

        return best;
    }

    void apply_damage(std::uint32_t attacker_id,
                      std::uint32_t victim_id,
                      float amount) {
        auto* victim = find(victim_id);
        if (victim == nullptr || !victim->alive) {
            return;
        }

        victim->health = std::max(0.0f, victim->health - amount);
        events_.publish(DamageEvent{attacker_id, victim_id, amount});

        if (victim->health <= config_.separation_epsilon) {
            victim->alive = false;
            victim->velocity = {};

            if (auto* attacker = find(attacker_id); attacker != nullptr) {
                ++attacker->score;
            }

            events_.publish(EliminationEvent{attacker_id, victim_id});
        }
    }

    Combatant* find(std::uint32_t id) noexcept {
        for (auto& combatant : combatants_) {
            if (combatant.id == id) {
                return &combatant;
            }
        }
        return nullptr;
    }

    void clamp_to_arena(math::Vec3f& position) const noexcept {
        const float distance_sq = position.length_squared();
        const float radius_sq = config_.arena_radius * config_.arena_radius;
        if (distance_sq <= radius_sq) {
            return;
        }
        position = position.normalized() * config_.arena_radius;
    }

    ArenaConfig config_;
    std::vector<Combatant> combatants_;
    core::EventBus events_;
    std::uint32_t next_id_{};
};

} // namespace scandium::game
