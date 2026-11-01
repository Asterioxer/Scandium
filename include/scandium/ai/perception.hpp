#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

#include "scandium/math/ray.hpp"
#include "scandium/math/vec3.hpp"

namespace scandium::ai {

enum class StimulusType : std::uint8_t {
    Visual,
    Acoustic,
    Damage,
    Alert
};

struct Stimulus {
    StimulusType type{StimulusType::Visual};
    std::uint32_t emitter_id{0};
    math::Vec3f position{};
    float intensity{1.0f};
    float timestamp{0.0f};
};

struct VisionConfig {
    float fov_degrees{120.0f};
    float view_distance{30.0f};
    float peripheral_radius{3.0f}; // 360-degree awareness within close proximity
};

struct HearingConfig {
    float hearing_threshold{0.1f};
    float attenuation_rate{0.05f}; // Attenuation factor per meter squared
};

struct PerceivedEntity {
    std::uint32_t entity_id{0};
    math::Vec3f last_known_position{};
    math::Vec3f last_known_velocity{};
    float confidence{1.0f}; // 1.0 = direct line of sight, decays to 0.0
    float time_since_sensed{0.0f};
    StimulusType sense_channel{StimulusType::Visual};
};

class PerceptionSystem {
public:
    using OcclusionRaycast = std::function<bool(const math::Ray&, float max_dist)>;

    /**
     * Determines whether an observer can visually perceive a target.
     * Evaluates distance, FOV cone dot product, peripheral radius, and optional line-of-sight raycast.
     */
    [[nodiscard]] static bool can_see(
        const math::Vec3f& observer_pos,
        const math::Vec3f& observer_forward,
        const math::Vec3f& target_pos,
        const VisionConfig& config,
        const OcclusionRaycast& is_occluded = nullptr) noexcept {

        const math::Vec3f diff = target_pos - observer_pos;
        const float dist_sq = diff.length_squared();

        if (dist_sq <= config.peripheral_radius * config.peripheral_radius) {
            // Within immediate peripheral proximity
            if (is_occluded) {
                const float dist = std::sqrt(dist_sq);
                if (dist > 1e-4f && is_occluded({observer_pos, diff / dist}, dist)) {
                    return false;
                }
            }
            return true;
        }

        if (dist_sq > config.view_distance * config.view_distance) {
            return false;
        }

        const float dist = std::sqrt(dist_sq);
        const math::Vec3f dir = diff / dist;

        // Half-angle cosine threshold
        constexpr float deg_to_rad = 3.14159265f / 180.0f;
        const float cos_half_fov = std::cos(config.fov_degrees * 0.5f * deg_to_rad);

        const float angle_dot = observer_forward.normalized().dot(dir);
        if (angle_dot < cos_half_fov) {
            return false;
        }

        // Line-of-sight obstruction check
        if (is_occluded && is_occluded({observer_pos, dir}, dist)) {
            return false;
        }

        return true;
    }

    /**
     * Determines whether an observer can hear an acoustic emission based on distance attenuation.
     */
    [[nodiscard]] static bool can_hear(
        const math::Vec3f& observer_pos,
        const Stimulus& acoustic_event,
        const HearingConfig& config) noexcept {

        const float dist_sq = (acoustic_event.position - observer_pos).length_squared();
        // Intensity decay: I = I_0 / (1 + alpha * d^2)
        const float perceived_intensity = acoustic_event.intensity / (1.0f + config.attenuation_rate * dist_sq);

        return perceived_intensity >= config.hearing_threshold;
    }
};

/**
 * Perception memory state attached to simulated entities.
 */
class PerceptionMemory {
public:
    void register_sensed(std::uint32_t id, const math::Vec3f& pos, const math::Vec3f& vel, StimulusType channel) {
        for (auto& entry : known_entities_) {
            if (entry.entity_id == id) {
                entry.last_known_position = pos;
                entry.last_known_velocity = vel;
                entry.confidence = 1.0f;
                entry.time_since_sensed = 0.0f;
                entry.sense_channel = channel;
                return;
            }
        }
        known_entities_.push_back({id, pos, vel, 1.0f, 0.0f, channel});
    }

    void update(float dt, float memory_decay_seconds = 5.0f) {
        for (auto it = known_entities_.begin(); it != known_entities_.end();) {
            it->time_since_sensed += dt;
            it->confidence = std::max(0.0f, 1.0f - (it->time_since_sensed / memory_decay_seconds));

            if (it->time_since_sensed >= memory_decay_seconds) {
                it = known_entities_.erase(it);
            } else {
                ++it;
            }
        }
    }

    [[nodiscard]] const std::vector<PerceivedEntity>& known_entities() const noexcept {
        return known_entities_;
    }

    [[nodiscard]] const PerceivedEntity* find(std::uint32_t id) const noexcept {
        for (const auto& entry : known_entities_) {
            if (entry.entity_id == id) return &entry;
        }
        return nullptr;
    }

    void clear() noexcept {
        known_entities_.clear();
    }

private:
    std::vector<PerceivedEntity> known_entities_;
};

} // namespace scandium::ai
